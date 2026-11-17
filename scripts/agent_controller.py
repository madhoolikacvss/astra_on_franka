#!/usr/bin/env python3
"""Operator-run JSON inbox bridge. Replay and live preview never execute commands."""
import argparse
from collections import deque
import hashlib
import json
from pathlib import Path
import threading
import time
import uuid

import numpy as np
import yaml

from record_observations import ROOT,create_run,write_json
from test_motion import supervised_move,set_dynamics
from astra_on_franka.agent_protocol import CommandFilter,decode_request
from astra_on_franka.hardware.arm_observer import ArmObserver
from astra_on_franka.hardware.motion_test import TIMEOUT_S,STEP_M,pose_guard
from astra_on_franka.hardware.managed_gripper import ManagedGripper


def atomic_json(path,value):
    temporary=path.with_name(path.name+'.tmp')
    with temporary.open('w',encoding='utf-8') as f: json.dump(value,f,allow_nan=False,indent=2)
    temporary.replace(path)


class ObservationLog:
    def __init__(self,run):
        self.run=run
        self.stats={'observation_error':None,'last_observation_error':None,
                    'observation_failure_count':0,'publication_count':0,
                    'last_publication_unix_ns':None}

    def record(self,error,stage):
        now=time.time_ns();previous=self.stats['observation_error']
        self.stats['observation_error']=error
        if error is not None:
            self.stats['observation_failure_count']+=1
            self.stats['last_observation_error']={'message':error,'stage':stage,'unix_ns':now}
        if error!=previous:
            with (self.run/'observation_events.jsonl').open('a',encoding='utf-8') as f:
                f.write(json.dumps({'unix_ns':now,'event':'failed' if error else 'recovered',
                    'stage':stage,'error':error,'previous_error':previous},allow_nan=False)+'\n')

    def published(self):
        self.stats['publication_count']+=1
        self.stats['last_publication_unix_ns']=time.time_ns()


class Cameras:
    def __init__(self,run):
        self.run=run;self.latest=None;self.error=None;self.done=threading.Event()
        self.stale_after_s=.5
        self.retained_frames=deque()
        self.thread=threading.Thread(target=self.work,daemon=True)

    def retain_frame_set(self,paths):
        """Only prune files created by this publisher in this session's frames dir."""
        directory=(self.run/'frames').resolve()
        paths=tuple(Path(path) for path in paths)
        if any(path.is_symlink() or path.resolve().parent!=directory for path in paths):
            raise RuntimeError('Frame retention path outside session frames directory')
        self.retained_frames.append(paths)
        while len(self.retained_frames)>5:
            for path in self.retained_frames[0]:
                if path.is_symlink() or path.resolve().parent!=directory:
                    raise RuntimeError('Frame retention path changed')
                path.unlink(missing_ok=True)
            self.retained_frames.popleft()

    def work(self):
        from astra_on_franka.hardware.cameras import CameraRig,StreamSettings
        from astra_on_franka.perception.cube_tracker import CubeTracker
        import cv2
        config=yaml.safe_load((ROOT/'config/cameras.yaml').read_text())
        cube=yaml.safe_load((ROOT/'config/cube.yaml').read_text())
        rig=None
        try:
            tracker=CubeTracker(cube);settings=StreamSettings(**config['stream'])
            self.stale_after_s=settings.stale_after_s
            (self.run/'frames').mkdir()
            with CameraRig({k:v['serial'] for k,v in config['cameras'].items()},settings) as rig:
                index=0
                while not self.done.is_set():
                    capture=rig.read();frames=capture['cameras']
                    processing_started=time.monotonic_ns()
                    result,preview=tracker.process(frames[cube['camera']],max_age_s=settings.stale_after_s)
                    save_started=time.monotonic_ns()
                    saved={};written=[]
                    for name,frame in frames.items():
                        stem=self.run/'frames'/f'{index:06d}_{name}'
                        with stem.with_suffix('.npz').open('xb') as f:
                            # Compression delayed publication enough to exhaust the
                            # freshness budget. Preserve all arrays without compression.
                            np.savez(f,**{k:frame[k] for k in ('rgb','depth_raw','depth_aligned')})
                        display=preview if name==cube['camera'] else frame['rgb'][:,:,::-1]
                        if not cv2.imwrite(str(stem.with_suffix('.jpg')),display): raise RuntimeError('Cannot save camera preview')
                        written.extend((stem.with_suffix('.npz'),stem.with_suffix('.jpg')))
                        saved[name]={k:v for k,v in frame.items() if k not in ('rgb','depth_raw','depth_aligned')}
                        saved[name]['rgbd_file']=str(stem.with_suffix('.npz').relative_to(ROOT))
                        saved[name]['preview_file']=str(stem.with_suffix('.jpg').relative_to(ROOT))
                    published=time.monotonic_ns()
                    oldest=min(f['host_received_monotonic_ns'] for f in frames.values())
                    timing={'sequence':index,'processing_ms':(save_started-processing_started)/1e6,
                            'save_ms':(published-save_started)/1e6,
                            'oldest_age_at_publication_ms':(published-oldest)/1e6}
                    self.latest={'cameras':saved,'cube':result,'oldest_receipt_ns':oldest,
                                 'camera_publication':timing,'retained_camera_frame_sets':5}
                    # Publish complete new files before removing the oldest set.
                    # Old snapshot image paths deliberately expire; re-read latest.
                    self.retain_frame_set(written)
                    # Retain timing evidence even if the final status is closed.
                    with (self.run/'camera_publication.jsonl').open('a',encoding='utf-8') as log:
                        log.write(json.dumps(timing,allow_nan=False)+'\n')
                    index+=1;self.done.wait(.01)
        except Exception as exc:self.error=f'{type(exc).__name__}: {exc}'
        finally:
            if rig is not None:write_json(self.run/'camera_diagnostics.json',rig.diagnostics())

    def snapshot(self):
        if self.error:raise RuntimeError(self.error)
        if self.latest is None:raise RuntimeError('Cameras are warming up')
        latest=self.latest  # Keep frame metadata and timing from the same publication.
        now=time.monotonic_ns()
        ages={name:(now-frame['host_received_monotonic_ns'])/1e6
              for name,frame in latest['cameras'].items()}
        if now-latest['oldest_receipt_ns']>self.stale_after_s*1e9:
            raise RuntimeError(f'Camera observations stale: ages_ms={ages}; '
                               f'limit_ms={self.stale_after_s*1000}; '
                               f'publication={latest.get("camera_publication")}')
        return {**latest,'camera_ages_at_snapshot_ms':ages}

    def close(self):
        self.done.set();self.thread.join(timeout=12)
        if self.thread.is_alive():raise RuntimeError('Camera publisher did not stop')


class LiveBackend:
    def __init__(self,workspace,gripper_config,execute):
        import franky
        self.franky=franky;self.workspace=workspace;self.execute=execute
        self.gripper=None;self.observer=None;self.latest=None;self.busy=False
        self.robot=franky.Robot('172.16.0.2')
        if self.robot.is_in_control:raise RuntimeError('Robot already controlled')
        self.observer=ArmObserver('172.16.0.2',robot_factory=lambda _:self.robot)
        try:self.gripper=ManagedGripper(gripper_config)
        except BaseException:self.observer.close();raise

    def observe(self):
        if not self.busy:
            self.latest={'arm':self.observer.read(),'gripper':self.gripper.read()}
        if self.latest is None:raise RuntimeError('No arm observation')
        if time.monotonic_ns()-self.latest['arm']['host_read_end_monotonic_ns']>150_000_000:
            raise RuntimeError('Live arm observation stale')
        return self.latest

    def arm_update(self,sample):
        self.latest={**self.latest,'arm':sample}

    def perform(self,plan,start,cancel,*,session_start=None,session_joint_limit=None):
        if not self.execute:return {'status':'previewed','plan':plan,'hardware_command_sent':False,
            'joint_trajectory_validated':False,
            'safety_scope':'EE/flange point guards and runtime monitoring; no full-body collision planner'}
        if plan['action']=='arm_step':
            set_dynamics(self.robot,self.franky)
            target=np.asarray(plan['target_O_T_EE'])
            motion=self.franky.CartesianMotion(self.franky.Affine(target),self.franky.ReferenceType.Absolute)
            motion.add_reaction(self.franky.Reaction(self.franky.Measure.ABS_TIME>TIMEOUT_S,self.franky.CartesianStopMotion()))
            result,samples=supervised_move(self.robot,self.observer,start['arm'],target,self.workspace,motion,
                cancel_event=cancel,on_observation=self.arm_update,
                session_start=session_start,session_joint_limit=session_joint_limit)
            return {**result,'arm_observations':samples}
        target=plan['closed_fraction'];begin=start['gripper']['closed_fraction'];samples=[]
        try:
            if cancel.is_set():raise RuntimeError('Stop requested before gripper enable')
            self.gripper.enable_at_current_position()
            started=time.monotonic();stable=0
            while time.monotonic()-started<5.:
                if cancel.is_set():raise RuntimeError('Stop requested')
                arm=self.observer.read();grip=self.gripper.read();self.latest={'arm':arm,'gripper':grip}
                if cancel.is_set():raise RuntimeError('Stop requested during observation')
                _,reasons=pose_guard(arm['O_T_EE'],arm['F_T_EE'],self.workspace)
                if arm['has_errors'] or reasons or np.max(np.abs(arm['joint_velocities_rad_s']))>.005:
                    raise RuntimeError('Arm changed or violated bounds during gripper command')
                if np.linalg.norm(np.asarray(arm['ee_position_base_m'])-start['arm']['ee_position_base_m'])>.001:
                    raise RuntimeError('Arm moved during gripper command')
                fraction=begin+np.clip(target-begin,-.1*(time.monotonic()-started),.1*(time.monotonic()-started))
                self.gripper.command(float(fraction));samples.append({'arm':arm,'gripper':grip})
                stable=stable+1 if abs(grip['closed_fraction']-target)<.025 and abs(fraction-target)<1e-6 else 0
                if stable>=3:return {'status':'completed','gripper_observations':samples,'hardware_command_sent':True}
                cancel.wait(.05)
            raise RuntimeError('Gripper did not reach target; possible contact. No retry.')
        except BaseException:
            self.gripper.disable();raise

    def close(self):
        try:self.gripper.close()
        finally:self.observer.close()


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode',choices=('replay','live-preview','live-execute'),default='replay')
    parser.add_argument('--observations',type=Path,help='Replay only: existing arm/combined JSONL; never considered live')
    parser.add_argument('--confirm-clear-path',action='store_true')
    args=parser.parse_args(argv)
    config=yaml.safe_load((ROOT/'config/agent_bridge.yaml').read_text())
    workspace=yaml.safe_load((ROOT/'config/workspace.yaml').read_text())
    grip_config=yaml.safe_load((ROOT/'config/gripper.yaml').read_text())
    execute=args.mode=='live-execute'
    if execute:
        gates=(config['autonomous_execution_enabled'],config['os_isolation_verified'],config['arm_hardware_test_passed'],config['gripper_hardware_test_passed'],workspace['motion_enabled'],grip_config['motion_enabled'],args.confirm_clear_path)
        if not all(x is True for x in gates):parser.error('Operator execution gates are closed; use preview mode')
        import resource
        if resource.getrlimit(resource.RLIMIT_RTPRIO)[0]<1:parser.error('Realtime priority unavailable')
    if args.mode=='replay' and not args.observations:parser.error('Replay requires --observations')
    if args.mode!='replay' and args.observations:parser.error('Live mode cannot use saved observations')
    data=(ROOT/config['data_directory']).resolve();inbox=ROOT/config['request_file']
    if inbox.is_symlink():parser.error('Symlink inbox refused')
    if not data.is_relative_to(ROOT) or not inbox.resolve().is_relative_to(ROOT):parser.error('Paths must stay inside project')
    data.mkdir(parents=True,exist_ok=True)
    # This service must be started by the operator, under a different OS account.
    lock=(data/'controller.lock').open('a+')
    try:
        if sys_platform_linux():
            import fcntl
            fcntl.flock(lock.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
        else:raise RuntimeError('Run the bridge on Linux; lock/permission deployment is Linux-specific')
    except BaseException:lock.close();raise
    run=create_run(data,prefix='session');filter_=CommandFilter(config,workspace)
    backend=cameras=None;cancel=threading.Event();worker=None;outcome={};busy=False;digest=None;fault=None
    last_request=None;inbox_index=0
    observation_log=ObservationLog(run);terminal_error=None
    try:
        if args.mode=='replay':
            with args.observations.open() as f:record=json.loads(next(f))
            replay=record if 'arm' in record else {'arm':record,'gripper':None}
        else:
            backend=LiveBackend(workspace,grip_config,execute);cameras=Cameras(run);cameras.thread.start()
        write_json(run/'metadata.json',{'mode':args.mode,'session_id':filter_.session_id,'bridge_config':config,'workspace':workspace,'gripper_config':grip_config,'translation_step_m':STEP_M})
        print(f'{args.mode}: session {filter_.session_id}; inbox {inbox}',flush=True)
        def work(plan,current):
            try:outcome['result']=backend.perform(plan,current,cancel,
                session_start=filter_.start,session_joint_limit=config['max_joint_excursion_from_session_start_rad']) if backend else {'status':'previewed','plan':plan,'hardware_command_sent':False}
            except BaseException as exc:outcome['result']={'status':'failed','error':f'{type(exc).__name__}: {exc}'}
        while True:
            if worker and not worker.is_alive() and busy:
                busy=False
                if backend:
                    backend.busy=False
                    if cancel.is_set():backend.gripper.disable()
                result=outcome['result']
                if result['status']=='failed':
                    fault=result.get('error') or result.get('stop_error') or 'Command failed';filter_.fault=fault
                    if backend:backend.gripper.disable()
                response={'request_id':last_request,'session_id':filter_.session_id,**result}
                write_json(run/f'response_{filter_.count:04d}_{last_request}.json',response)
                atomic_json(data/'response.json',response)
                worker=None
            observation=None
            observation_stage='arm_and_gripper' if backend else 'replay'
            try:
                observation=backend.observe() if backend else replay
                if cameras:
                    observation_stage='cameras'
                    observation={**observation,**cameras.snapshot()}
            except Exception as exc:
                if busy:cancel.set();fault=str(exc);filter_.fault=fault
                observation_error=f'{type(exc).__name__}: {exc}'
            else:observation_error=None
            observation_log.record(observation_error,observation_stage)
            if observation is not None and observation_error is None:
                token=filter_.publish(observation)
                atomic_json(data/'latest.json',{'schema_version':1,'session_id':filter_.session_id,'observation_id':token,
                    'published_unix_ns':time.time_ns(),'mode':args.mode,'live':args.mode!='replay','busy':busy,'fault':fault,
                    'motion_authorized':False,'observation':observation,
                    'translation_step_m':STEP_M,
                    'note':'Agent cannot authorize motion; only operator-controlled service gates permit execution.'})
                observation_log.published()
            atomic_json(data/'status.json',{'session_id':filter_.session_id,'mode':args.mode,'busy':busy,'fault':fault,
                **observation_log.stats,'commands_used':filter_.count,'heartbeat_unix_ns':time.time_ns()})
            if inbox.is_symlink():raise RuntimeError('Symlink inbox refused')
            with inbox.open('rb') as f:raw=f.read(config['max_request_bytes']+1)
            current_digest=hashlib.sha256(raw).digest()
            if current_digest!=digest:
                digest=current_digest;request=None
                write_json(run/f'inbox_{inbox_index:06d}.json',{'received_unix_ns':time.time_ns(),'bytes_utf8':raw.decode('utf-8',errors='replace')})
                inbox_index+=1
                try:
                    request=decode_request(raw,config['max_request_bytes'])
                    if not request:continue
                    if request.get('action')!='stop' and (observation is None or observation_error):
                        raise ValueError(f'Fresh complete observation unavailable: {observation_error or "No observation returned"}')
                    plan=filter_.accept(request,observation,busy=busy)
                    if plan['action']=='stop':
                        cancel.set();fault=filter_.fault
                        if backend and not busy:backend.gripper.disable()
                        atomic_json(data/'response.json',{'request_id':request['request_id'],'status':'stop_requested','session_id':filter_.session_id})
                    else:
                        cancel.clear();outcome.clear();last_request=request['request_id'];busy=True
                        if backend:backend.busy=True
                        atomic_json(data/'response.json',{'request_id':last_request,'status':'accepted','session_id':filter_.session_id,'plan':plan})
                        worker=threading.Thread(target=work,args=(plan,observation),daemon=True);worker.start()
                except (ValueError,KeyError,TypeError) as exc:
                    rejection={'status':'rejected','request_id':request.get('request_id') if isinstance(request,dict) else None,
                        'reason':str(exc),'session_id':filter_.session_id,'rejected_unix_ns':time.time_ns(),
                        'observation_error':observation_error,
                        'observation_stage':observation_stage if observation_error else None,
                        'last_observation_error':observation_log.stats['last_observation_error']}
                    write_json(run/f'rejection_{inbox_index-1:06d}.json',rejection)
                    atomic_json(data/'response.json',rejection)
            time.sleep(.1)
    except KeyboardInterrupt:pass
    except BaseException as exc:
        terminal_error=f'{type(exc).__name__}: {exc}'
        raise
    finally:
        cancel.set()
        if worker:worker.join(timeout=35)
        errors=[]
        if worker and worker.is_alive():errors.append('Command worker did not stop; use physical stop')
        else:
            if backend:
                try:backend.close()
                except Exception as exc:errors.append(str(exc))
        if cameras:
            try:cameras.close()
            except Exception as exc:errors.append(str(exc))
        final_status={'session_id':filter_.session_id,'mode':args.mode,'status':'closed',
            'fault':fault,'terminal_error':terminal_error,'cleanup_errors':errors,
            'commands_used':filter_.count,**observation_log.stats,'heartbeat_unix_ns':time.time_ns()}
        atomic_json(data/'status.json',final_status)
        write_json(run/'summary.json',{'closed':True,**final_status})
        lock.close()


def sys_platform_linux():
    import sys
    return sys.platform.startswith('linux')


if __name__=='__main__':main()
