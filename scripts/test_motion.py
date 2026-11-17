#!/usr/bin/env python3
"""Preview-first single Cartesian preset with live state supervision. No automatic retries."""
import argparse
import json
from pathlib import Path
import sys
import time
import numpy as np
import yaml

from record_observations import ROOT,create_run,write_json
from astra_on_franka.hardware.arm_observer import ArmObserver
from astra_on_franka.hardware.motion_test import (
    PRESETS, GUARD_M, STALE_S, TIMEOUT_S, Monitor, target_pose, pose_guard,
    angle_between, validate_sample,
)


def set_dynamics(robot, franky):
    # Explicit absolute caps, with 0.05 global multiplier. Values here are BEFORE scaling.
    factor=.05
    caps={'translation_velocity_limit':.005, 'translation_acceleration_limit':.01,
          'translation_jerk_limit':.05, 'rotation_velocity_limit':np.deg2rad(1),
          'rotation_acceleration_limit':np.deg2rad(2), 'rotation_jerk_limit':np.deg2rad(10),
          'elbow_velocity_limit':.02,'elbow_acceleration_limit':.04,'elbow_jerk_limit':.2}
    for name,value in caps.items(): getattr(robot,name).set(value/factor)
    for name,value in (('joint_velocity_limit',.02),('joint_acceleration_limit',.04),('joint_jerk_limit',.2)):
        getattr(robot,name).set([value/factor]*7)
    robot.relative_dynamics_factor=franky.RelativeDynamicsFactor(velocity=factor,acceleration=factor,jerk=factor)
    return {**caps,'joint_velocity_rad_s':.02,'joint_acceleration_rad_s2':.04,'joint_jerk_rad_s3':.2,
            'relative_factor':factor}


def supervised_move(robot,observer,start,target,workspace,motion,*,cancel_event=None,on_observation=None,
                    session_start=None,session_joint_limit=None):
    monitor=Monitor(observer,start,target,workspace,on_observation=on_observation,
                    session_start=session_start,session_joint_limit=session_joint_limit)
    error=None; stop_error=None; final=None; submitted=False
    monitor.thread.start()
    try:
        if not monitor.ready.wait(2.): raise RuntimeError('No initial monitored observation')
        monitor.check()
        latest=monitor.samples[-1]
        if cancel_event is not None and cancel_event.is_set(): raise RuntimeError('Operator/agent stop requested')
        if (np.linalg.norm(np.asarray(latest['ee_position_base_m'])-np.asarray(start['ee_position_base_m']))>.0005
                or angle_between(np.asarray(latest['O_T_EE'])[:3,:3],np.asarray(start['O_T_EE'])[:3,:3])>np.deg2rad(.1)):
            raise RuntimeError('Arm moved since preview; rerun from new state')
        submitted=True  # Stop even if submission raises after partially starting control.
        started=time.monotonic()
        robot.move(motion,asynchronous=True)
        while True:
            if cancel_event is not None and cancel_event.is_set(): raise RuntimeError('Operator/agent stop requested')
            monitor.check()
            if time.monotonic()-started>TIMEOUT_S: raise RuntimeError('Motion timeout')
            if robot.join_motion(timeout=.005): break
        monitor.check()
        # Require new observations AFTER motion completion, then verify settling.
        count=len(monitor.samples); until=time.monotonic()+1.
        while len(monitor.samples)<count+10:
            if cancel_event is not None and cancel_event.is_set(): raise RuntimeError('Operator/agent stop requested')
            monitor.check()
            if time.monotonic()>until: raise RuntimeError('No settled final observations')
            time.sleep(.005)
        final=monitor.samples[-1]
        actual=np.asarray(final['O_T_EE'])
        if np.linalg.norm(actual[:3,3]-target[:3,3])>.001: raise RuntimeError('Final position error exceeds 1 mm')
        if angle_between(actual[:3,:3],target[:3,:3])>np.deg2rad(.3): raise RuntimeError('Final rotation error exceeds 0.3 degrees')
        if np.max(np.abs(final['joint_velocities_rad_s']))>.005: raise RuntimeError('Arm did not settle')
    except BaseException as exc:
        error=f'{type(exc).__name__}: {exc}'
    finally:
        if submitted:
            try: robot.stop()
            except BaseException as exc: stop_error=f'{type(exc).__name__}: {exc}'
        try: monitor.close()
        except Exception as exc:
            error=error or str(exc)
    return {'status':'failed' if error or stop_error else 'completed', 'error':error,
            'stop_error':stop_error,'command_submitted':submitted,'final':final},monitor.samples


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--preset',choices=sorted(PRESETS),default='tz+')
    parser.add_argument('--robot-host',default='172.16.0.2')
    parser.add_argument('--workspace',type=Path,default=ROOT/'config/workspace.yaml')
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--confirm-clear-path',action='store_true',help='Operator has inspected full arm, tool and cable clearance and has physical stop access')
    parser.add_argument('--observations',type=Path,help='Offline preview from first arm/combined JSONL row; never executable')
    args=parser.parse_args(argv)
    workspace=yaml.safe_load(args.workspace.read_text())
    if args.execute:
        if args.observations: parser.error('Saved observations cannot authorize motion')
        if workspace.get('motion_enabled') is not True: parser.error('workspace.yaml motion_enabled is false; execution remains locked')
        if not args.confirm_clear_path: parser.error('Execution requires --confirm-clear-path after physical inspection')
        parser.error('Arm execution blocked: verified FR3 model and joint-trajectory/collision preflight are not implemented; use preview')
        import resource
        if resource.getrlimit(resource.RLIMIT_RTPRIO)[0]<1: parser.error('Realtime priority is unavailable')
    run=create_run(ROOT/'data/motion_tests',prefix='motion_test')
    observer=robot=None; samples=[]; result={'status':'failed','error':None}
    try:
        if args.observations:
            with args.observations.open() as file: row=json.loads(next(file))
            start=row.get('arm',row)
        else:
            import franky
            robot=franky.Robot(args.robot_host)
            if robot.is_in_control: raise RuntimeError('Robot already in control; refusing to take over')
            observer=ArmObserver(args.robot_host,robot_factory=lambda _:robot)
            start=observer.read()
        if not start['valid'] or start['has_errors']: raise RuntimeError('Invalid or error starting state')
        if 'idle' not in start['robot_mode'].lower(): raise RuntimeError('Starting robot mode is not Idle')
        if np.max(np.abs(start['joint_velocities_rad_s']))>.005: raise RuntimeError('Starting arm is not stationary')
        target=target_pose(start['O_T_EE'],args.preset)
        reasons=[]
        for fraction in np.linspace(0,1,21):
            _,violations=pose_guard(target_pose(start['O_T_EE'],args.preset,fraction),start['F_T_EE'],workspace)
            reasons.extend(violations)
        preview={'preset':args.preset,'start':start,'target_O_T_EE':target.tolist(),
            'workspace':workspace,'offline':bool(args.observations),'guard_margin_m':GUARD_M,
            'rejection_reasons':sorted(set(reasons)),
            'target_rotation_projection':'Closest proper rotation from measured rounded matrix via SVD; raw observation preserved',
            'scope':'EE and flange POINT guards only. No full-arm/tool-volume collision enforcement. Base-axis rotation about configured EE origin; not fingertips. No gripper or camera commands.'}
        write_json(run/'preview.json',preview)
        print('Current EE XYZ:',start['ee_position_base_m'])
        print('Proposed EE XYZ:',target[:3,3].tolist())
        print('Preset:',args.preset,'(t: 50 mm, r: 1 degree, robot-base axes)')
        print('Checks:',preview['rejection_reasons'] or 'EE/flange point guards passed')
        if reasons: raise RuntimeError('Preview failed point guards')
        if not args.execute:
            result={'status':'preview_only','error':None,'command_submitted':False}
        else:
            limits=set_dynamics(robot,franky); write_json(run/'dynamics.json',limits)
            # Recheck after setup; do not execute from an old pose or stale preview.
            time.sleep(.02); fresh=observer.read()
            validate_sample(fresh,start,target,workspace)
            motion=franky.CartesianMotion(franky.Affine(target),franky.ReferenceType.Absolute)
            # C++ time condition remains effective if Python supervision stalls.
            motion.add_reaction(franky.Reaction(franky.Measure.ABS_TIME > TIMEOUT_S,
                                               franky.CartesianStopMotion()))
            result,samples=supervised_move(robot,observer,start,target,workspace,motion)
    except BaseException as exc:
        result={'status':'failed','error':f'{type(exc).__name__}: {exc}'}
    finally:
        with (run/'observations.jsonl').open('x',encoding='utf-8') as file:
            for sample in samples: file.write(json.dumps(sample,allow_nan=False)+'\n')
        if observer is not None: observer.close()
        write_json(run/'summary.json',result)
    print(f"{result['status']}: {run}")
    if result.get('error'): print(result['error'],file=sys.stderr)
    if result.get('stop_error'): print('STOP FAILED: '+result['stop_error'],file=sys.stderr)
    return 1 if result['status']=='failed' else 0


if __name__=='__main__':
    raise SystemExit(main())
