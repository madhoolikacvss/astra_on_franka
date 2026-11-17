"""Bounded supervised motion-test primitives, independent of Franky imports."""
import threading
import time
import numpy as np

from astra_on_franka.perception.tool_clearance import rigid

STEP_M = 0.050
STEP_RAD = np.deg2rad(1.)
GUARD_M = 0.05
PERIOD_S = 0.01
STALE_S = 0.10
TIMEOUT_S = 30.
PRESETS = {f'{kind}{axis}{sign}': (kind, i, direction)
           for kind in ('t','r') for i,axis in enumerate('xyz')
           for sign,direction in (('+',1),('-',-1))}


def orthogonal(r):
    u, _, vt = np.linalg.svd(r)
    return u @ np.diag([1,1,np.linalg.det(u@vt)]) @ vt


def rotation_delta(axis, angle):
    v=np.eye(3)[axis]; k=np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
    return np.eye(3)+np.sin(angle)*k+(1-np.cos(angle))*(k@k)


def angle_between(a,b):
    return float(np.arccos(np.clip((np.trace(orthogonal(a).T@orthogonal(b))-1)/2,-1,1)))


def target_pose(start, preset, fraction=1.):
    start=rigid(start); target=start.copy()
    kind,axis,direction=PRESETS[preset]
    target[:3,:3]=orthogonal(start[:3,:3])
    if kind=='t': target[axis,3]+=direction*STEP_M*fraction
    else: target[:3,:3]=rotation_delta(axis,direction*STEP_RAD*fraction)@target[:3,:3]
    return target


def pose_guard(pose, flange_ee, workspace):
    pose=rigid(pose); flange=pose@np.linalg.inv(rigid(flange_ee))
    e=workspace['proposed_envelopes_m']; f=float(workspace['measurement_frame']['tabletop_height_m'])
    distances={}; reasons=[]
    for name,p,rear in (('configured_ee',pose[:3,3],e['gripper_forward']['min']),
                        ('flange',flange[:3,3],e['arm_links_forward']['min'])):
        lo=np.array([rear,-e['lateral']['max'],f])+GUARD_M
        hi=np.array([e['gripper_forward']['max'],-e['lateral']['min'],f+e['ceiling_above_tabletop']])-GUARD_M
        if not np.isfinite([lo,hi]).all() or np.any(hi<=lo): raise ValueError('Invalid guard limits')
        d=dict(zip(('rear','right','floor'),(p-lo).tolist()))
        d.update(zip(('front','left','ceiling'),(hi-p).tolist()));distances[name]=d
        reasons += [f'{name}: {key} guard crossed by {-v*1000:.2f} mm' for key,v in d.items() if v<0]
    return distances,reasons


def joint_excursion_reasons(sample,reference,limit,label):
    current=np.asarray(sample['joint_positions_rad'],dtype=float)
    baseline=np.asarray(reference['joint_positions_rad'],dtype=float)
    if current.shape!=(7,) or baseline.shape!=(7,) or not np.isfinite([current,baseline]).all():
        raise ValueError('Invalid joint excursion observations')
    if not np.isfinite(limit) or limit<=0:raise ValueError('Invalid joint excursion limit')
    delta=current-baseline
    return [f'{label}: joint {i+1} excursion {delta[i]:+.6f} rad exceeds {limit:.6f} rad'
            for i in range(7) if abs(delta[i])>limit]


def validate_sample(sample,start,target,workspace,*,session_start=None,session_joint_limit=None):
    if not sample['valid'] or sample['has_errors']: raise RuntimeError('Invalid/error robot state')
    if sample['robot_mode'].lower().split('.')[-1] not in ('idle','move'):
        raise RuntimeError(f"Unexpected robot mode: {sample['robot_mode']}")
    if sample['read_duration_ms'] > STALE_S*1000: raise RuntimeError('Robot state read exceeded freshness limit')
    if time.monotonic_ns()-sample['host_read_end_monotonic_ns'] > STALE_S*1e9: raise RuntimeError('Stale host observation')
    pose=rigid(sample['O_T_EE']); distances,reasons=pose_guard(pose,sample['F_T_EE'],workspace)
    if not np.allclose(sample['F_T_EE'],start['F_T_EE'],atol=1e-8,rtol=0): reasons.append('Configured tool transform changed')
    reasons.extend(joint_excursion_reasons(sample,start,.05,'Per-move fallback guard'))
    if session_start is not None:
        reasons.extend(joint_excursion_reasons(sample,session_start,session_joint_limit,'Session guard'))
    if np.max(np.abs(sample['joint_velocities_rad_s']))>.08: reasons.append('Joint velocity exceeded 0.08 rad/s')
    a=np.asarray(start['O_T_EE']); delta=target[:3,3]-a[:3,3]
    fraction=np.clip(np.dot(pose[:3,3]-a[:3,3],delta)/max(np.dot(delta,delta),1e-15),0,1)
    if np.linalg.norm(pose[:3,3]-(a[:3,3]+fraction*delta))>.002: reasons.append('EE left 2 mm translation corridor')
    if angle_between(a[:3,:3],pose[:3,:3])>angle_between(a[:3,:3],target[:3,:3])+np.deg2rad(.5): reasons.append('Rotation exceeded preset excursion')
    if reasons: raise RuntimeError('; '.join(reasons))
    return distances


class Monitor:
    """Poll current state, not delayed callback queues. No disk/camera work here."""
    def __init__(self,observer,start,target,workspace,on_observation=None,session_start=None,session_joint_limit=None):
        self.observer,self.start,self.target,self.workspace=observer,start,target,workspace
        self.samples=[]; self.error=None; self.last_ok=time.monotonic(); self.done=threading.Event()
        self.ready=threading.Event(); self.thread=threading.Thread(target=self.run,daemon=True)
        self.on_observation=on_observation
        self.session_start=session_start;self.session_joint_limit=session_joint_limit

    def run(self):
        try:
            while not self.done.is_set():
                t=time.monotonic(); sample=self.observer.read();self.samples.append(sample)
                if len(self.samples)>10000: raise RuntimeError('Observation buffer limit')
                validate_sample(sample,self.start,self.target,self.workspace,
                                session_start=self.session_start,session_joint_limit=self.session_joint_limit)
                if self.on_observation is not None: self.on_observation(sample)
                if len(self.samples)>1:
                    previous=self.samples[-2]
                    robot_dt=(sample['robot_time_ms']-previous['robot_time_ms'])/1000
                    host_dt=(sample['host_read_end_monotonic_ns']-previous['host_read_end_monotonic_ns'])/1e9
                    if robot_dt<=0 or host_dt>STALE_S or abs(host_dt-robot_dt)>STALE_S:
                        raise RuntimeError('State clock stopped, jumped, or observation gap exceeded limit')
                    current_pose=np.asarray(sample['O_T_EE']); previous_pose=np.asarray(previous['O_T_EE'])
                    if np.linalg.norm(current_pose[:3,3]-previous_pose[:3,3])/robot_dt>.015:
                        raise RuntimeError('Observed EE speed exceeded 15 mm/s')
                    if angle_between(previous_pose[:3,:3],current_pose[:3,:3])/robot_dt>np.deg2rad(3):
                        raise RuntimeError('Observed EE rotation rate exceeded 3 degrees/s')
                self.last_ok=time.monotonic();self.ready.set()
                self.done.wait(max(0,PERIOD_S-(time.monotonic()-t)))
        except Exception as exc:
            self.error=f'{type(exc).__name__}: {exc}';self.ready.set()

    def check(self):
        if self.error: raise RuntimeError(self.error)
        if time.monotonic()-self.last_ok>STALE_S: raise RuntimeError('Observation watchdog expired')

    def close(self):
        self.done.set();self.thread.join(timeout=1.)
        if self.thread.is_alive(): raise RuntimeError('Observation reader did not exit')
