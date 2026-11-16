"""Strict, data-only agent protocol. No evaluation, imports or paths from requests."""
import json
import math
import re
import time
import uuid
from collections import OrderedDict

import numpy as np
from astra_on_franka.hardware.motion_test import PRESETS, target_pose, pose_guard, angle_between


def decode_request(raw, maximum=4096):
    if len(raw)>maximum: raise ValueError('Request too large')
    def unique(pairs):
        result={}
        for k,v in pairs:
            if k in result: raise ValueError('Duplicate JSON key')
            result[k]=v
        return result
    def invalid(value): raise ValueError('Nonfinite JSON number')
    value=json.loads(raw,object_pairs_hook=unique,parse_constant=invalid)
    if not isinstance(value,dict): raise ValueError('Request must be a JSON object')
    return value


class CommandFilter:
    def __init__(self,config,workspace,*,clock=time.monotonic):
        self.config,self.workspace,self.clock=config,workspace,clock
        self.session_id=uuid.uuid4().hex
        self.observations=OrderedDict(); self.seen=set(); self.count=0;self.last_command=-math.inf
        self.start=None;self.fault=None

    def publish(self,observation):
        token=uuid.uuid4().hex
        self.observations[token]=(self.clock(),observation)
        while len(self.observations)>100: self.observations.popitem(last=False)
        if self.start is None: self.start=observation['arm']
        return token

    def accept(self,request,current,*,busy=False):
        required={'schema_version','request_id','session_id','observation_id','action','arguments'}
        if set(request)!=required or type(request['schema_version']) is not int or request['schema_version']!=1:
            raise ValueError('Unexpected request schema or fields')
        identifier=request['request_id']
        if not isinstance(identifier,str) or not re.fullmatch(r'[a-zA-Z0-9_-]{1,64}',identifier):
            raise ValueError('Invalid request_id')
        if request['session_id']!=self.session_id: raise ValueError('Old/wrong controller session')
        if identifier in self.seen: raise ValueError('Replayed request_id')
        if len(self.seen)>=10000: raise ValueError('Session request capacity exhausted; operator restart required')
        self.seen.add(identifier)  # Failed requests are also one-shot.
        action,args=request['action'],request['arguments']
        if not isinstance(action,str) or not isinstance(args,dict): raise ValueError('Invalid action/arguments')
        if action=='stop':
            if args: raise ValueError('Stop takes no arguments')
            self.fault='Stop requested; operator restart required'
            return {'action':'stop'}
        if self.fault: raise ValueError(self.fault)
        if busy: raise ValueError('Controller busy; no command queue')
        if self.count>=self.config['max_commands_per_session']: raise ValueError('Session command budget exhausted')
        if self.clock()-self.last_command<self.config['minimum_command_interval_s']: raise ValueError('Command rate limit')
        token=request['observation_id']
        if not isinstance(token,str) or token not in self.observations: raise ValueError('Unknown observation_id')
        created,observed=self.observations[token]
        if self.clock()-created>self.config['observation_ttl_s']: raise ValueError('Stale observation_id')
        arm=current['arm']; previous=observed['arm']
        if not arm['valid'] or arm['has_errors'] or arm['robot_mode'].lower().split('.')[-1]!='idle':
            raise ValueError('Arm must be valid, error-free and Idle')
        if np.max(np.abs(arm['joint_velocities_rad_s']))>.005: raise ValueError('Arm is not stationary')
        pose=np.asarray(arm['O_T_EE']); old=np.asarray(previous['O_T_EE']); initial=np.asarray(self.start['O_T_EE'])
        if np.linalg.norm(pose[:3,3]-old[:3,3])>.001 or angle_between(pose[:3,:3],old[:3,:3])>np.deg2rad(.3):
            raise ValueError('Pose changed since referenced observation')
        if np.max(np.abs(np.asarray(arm['joint_positions_rad'])-self.start['joint_positions_rad']))>self.config['max_joint_excursion_from_session_start_rad']:
            raise ValueError('Session joint excursion limit')
        if action=='arm_step':
            if set(args)!={'preset'} or not isinstance(args['preset'],str) or args['preset'] not in PRESETS:
                raise ValueError('Only allowlisted arm presets are accepted')
            target=target_pose(pose,args['preset'])
            for fraction in np.linspace(0,1,21):
                _,reasons=pose_guard(target_pose(pose,args['preset'],fraction),arm['F_T_EE'],self.workspace)
                if reasons: raise ValueError('; '.join(reasons))
            if np.linalg.norm(target[:3,3]-initial[:3,3])>self.config['max_translation_from_session_start_m']+1e-9:
                raise ValueError('Session translation limit')
            if angle_between(initial[:3,:3],target[:3,:3])>np.deg2rad(self.config['max_rotation_from_session_start_deg'])+1e-9:
                raise ValueError('Session rotation limit')
            plan={'action':action,'preset':args['preset'],'target_O_T_EE':target.tolist()}
        elif action=='gripper_set':
            if set(args)!={'closed_fraction'} or type(args['closed_fraction']) not in (float,int):
                raise ValueError('Expected numeric closed_fraction only')
            target=float(args['closed_fraction'])
            grip=current.get('gripper');old_grip=observed.get('gripper')
            if not math.isfinite(target) or not 0<=target<=1: raise ValueError('Gripper target outside [0,1]')
            if not grip or not grip.get('valid') or not old_grip or not old_grip.get('valid'):
                raise ValueError('No valid gripper observation')
            if abs(grip['closed_fraction']-old_grip['closed_fraction'])>.02: raise ValueError('Gripper changed since observation')
            if abs(target-grip['closed_fraction'])>self.config['max_gripper_fraction_step']+1e-9:
                raise ValueError('Gripper step exceeds 0.10 closed fraction')
            _,reasons=pose_guard(arm['O_T_EE'],arm['F_T_EE'],self.workspace)
            if reasons: raise ValueError('; '.join(reasons))
            plan={'action':action,'closed_fraction':target}
        else: raise ValueError('Unknown action')
        self.count+=1;self.last_command=self.clock()
        return plan
