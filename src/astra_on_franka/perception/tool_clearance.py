"""Static conservative tool-box envelope checks, no robot connections."""
from itertools import product
import numpy as np


def rigid(value):
    t = np.asarray(value, float)
    if (t.shape != (4,4) or not np.isfinite(t).all()
            or not np.allclose(t[3], [0,0,0,1], atol=1e-6, rtol=0)
            or not np.allclose(t[:3,:3].T @ t[:3,:3], np.eye(3), atol=1e-4, rtol=0)
            or not np.isclose(np.linalg.det(t[:3,:3]), 1, atol=1e-4, rtol=0)):
        raise ValueError('Expected finite rigid 4x4 transform')
    return t


def vertices(lo, hi):
    lo, hi = np.asarray(lo,float), np.asarray(hi,float)
    if lo.shape != (3,) or hi.shape != (3,) or not np.isfinite([lo,hi]).all() or np.any(hi <= lo):
        raise ValueError('Box needs finite XYZ min < max on every axis')
    return np.asarray(list(product(*zip(lo,hi))))


def check_tool_pose(base_T_ee, geometry, workspace, *, holding_cube=False):
    result = {'schema_version':1, 'static_tool_envelope_passed':False,
              'motion_authorized':False, 'holding_cube':holding_cube,
              'incomplete_reasons':[], 'rejection_reasons':[], 'components':{},
              'scope':'Static boxes against workspace planes and a conservative global tabletop plane. No arm links, self-collision, obstacles inside the envelope, trajectory, reachability, or grasp-contact checks. Board goal margin remains a separate stage-1 check.'}
    missing = result['incomplete_reasons']
    if geometry.get('frame') != 'configured_ee':
        raise ValueError('Tool geometry must use configured_ee coordinates')
    if geometry.get('coverage_confirmed') is not True:
        missing.append('Tool component coverage has not been confirmed')
    boxes = geometry.get('boxes', {})
    for name in ('gripper','adapter','wrist_camera'):
        if name not in boxes:
            missing.append(f'Missing component: {name}')
    for name, box in boxes.items():
        if box.get('min_m') is None or box.get('max_m') is None:
            missing.append(f'Missing outer bounds: {name}')
    if geometry.get('padding_m') is None:
        missing.append('Measurement padding is unset')
    if holding_cube and geometry.get('configured_ee_T_cube_center') is None:
        missing.append('Held cube attachment transform is unset')
    T = rigid(base_T_ee)
    if missing:
        return result
    padding = float(geometry['padding_m'])
    if not np.isfinite(padding) or padding < 0:
        raise ValueError('Padding must be finite and nonnegative')
    local = {name:vertices(box['min_m'],box['max_m']) for name,box in boxes.items()}
    # Padding each box on every local axis conservatively contains isotropic padding.
    local = {name:vertices(np.min(v,axis=0)-padding,np.max(v,axis=0)+padding) for name,v in local.items()}
    if holding_cube:
        side = float(workspace['task_object']['side_length_m'])
        if not np.isfinite(side) or side <= 0:
            raise ValueError('Cube side length must be positive and finite')
        attachment = rigid(geometry['configured_ee_T_cube_center'])
        v = vertices(np.full(3,-side/2-padding),np.full(3,side/2+padding))
        local['held_cube'] = v @ attachment[:3,:3].T + attachment[:3,3]
    envelope = workspace['proposed_envelopes_m']
    floor = float(workspace['measurement_frame']['tabletop_height_m'])
    lo = np.array([envelope['gripper_forward']['min'], -envelope['lateral']['max'],floor])
    hi = np.array([envelope['gripper_forward']['max'], -envelope['lateral']['min'],envelope['ceiling_above_tabletop']+floor])
    if not np.isfinite([lo,hi]).all() or np.any(hi<=lo):
        raise ValueError('Invalid envelope bounds')
    for name,v in local.items():
        world = v @ T[:3,:3].T + T[:3,3]
        minimum,maximum=world.min(0),world.max(0)
        distances = dict(zip(('rear','right','tabletop'),(minimum-lo).tolist()))
        distances.update(zip(('front','left','ceiling'),(hi-maximum).tolist()))
        result['components'][name] = {'minimum_base_m':minimum.tolist(),'maximum_base_m':maximum.tolist(),
                                    'signed_clearance_m':distances}
        for boundary,distance in distances.items():
            if distance < -1e-9:
                result['rejection_reasons'].append(f'{name}: exceeds {boundary} by {-distance*1000:.1f} mm')
    result['static_tool_envelope_passed'] = not result['rejection_reasons']
    result['padding_m'] = padding
    return result
