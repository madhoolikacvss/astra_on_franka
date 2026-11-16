"""Stage-1 offline point checks; never authorize robot motion."""
import numpy as np

from .board import BoardMapping, ORDER


def check_goal(uv, cube, workspace):
    uv = np.asarray(uv, float)
    if uv.shape != (2,) or not np.isfinite(uv).all():
        raise ValueError('Goal must contain two finite normalized coordinates')
    mapping = cube['board_mapping']
    if (mapping['method'] != 'homography_reference_xy'
            or mapping['reference_tag_id'] != cube['reference_tag_id']
            or mapping['cube_tag_id'] != cube['cube_tag_id']
            or workspace['registration_provenance']['source_frame'] != f"apriltag_{cube['reference_tag_id']}"
            or workspace['registration_provenance']['target_frame'] != 'robot_base'):
        raise ValueError('Calibration frame/method mismatch')
    T = np.asarray(workspace['calibration']['robot_base_T_reference_tag'], float)
    if (T.shape != (4,4) or not np.isfinite(T).all()
            or not np.allclose(T[3], [0,0,0,1], atol=1e-8, rtol=0)
            or not np.allclose(T[:3,:3].T @ T[:3,:3], np.eye(3), atol=1e-6, rtol=0)
            or not np.isclose(np.linalg.det(T[:3,:3]), 1, atol=1e-6, rtol=0)):
        raise ValueError('Invalid rigid registration')
    margin = float(workspace['goal_check']['board_center_clearance_m'])
    if not np.isfinite(margin) or margin < 0:
        raise ValueError('Board margin must be finite and nonnegative')
    board = BoardMapping(mapping['corners_reference_m'])
    corners = np.asarray([mapping['corners_reference_m'][k] for k in ORDER], float)
    homogeneous = np.linalg.solve(board.matrix, [*uv, 1])
    if abs(homogeneous[2]) < 1e-9:
        raise ValueError('Goal maps to a projective singularity')
    xy = homogeneous[:2] / homogeneous[2]
    # Fit the tag-center resting plane; do not invent a gripper TCP or cube-center offset.
    plane, *_ = np.linalg.lstsq(np.column_stack((corners[:,:2], np.ones(4))), corners[:,2], rcond=None)
    reference = np.array([*xy, np.dot([*xy, 1], plane)])
    base = T[:3,:3] @ reference + T[:3,3]
    polygon = corners[:,:2]
    signed_area = sum(np.linalg.det(np.stack((polygon[i], polygon[(i+1)%4]))) for i in range(4))
    winding = np.sign(signed_area)
    boundaries, reasons = {}, []
    for i, name in enumerate(('near','right','far','left')):
        edge = polygon[(i+1)%4] - polygon[i]
        distance = float(winding * np.linalg.det(np.stack((edge, xy-polygon[i]))) / np.linalg.norm(edge))
        boundaries[name] = {'signed_distance_m': distance, 'required_margin_m': margin,
                            'remaining_clearance_m': distance-margin}
        if distance < margin - 1e-9:
            reasons.append(f'board_{name}: clearance {distance*1000:.1f} mm below required {margin*1000:.1f} mm')
    if np.any(uv < 0) or np.any(uv > 1):
        reasons.append('normalized_goal_outside_unit_square')
    e = workspace['proposed_envelopes_m']
    limits = np.asarray([e['gripper_forward']['min'], e['gripper_forward']['max'],
                         e['lateral']['min'], e['lateral']['max'], e['ceiling_above_tabletop']],float)
    if not np.isfinite(limits).all() or limits[0] >= limits[1] or limits[2] >= limits[3]:
        raise ValueError('Invalid workspace limits')
    # Robot Y positive left, lab lateral positive right.
    distances = {'rear': base[0]-limits[0], 'front': limits[1]-base[0],
                 'left': -limits[2]-base[1], 'right': base[1]+limits[3],
                 'ceiling': limits[4]-base[2]}
    for name, distance in distances.items():
        if distance < -1e-9:
            reasons.append(f'workspace_{name}: point exceeds boundary by {-distance*1000:.1f} mm')
    return {'schema_version': 1, 'stage1_passed': not reasons, 'motion_authorized': False,
        'goal_normalized': uv.tolist(), 'tracked_point': f"apriltag_{cube['cube_tag_id']}_center",
        'proposed_tag_position_reference_m': reference.tolist(),
        'proposed_tag_position_robot_base_m': base.tolist(),
        'board_boundaries': boundaries,
        'workspace_boundary_distances_m': {k:float(v) for k,v in distances.items()},
        'rejection_reasons': reasons, 'board_calibration_id': mapping['calibration_id'],
        'registration_quality': workspace['registration_provenance']['quality'],
        'cube_side_length_m': workspace['task_object']['side_length_m'],
        'height_method': 'Least-squares resting tag-center plane from four captured XYZ corners',
        'scope': 'Board distances are perpendicular distances to edge lines in reference XY. 50 mm margin applies to captured tag-center placement region. Workspace checks use point only, without added margin. No cube-volume, tool, arm, table-surface, reachability or path checks.'}
