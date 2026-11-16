#!/usr/bin/env python3
"""Offline point-envelope audit. Not full-body collision or trajectory validation."""
import argparse
from pathlib import Path

import numpy as np
import yaml

from record_observations import ROOT, create_run, write_json


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, default=ROOT / 'config/workspace.yaml')
    parser.add_argument('--cube-config', type=Path, default=ROOT / 'config/cube.yaml')
    args = parser.parse_args(argv)
    workspace = yaml.safe_load(args.workspace.read_text())
    cube = yaml.safe_load(args.cube_config.read_text())
    T = np.asarray(workspace['calibration']['robot_base_T_reference_tag'], float)
    if (T.shape != (4,4) or not np.isfinite(T).all()
            or not np.allclose(T[3], [0,0,0,1])
            or not np.allclose(T[:3,:3].T @ T[:3,:3], np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(T[:3,:3]),1,atol=1e-6)):
        parser.error('Invalid rigid transform')
    if workspace['registration_provenance']['source_frame'] != f"apriltag_{cube['reference_tag_id']}":
        parser.error('Reference tag does not match registration')
    envelope = workspace['proposed_envelopes_m']
    # Robot X=forward, Y=left. Lab lateral coordinate is positive right.
    lower = np.array([envelope['gripper_forward']['min'], -envelope['lateral']['max']])
    upper = np.array([envelope['gripper_forward']['max'], -envelope['lateral']['min']])
    results = {}
    for name, xyz in cube['board_mapping']['corners_reference_m'].items():
        p = T[:3,:3] @ np.asarray(xyz) + T[:3,3]
        violations = []
        for i, axis in enumerate(('X','Y')):
            if p[i] < lower[i]: violations.append(f'{axis} below minimum by {(lower[i]-p[i])*1000:.1f} mm')
            if p[i] > upper[i]: violations.append(f'{axis} above maximum by {(p[i]-upper[i])*1000:.1f} mm')
        if p[2] > envelope['ceiling_above_tabletop']:
            violations.append('Above ceiling')
        results[name] = {'cube_tag_center_base_m': p.tolist(), 'violations': violations}
        print(f'{name}: base XYZ {np.round(p,4).tolist()}; {violations or "within tested point limits"}')
    run = create_run(ROOT / 'data/calibration', prefix='workspace_audit')
    write_json(run / 'audit.json', {'corners': results, 'workspace': workspace,
        'board_mapping': cube['board_mapping'], 'motion_authorized': False,
        'note': 'Approximate point checks only. Cube-tag point is not gripper TCP. No tool size, arm links, table collision, self-collision, uncertainty margin or path checks.'})
    print(f'Saved {run}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
