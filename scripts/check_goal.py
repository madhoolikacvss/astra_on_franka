#!/usr/bin/env python3
"""Check a normalized board goal offline; never connect to hardware."""
import argparse
import json
from pathlib import Path

import yaml

from record_observations import ROOT, create_run, write_json
from astra_on_franka.perception.goal_check import check_goal


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--goal', type=float, nargs=2, required=True, metavar=('U','V'),
                        help='U left-to-right, V near-to-far; range 0..1')
    parser.add_argument('--workspace', type=Path, default=ROOT / 'config/workspace.yaml')
    parser.add_argument('--cube-config', type=Path, default=ROOT / 'config/cube.yaml')
    args = parser.parse_args(argv)
    workspace = yaml.safe_load(args.workspace.read_text())
    cube = yaml.safe_load(args.cube_config.read_text())
    try:
        result = check_goal(args.goal, cube, workspace)
    except (ValueError, KeyError, TypeError) as exc:
        parser.error(str(exc))
    run = create_run(ROOT / 'data/calibration', prefix='goal_check')
    write_json(run / 'result.json', result)
    write_json(run / 'config_snapshot.json', {'workspace': workspace, 'cube': cube})
    print(json.dumps(result, indent=2, allow_nan=False))
    print(f'Saved {run}')
    return 0 if result['stage1_passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
