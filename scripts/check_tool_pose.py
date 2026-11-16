#!/usr/bin/env python3
"""Offline static tool clearance from an existing arm/combined JSONL recording."""
import argparse
import json
from pathlib import Path
import yaml

from record_observations import ROOT, create_run, write_json
from astra_on_franka.perception.tool_clearance import check_tool_pose


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--observations',type=Path,required=True)
    parser.add_argument('--sample',type=int,default=0,help='Zero-based JSONL row')
    parser.add_argument('--geometry',type=Path,default=ROOT/'config/tool_geometry.yaml')
    parser.add_argument('--workspace',type=Path,default=ROOT/'config/workspace.yaml')
    parser.add_argument('--holding-cube',action='store_true')
    args=parser.parse_args(argv)
    if args.sample < 0: parser.error('sample must be nonnegative')
    row=None
    with args.observations.open() as stream:
        for i,line in enumerate(stream):
            if i==args.sample:
                row=json.loads(line); break
    if row is None: parser.error('Sample not found')
    arm=row.get('arm',row)
    geometry=yaml.safe_load(args.geometry.read_text())
    workspace=yaml.safe_load(args.workspace.read_text())
    try:
        if arm.get('valid') is not True or arm.get('has_errors') is not False:
            raise ValueError('Recorded arm sample is invalid or reports errors')
        result=check_tool_pose(arm['O_T_EE'],geometry,workspace,holding_cube=args.holding_cube)
    except (ValueError,KeyError,TypeError) as exc:
        parser.error(str(exc))
    run=create_run(ROOT/'data/calibration',prefix='tool_check')
    write_json(run/'result.json',result)
    write_json(run/'inputs.json',{'arm':arm,'geometry':geometry,'workspace':workspace,
                                'source':str(args.observations),'row':args.sample})
    print(json.dumps(result,indent=2,allow_nan=False))
    print(f'Saved {run}')
    return 0 if result['static_tool_envelope_passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
