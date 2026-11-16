#!/usr/bin/env python3
"""Fit a candidate reference-tag-to-robot-base transform from measured points. Offline only."""
import argparse
from pathlib import Path
import math

import numpy as np
import yaml

from record_observations import ROOT, create_run, write_json
from astra_on_franka.perception.registration import fit_rigid, residuals


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--measurements", type=Path, default=ROOT / "config/registration_points.yaml")
    parser.add_argument("--max-error-mm", type=float, required=True,
                        help="Explicit diagnostic residual limit; not a certified safety tolerance")
    parser.add_argument("--output-root", type=Path, default=ROOT / "data/calibration")
    args = parser.parse_args(argv)
    if not math.isfinite(args.max_error_mm) or args.max_error_mm <= 0:
        parser.error("max-error-mm must be finite and positive")
    data = yaml.safe_load(args.measurements.read_text())
    if data.get("units") != "meters" or data.get("target_frame") != "robot_base":
        parser.error("Expected meter coordinates in the actual robot_base frame")
    cfg = yaml.safe_load((ROOT / "config/cube.yaml").read_text())
    if data.get("source_frame") != f"apriltag_{cfg['reference_tag_id']}":
        parser.error("Source frame must match the current reference tag")
    rows = data["points"]
    if len({p['name'] for p in rows}) != len(rows) or any(p['use'] not in ('fit','check') for p in rows):
        parser.error("Point names must be unique; use must be fit or check")
    fit = [p for p in rows if p['use'] == 'fit']
    check = [p for p in rows if p['use'] == 'check']
    if len(fit) < 3 or len(check) < 2:
        parser.error("Provide at least three fit points and two independent check points")
    def arrays(group):
        return [p['reference_xyz_m'] for p in group], [p['base_xyz_m'] for p in group]
    try:
        transform = fit_rigid(*arrays(fit))
        fit_errors = residuals(transform, *arrays(fit))
        check_errors = residuals(transform, *arrays(check))
    except (ValueError, np.linalg.LinAlgError) as exc:
        parser.error(str(exc))
    passed = bool(max(fit_errors.max(), check_errors.max()) * 1000 <= args.max_error_mm)
    run = create_run(args.output_root.resolve(), prefix="registration")
    write_json(run / "measurements.json", data)
    report = {"schema_version": 1, "source_frame": data['source_frame'], "target_frame": "robot_base",
        "robot_base_T_reference_tag": transform.tolist(),
        "convention": "p_base = R @ p_reference + t; nested rows; meters; rigid transform without scale fit",
        "fit_errors_mm": dict(zip([p['name'] for p in fit], (fit_errors*1000).tolist())),
        "independent_check_errors_mm": dict(zip([p['name'] for p in check], (check_errors*1000).tolist())),
        "residual_limit_mm": args.max_error_mm, "residual_check_passed": passed,
        "validated": False, "motion_enabled": False,
        "note": "Candidate only. Check measurement provenance, axes and units; residuals cannot detect a shared coordinate-frame mistake. Active workspace is unchanged."}
    write_json(run / "registration.json", report)
    print(f"Residual check: {'PASS' if passed else 'FAIL'}; candidate saved to {run}")
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
