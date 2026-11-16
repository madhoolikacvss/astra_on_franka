#!/usr/bin/env python3
"""Standalone configured cube-tag observer. No arm/gripper or goal integration."""
import argparse
import json
import math
from pathlib import Path
import sys
import time

import cv2
import yaml
import numpy as np

from record_observations import ROOT, create_run, write_json
from astra_on_franka.hardware.cameras import CameraRig, StreamSettings
from astra_on_franka.perception.cube_tracker import CubeTracker


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config/cube.yaml")
    parser.add_argument("--camera-config", type=Path, default=ROOT / "config/cameras.yaml")
    parser.add_argument("--samples", type=int, default=1, help="0: until Ctrl-C or q")
    parser.add_argument("--hz", type=float, default=10)
    parser.add_argument("--preview", action="store_true")
    parser.add_argument("--check-center", action="store_true", help="Place cube at independently measured board center; record center-check statistics")
    parser.add_argument("--center-tolerance", type=float, default=0.03, help="Per-axis normalized tolerance; default 3 percent of board span")
    parser.add_argument("--output-root", type=Path, default=ROOT / "data/cube")
    args = parser.parse_args(argv)
    if not math.isfinite(args.center_tolerance) or not 0 < args.center_tolerance < 0.5:
        parser.error("center-tolerance must be finite and between 0 and 0.5")
    if args.check_center and args.samples < 30:
        parser.error("--check-center requires at least 30 finite samples (suggest --samples 100)")
    if args.samples < 0 or not math.isfinite(args.hz) or args.hz <= 0:
        parser.error("samples must be nonnegative and hz finite and positive")
    config = yaml.safe_load(args.config.read_text())
    if args.check_center and not config.get("board_mapping"):
        parser.error("Center verification requires a four-corner board mapping")
    camera_config = yaml.safe_load(args.camera_config.read_text())
    view = config["camera"]
    serial = camera_config["cameras"][view]["serial"]
    settings = StreamSettings(**camera_config["stream"])
    run = create_run(args.output_root.expanduser().resolve(), prefix="cube")
    write_json(run / "metadata.json", {"schema_version": 1, "cube_config": config,
        "camera_config": camera_config, "source_view": view, "python": sys.version,
        "calibration_source": "active RealSense color stream intrinsics, logged per sample",
        "tracked_point": f"tag {config['cube_tag_id']} center, not a calibrated physical cube center",
        "normalization": "Unclipped four-corner mapping if configured; otherwise legacy axis bounds",
        "angle_unit": "radians"})
    print(f"Recording cube positions to {run}", flush=True)
    count, detected, code, status, error, rig = 0, 0, 0, "completed", None, None
    center_positions = []
    if args.check_center:
        print("Place cube at the independently measured center BEFORE recording; do not position it using the displayed normalized values.", flush=True)
        input("Remove your hand, then press Enter here to start the center check: ")
    try:
        tracker = CubeTracker(config)
        with (run / "observations.jsonl").open("x", encoding="utf-8", buffering=1) as log:
            with CameraRig({view: serial}, settings) as rig:
                while args.samples == 0 or count < args.samples:
                    started = time.monotonic()
                    frame = rig.read()["cameras"][view]
                    result, preview = tracker.process(frame, max_age_s=settings.stale_after_s)
                    result["sample_index"] = count
                    log.write(json.dumps(result, allow_nan=False) + "\n")
                    # Save first preview and first detected pose for offline inspection.
                    if count == 0 or (result["detected"] and detected == 0):
                        if not cv2.imwrite(str(run / f"preview_{count:06d}.png"), preview):
                            raise RuntimeError("Could not save annotated preview")
                    count += 1
                    detected += int(result["detected"])
                    if args.check_center and result["detected"]:
                        center_positions.append(result["position_board_normalized"])
                    print(f"[{count-1}] detected={result['detected']} xyz_m={result['position_reference_m']} "
                          f"xy_norm={result['position_board_normalized']} yaw_rad={result['yaw_rad']} "
                          f"reason={result['reason']}", flush=True)
                    if args.preview:
                        cv2.imshow("Cube tag relative to reference tag", preview)
                        if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
                            status = "stopped_by_user"
                            break
                    if args.samples == 0 or count < args.samples:
                        time.sleep(max(0, 1 / args.hz - (time.monotonic() - started)))
    except KeyboardInterrupt:
        status, code = "interrupted", 130
    except Exception as exc:
        status, error, code = "failed", f"{type(exc).__name__}: {exc}", 1
        print(error, file=sys.stderr)
    finally:
        if args.check_center:
            points = np.asarray(center_positions, dtype=float).reshape(-1, 2)
            deviation = np.abs(points - 0.5)
            within = np.all(deviation <= args.center_tolerance, axis=1)
            passed = (status == "completed" and count == args.samples and count >= 30
                      and detected / count >= 0.95 and len(points) > 0 and float(within.mean()) >= 0.95)
            report = {"passed": bool(passed), "calibration_id": config["board_mapping"]["calibration_id"],
                "target_normalized": [0.5, 0.5], "tolerance_per_axis": args.center_tolerance,
                "samples": count, "detected_samples": detected,
                "fraction_detected_within_tolerance": float(within.mean()) if len(points) else None,
                "median_normalized": np.median(points, axis=0).tolist() if len(points) else None,
                "std_normalized": np.std(points, axis=0).tolist() if len(points) else None,
                "max_absolute_error_per_axis": deviation.max(axis=0).tolist() if len(points) else None,
                "note": "Diagnostic center check only; assumes independent physical placement. Does not validate robot registration or motion safety."}
            write_json(run / "center_check.json", report)
            print(f"Center check: {'PASS' if passed else 'NOT PASSED'}; median={report['median_normalized']}")
        if rig is not None:
            write_json(run / "camera_diagnostics.json", rig.diagnostics())
        if args.preview:
            cv2.destroyAllWindows()
        write_json(run / "summary.json", {"status": status, "samples": count,
            "detected_samples": detected, "missing_or_rejected_samples": count - detected, "error": error})
    print(f"{status}: {detected}/{count} detected; {run}")
    return code


if __name__ == "__main__":
    raise SystemExit(main())
