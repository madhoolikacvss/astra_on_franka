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
    parser.add_argument("--output-root", type=Path, default=ROOT / "data/cube")
    args = parser.parse_args(argv)
    if args.samples < 0 or not math.isfinite(args.hz) or args.hz <= 0:
        parser.error("samples must be nonnegative and hz finite and positive")
    config = yaml.safe_load(args.config.read_text())
    camera_config = yaml.safe_load(args.camera_config.read_text())
    view = config["camera"]
    serial = camera_config["cameras"][view]["serial"]
    settings = StreamSettings(**camera_config["stream"])
    run = create_run(args.output_root.expanduser().resolve(), prefix="cube")
    write_json(run / "metadata.json", {"schema_version": 1, "cube_config": config,
        "camera_config": camera_config, "source_view": view, "python": sys.version,
        "calibration_source": "active RealSense color stream intrinsics, logged per sample",
        "tracked_point": f"tag {config['cube_tag_id']} center, not a calibrated physical cube center",
        "normalization": "Unclipped (coordinate-min)/(max-min); reference tag axes",
        "angle_unit": "radians"})
    print(f"Recording cube positions to {run}", flush=True)
    count, detected, code, status, error, rig = 0, 0, 0, "completed", None, None
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
