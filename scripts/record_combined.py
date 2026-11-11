#!/usr/bin/env python3
"""Record arm, three RGB-D views and cube pose together; no motion commands."""
import argparse
import importlib.metadata
import json
import math
from pathlib import Path
import sys
import time

import yaml

from record_observations import ROOT, create_run, write_json
from astra_on_franka.hardware.arm_observer import ArmObserver, ObservationValidationError
from astra_on_franka.hardware.cameras import CameraRig, StreamSettings
from astra_on_franka.perception.cube_tracker import CubeTracker
from astra_on_franka.observations import CombinedObserver, save_frames


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-host", default="172.16.0.2")
    parser.add_argument("--camera-config", type=Path, default=ROOT / "config/cameras.yaml")
    parser.add_argument("--cube-config", type=Path, default=ROOT / "config/cube.yaml")
    parser.add_argument("--samples", type=int, default=1, help="0: until Ctrl-C or q")
    parser.add_argument("--hz", type=float, default=10)
    parser.add_argument("--preview", action="store_true")
    parser.add_argument("--output-root", type=Path, default=ROOT / "data/combined")
    args = parser.parse_args(argv)
    if args.samples < 0 or not math.isfinite(args.hz) or args.hz <= 0:
        parser.error("samples must be nonnegative and hz finite and positive")
    camera_config = yaml.safe_load(args.camera_config.read_text())
    cube_config = yaml.safe_load(args.cube_config.read_text())
    cameras = {name: item["serial"] for name, item in camera_config["cameras"].items()}
    if set(cameras) != {"top_down", "wrist", "side"} or cube_config["camera"] != "top_down":
        parser.error("Expected top_down, wrist and side cameras; cube source must be top_down")
    settings = StreamSettings(**camera_config["stream"])
    run = create_run(args.output_root.expanduser().resolve(), prefix="combined")
    (run / "frames").mkdir()
    versions = {}
    for package in ("franky-control", "pyrealsense2", "numpy", "PyYAML", "pupil-apriltags"):
        try:
            versions[package] = importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:
            versions[package] = None
    write_json(run / "metadata.json", {
        "schema_version": 1, "camera_config": camera_config, "cube_config": cube_config,
        "robot_host": args.robot_host, "packages": versions, "python": sys.version,
        "requested_hz": args.hz, "requested_samples": args.samples,
        "freshness_limit_s": settings.stale_after_s,
        "frames": {"arm": "robot base to configured EE; grasp TCP not calibrated",
                   "cube": "cube tag center in reference tag axes; no robot-base registration yet"},
        "units": "Joint angles/yaw radians; translations meters; board XY normalized and unclipped",
        "clock_note": "Host receipt ages/offsets are diagnostic, not exposure synchronization. Robot uptime and camera timestamps retain separate clocks.",
        "arrays": "NPZ: rgb uint8 RGB, depth_raw and depth_aligned uint16; meters = depth * depth_scale_m_per_unit; zero invalid",
        "valid_note": "Fresh observations, arm without errors and cube detected; not a motion safety authorization or board-bound check",
        "initialization": "Franky Robot applies default collision thresholds; no motion commands",
    })
    print(f"Recording combined observations to {run}", flush=True)
    print("No motion commands. Franky initialization applies default collision thresholds.", flush=True)
    count = valid = detected = 0
    status, error, code = "completed", None, 0
    arm = rig = None
    cleanup_errors = []
    try:
        tracker = CubeTracker(cube_config)
        arm = ArmObserver(args.robot_host)
        rig = CameraRig(cameras, settings)
        rig.__enter__()
        observer = CombinedObserver(arm, rig, tracker, max_age_s=settings.stale_after_s)
        with (run / "observations.jsonl").open("x", encoding="utf-8", buffering=1) as log:
            while args.samples == 0 or count < args.samples:
                started = time.monotonic()
                sample, preview = observer.read()
                row = save_frames(run, sample, count)
                row["host_persisted_unix_ns"] = time.time_ns()
                row["timing"]["assembly_to_persist_ms"] = (time.monotonic_ns() - row["timing"]["assembly_finished_monotonic_ns"]) / 1e6
                log.write(json.dumps(row, allow_nan=False) + "\n")
                valid += int(row["valid"])
                detected += int(row["cube"]["detected"])
                print(f"[{count}] valid={row['valid']} cube={row['cube']['position_board_normalized']} "
                      f"receipt_spread={row['host_receipt_spread_ms']:.1f}ms reasons={row['invalid_reasons']}", flush=True)
                count += 1
                if args.preview:
                    import cv2
                    cv2.imshow("Combined: top-down cube", preview)
                    for name in ("wrist", "side"):
                        cv2.imshow(f"Combined: {name}", sample["cameras"][name]["rgb"][:, :, ::-1])
                    if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
                        status = "stopped_by_user"
                        break
                if args.samples == 0 or count < args.samples:
                    time.sleep(max(0, 1 / args.hz - (time.monotonic() - started)))
    except KeyboardInterrupt:
        status, code = "interrupted", 130
    except Exception as exc:
        status, error, code = "failed", f"{type(exc).__name__}: {exc}", 1
        if isinstance(exc, ObservationValidationError):
            write_json(run / "rejected_sample.json", exc.diagnostics)
        print(error, file=sys.stderr)
    finally:
        for name, connection in (("cameras", rig), ("arm", arm)):
            if connection is not None:
                try:
                    connection.close()
                except Exception as exc:
                    cleanup_errors.append(f"{name}: {type(exc).__name__}: {exc}")
        if cleanup_errors:
            status, code = "failed", 1
        if rig is not None:
            write_json(run / "camera_diagnostics.json", rig.diagnostics())
        if args.preview:
            import cv2
            cv2.destroyAllWindows()
        write_json(run / "summary.json", {"status": status, "samples": count,
            "valid_samples": valid, "detected_samples": detected,
            "invalid_samples": count - valid, "error": error, "cleanup_errors": cleanup_errors})
    print(f"{status}: {valid}/{count} valid combined samples; {run}")
    return code


if __name__ == "__main__":
    raise SystemExit(main())
