#!/usr/bin/env python3
"""Record configured Cartesian EE state without commanding motion."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import importlib.metadata
import json
import math
from pathlib import Path
import platform
import sys
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from astra_on_franka.hardware.arm_observer import ArmObserver, ObservationValidationError


def create_run(root: Path, prefix: str = "arm") -> Path:
    root.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    run = root / f"{prefix}_{stamp}_{uuid.uuid4().hex[:8]}"
    run.mkdir(exist_ok=False)
    return run


def write_json(path: Path, value: dict) -> None:
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write("\n")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-host", default="172.16.0.2")
    parser.add_argument("--samples", type=int, default=1, help="0: run until Ctrl-C; default: 1")
    parser.add_argument("--hz", type=float, default=15.0, help="Requested sampling rate; not a realtime guarantee")
    parser.add_argument("--output-root", type=Path, default=ROOT / "data" / "observations")
    args = parser.parse_args(argv)
    if args.samples < 0 or not math.isfinite(args.hz) or args.hz <= 0:
        parser.error("samples must be nonnegative and hz must be finite and positive")
    run = create_run(args.output_root.expanduser().resolve())
    versions = {}
    for name in ("franky-control", "numpy"):
        try:
            versions[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            versions[name] = None
    write_json(run / "metadata.json", {
        "schema_version": 1, "robot_host": args.robot_host,
        "requested_hz": args.hz, "requested_samples": args.samples,
        "python": sys.version, "platform": platform.platform(), "packages": versions,
        "frame_definitions": {"O": "robot base", "F": "flange", "EE": "configured end effector"},
        "transform_convention": "A_T_B maps coordinates in B to A; nested rows, translation in meters",
        "tcp_calibrated": False,
        "initialization": "Franky Robot(host) applies library default collision thresholds",
        "timestamp_note": "Host receipt times and robot uptime are different clocks; not synchronized",
    })
    print(f"Saving to: {run}", flush=True)
    print("No motion commands. Franky initialization applies its default collision thresholds.", flush=True)
    count = 0
    status, error, code = "completed", None, 0
    try:
        with (run / "observations.jsonl").open("x", encoding="utf-8", buffering=1) as stream:
            with ArmObserver(args.robot_host) as observer:
                while args.samples == 0 or count < args.samples:
                    started = time.monotonic()
                    sample = observer.read()
                    sample["sample_index"] = count
                    stream.write(json.dumps(sample, allow_nan=False) + "\n")
                    xyz = sample["ee_position_base_m"]
                    print(f"[{count}] configured EE in base [m]: "
                          f"x={xyz[0]:.6f} y={xyz[1]:.6f} z={xyz[2]:.6f} "
                          f"errors={sample['has_errors']} read_ms={sample['read_duration_ms']:.2f}", flush=True)
                    if count == 0:
                        print("Joint positions [rad]:", sample["joint_positions_rad"])
                        print("Rotation EE -> base:")
                        for row in sample["ee_rotation_base"]:
                            print(" ", row)
                        print("F_T_EE (configured EE -> flange):")
                        for row in sample["F_T_EE"]:
                            print(" ", row)
                    count += 1
                    if args.samples == 0 or count < args.samples:
                        time.sleep(max(0, 1 / args.hz - (time.monotonic() - started)))
    except KeyboardInterrupt:
        status, code = "interrupted", 130
    except Exception as exc:
        status, error, code = "failed", f"{type(exc).__name__}: {exc}", 1
        if isinstance(exc, ObservationValidationError):
            write_json(run / "rejected_sample.json", exc.diagnostics)
            print(f"Rejected transform diagnostics: {run / 'rejected_sample.json'}", file=sys.stderr)
        print(error, file=sys.stderr)
    finally:
        write_json(run / "summary.json", {
            "status": status, "valid_samples": count, "error": error,
            "ended_unix_ns": time.time_ns(),
        })
    print(f"{status}: {count} valid samples. Logs: {run}")
    return code


if __name__ == "__main__":
    raise SystemExit(main())
