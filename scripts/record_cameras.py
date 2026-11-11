#!/usr/bin/env python3
"""Stream and optionally preview three cameras, without connecting to the arm."""
from __future__ import annotations

import argparse
import importlib.metadata
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
import yaml

from record_observations import ROOT, create_run, write_json
from astra_on_franka.hardware.cameras import CameraRig, StreamSettings, inventory


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config" / "cameras.yaml")
    parser.add_argument("--list", action="store_true", help="Enumerate cameras without starting streams or resetting devices")
    parser.add_argument("--samples", type=int, default=1, help="0: until Ctrl-C")
    parser.add_argument("--hz", type=float, default=15, help="Recording rate, independent of capture fps")
    parser.add_argument("--preview", action="store_true", help="Requires a local graphical desktop; q exits")
    parser.add_argument("--output-root", type=Path, default=ROOT / "data" / "cameras")
    args = parser.parse_args(argv)
    if args.list:
        print(json.dumps(inventory(), indent=2))
        return 0
    if args.samples < 0 or not math.isfinite(args.hz) or args.hz <= 0:
        parser.error("samples must be nonnegative and hz finite and positive")
    config = yaml.safe_load(args.config.read_text())
    cameras = {name: item["serial"] for name, item in config["cameras"].items()}
    settings = StreamSettings(**config.get("stream", {}))
    # Validate names because they become filename prefixes.
    if any(not name or any(c not in "abcdefghijklmnopqrstuvwxyz0123456789_" for c in name) for name in cameras):
        parser.error("Camera names must contain only lowercase letters, digits, underscores")
    cv2 = None
    if args.preview:
        import cv2
    run = create_run(args.output_root.expanduser().resolve(), prefix="cameras")
    (run / "frames").mkdir()
    write_json(run / "metadata.json", {
        "schema_version": 1, "config": config, "python": sys.version,
        "pyrealsense2": importlib.metadata.version("pyrealsense2"),
        "requested_record_hz": args.hz, "requested_samples": args.samples,
        "clock_note": "Per-stream device timestamps retain their domains. Host receipt spread is not exposure synchronization.",
        "arrays": "RGB uint8; depth_raw and depth_aligned uint16. meters = value * depth_scale_m_per_unit; zero is invalid.",
        "alignment": "depth_aligned is reprojected onto the color image; native depth is also retained",
    })
    print(f"Recording cameras to {run}", flush=True)
    count, code, error, status = 0, 0, None, "completed"
    try:
        with (run / "observations.jsonl").open("x", encoding="utf-8", buffering=1) as log:
            with CameraRig(cameras, settings) as rig:
                while args.samples == 0 or count < args.samples:
                    started = time.monotonic()
                    sample = rig.read()
                    previews = []
                    for name, frame in sample["cameras"].items():
                        rgb = frame.pop("rgb")
                        raw = frame.pop("depth_raw")
                        aligned = frame.pop("depth_aligned")
                        relative = f"frames/{name}_{count:06d}.npz"
                        with (run / relative).open("xb") as file:
                            np.savez_compressed(file, rgb=rgb, depth_raw=raw, depth_aligned=aligned)
                        frame["file"] = relative
                        if cv2 is not None:
                            meters = aligned.astype(np.float32) * frame["calibration"]["depth_scale_m_per_unit"]
                            colors = cv2.applyColorMap((np.clip(meters / 2, 0, 1) * 255).astype(np.uint8), cv2.COLORMAP_JET)
                            colors[aligned == 0] = 0
                            pair = np.hstack([rgb[:, :, ::-1], colors])
                            cv2.putText(pair, name, (12, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                            previews.append(pair)
                    sample["sample_index"] = count
                    sample["valid"] = True
                    log.write(json.dumps(sample, allow_nan=False) + "\n")
                    print(f"[{count}] receipt spread {sample['host_receipt_spread_ms']:.1f} ms; " +
                          "; ".join(f"{name}: age={f['host_age_ms']:.1f}ms depth={f['depth_valid_fraction']:.1%}"
                                    for name, f in sample["cameras"].items()), flush=True)
                    count += 1
                    if cv2 is not None:
                        cv2.imshow("RGB | aligned depth (display range 0-2 m)", np.vstack(previews))
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
        if cv2 is not None:
            cv2.destroyAllWindows()
        write_json(run / "summary.json", {"status": status, "valid_samples": count,
                                         "error": error, "ended_unix_ns": time.time_ns()})
    print(f"{status}: {count} recorded sets; {run}")
    return code


if __name__ == "__main__":
    raise SystemExit(main())
