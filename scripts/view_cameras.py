#!/usr/bin/env python3
"""Display live camera RGB without recording or connecting to arm/gripper."""
import argparse
from pathlib import Path
import sys

import cv2
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from astra_on_franka.hardware.cameras import CameraRig, StreamSettings


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config/cameras.yaml")
    parser.add_argument("--camera", action="append",
                        help="Configured view; repeat for several (default: all)")
    args = parser.parse_args(argv)
    config = yaml.safe_load(args.config.read_text())
    names = args.camera or list(config["cameras"])
    if any(name not in config["cameras"] for name in names):
        parser.error(f"Choose cameras from {list(config['cameras'])}")
    cameras = {name: config["cameras"][name]["serial"] for name in names}
    settings = StreamSettings(**config.get("stream", {}))
    print("Display only; no recordings. Press q or Escape in a camera window, or Ctrl+C to exit.")
    try:
        with CameraRig(cameras, settings) as rig:
            for name in cameras:
                cv2.namedWindow(name, cv2.WINDOW_NORMAL)
            while True:
                sample = rig.read()
                for name, frame in sample["cameras"].items():
                    cv2.imshow(name, frame["rgb"][:, :, ::-1])
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
                if any(cv2.getWindowProperty(name, cv2.WND_PROP_VISIBLE) < 1
                       for name in cameras):
                    break
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"{type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    finally:
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
