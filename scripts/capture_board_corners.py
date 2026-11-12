#!/usr/bin/env python3
"""Capture four cube-tag positions interactively; never connect to the arm."""
import argparse
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import yaml

from record_observations import ROOT, create_run, write_json
from astra_on_franka.hardware.cameras import CameraRig, StreamSettings
from astra_on_franka.perception.cube_tracker import CubeTracker


# Physical directions viewed from the robot base looking toward the board.
CORNERS = ("near_left", "near_right", "far_right", "far_left")


def capture_ready(result, frame, max_age_s):
    age = (time.monotonic_ns() - frame["host_received_monotonic_ns"]) / 1e9
    xyz = result.get("position_reference_m")
    return (result["detected"] and 0 <= age <= max_age_s and xyz is not None
            and np.asarray(xyz).shape == (3,) and np.isfinite(xyz).all())


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config/cube.yaml")
    parser.add_argument("--camera-config", type=Path, default=ROOT / "config/cameras.yaml")
    parser.add_argument("--output-root", type=Path, default=ROOT / "data/calibration")
    args = parser.parse_args(argv)
    config = yaml.safe_load(args.config.read_text())
    camera_config = yaml.safe_load(args.camera_config.read_text())
    view = config["camera"]
    settings = StreamSettings(**camera_config["stream"])
    serial = camera_config["cameras"][view]["serial"]
    run = create_run(args.output_root.expanduser().resolve(), prefix="board_corners")
    write_json(run / "metadata.json", {
        "schema_version": 1, "cube_config": config, "camera_config": camera_config,
        "corner_order": CORNERS,
        "directions": "Viewed from robot base toward board; not camera image left/right",
        "measurement": "Cube tag center relative to reference tag, meters",
        "placement": "Cube placed fully on board at each corner; measured tag centers define cube placement limits, not physical board edges",
        "normalization": "Legacy normalized values are diagnostic only; use raw positions for calibration",
        "validated": False, "motion_commands": False,
    })
    print(f"Saving to {run}", flush=True)
    print("Directions are as viewed from the robot base toward the board.")
    print("Place the cube fully on the board at each corner, keeping its orientation consistent.")
    print("Keep both tags visible and remove your hand before capture.")
    print("Click the camera window: ENTER captures, Q/ESC exits. Enter is handled in the camera window, not the terminal.")
    captured, rig = [], None
    status, error, code = "incomplete", None, 0
    window = "Board corners - ENTER capture | Q quit"
    message = "Place cube, remove hand, then press ENTER"
    try:
        tracker = CubeTracker(config)
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        with CameraRig({view: serial}, settings) as rig:
            print(f"Place cube at {CORNERS[0]}", flush=True)
            while len(captured) < len(CORNERS):
                frame = rig.read()["cameras"][view]
                result, preview = tracker.process(frame, max_age_s=settings.stale_after_s)
                corner = CORNERS[len(captured)]
                ready = capture_ready(result, frame, settings.stale_after_s)
                # Add a separate banner; legacy normalized values in the preview
                # are not used to accept/reject these raw calibration captures.
                banner = np.zeros((110, preview.shape[1], 3), np.uint8)
                xyz = result["position_reference_m"]
                lines = [f"{len(captured)+1}/4: {corner} (robot-base viewpoint)",
                         f"{'TRACKING' if ready else 'NO FRESH POSE'} | raw XYZ m: " +
                         (", ".join(f"{v:.4f}" for v in xyz) if xyz else "unavailable"),
                         message, "ENTER capture | Q/ESC quit | ignore legacy normalized XY"]
                for i, line in enumerate(lines):
                    cv2.putText(banner, line, (8, 22 + i * 25), cv2.FONT_HERSHEY_SIMPLEX,
                                0.45, (0, 255, 0) if ready else (0, 180, 255), 1)
                cv2.imshow(window, np.vstack((banner, preview)))
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27) or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                    status = "stopped_by_user"
                    break
                if key not in (10, 13):
                    continue
                # Recheck age at the key event, not only before rendering.
                if not capture_ready(result, frame, settings.stale_after_s):
                    message = "Capture refused: keep BOTH tags visible and retry ENTER"
                    print(message, flush=True)
                    continue
                entry = {"corner": corner, "captured_unix_ns": time.time_ns(),
                         "source_sequence": frame["sequence"], "observation": result,
                         "frame_file": f"{corner}.npz", "preview_file": f"{corner}.png"}
                with (run / entry["frame_file"]).open("xb") as output:
                    np.savez_compressed(output, **{k: frame[k] for k in ("rgb", "depth_raw", "depth_aligned")})
                if not cv2.imwrite(str(run / entry["preview_file"]), preview):
                    raise RuntimeError("Could not save annotated capture")
                entry["camera_metadata"] = {k: v for k, v in frame.items()
                                             if k not in ("rgb", "depth_raw", "depth_aligned")}
                write_json(run / f"{corner}.json", entry)
                captured.append(entry)
                print(f"Saved {corner}: {xyz} m", flush=True)
                message = "Saved. Place cube at next corner, then ENTER"
                if len(captured) < 4:
                    print(f"Place cube at {CORNERS[len(captured)]}", flush=True)
            if len(captured) == 4:
                status = "completed"
    except KeyboardInterrupt:
        status, code = "interrupted", 130
    except Exception as exc:
        status, error, code = "failed", f"{type(exc).__name__}: {exc}", 1
        print(error, file=sys.stderr)
    finally:
        cv2.destroyAllWindows()
        if rig is not None:
            write_json(run / "camera_diagnostics.json", rig.diagnostics())
        write_json(run / "corners.json", {"schema_version": 1, "validated": False,
            "reference_frame": f"apriltag_{config['reference_tag_id']}",
            "corners": {e["corner"]: e["observation"]["position_reference_m"] for e in captured}})
        write_json(run / "summary.json", {"status": status, "captured_corners": len(captured),
            "error": error, "active_calibration_modified": False})
    print(f"{status}: {len(captured)}/4 corners; {run}")
    return code


if __name__ == "__main__":
    raise SystemExit(main())
