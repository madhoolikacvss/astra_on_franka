"""Configured cube-tag center relative to the configured world/reference tag.

Input is an existing RGB frame, never a new camera connection. Output XY uses
the unchanged board bounds; no robot-base or physical cube-center calibration
is implied. Angles are radians, unlike the original tracker's degree output.
"""
from __future__ import annotations

import math
import time

import cv2
import numpy as np


def normalize_xy(xy, bounds):
    values = np.array([bounds[k] for k in ("x_min", "x_max", "y_min", "y_max")], float)
    if not np.isfinite(values).all() or values[1] <= values[0] or values[3] <= values[2]:
        raise ValueError("Board bounds must be finite with min < max")
    xy = np.asarray(xy, float)
    if xy.shape != (2,) or not np.isfinite(xy).all():
        raise ValueError("XY must be a finite pair")
    return [(float(xy[0]) - values[0]) / (values[1] - values[0]),
            (float(xy[1]) - values[2]) / (values[3] - values[2])]


class CubeTracker:
    def __init__(self, config, *, detector=None):
        self.config = dict(config)
        self.reference_id = int(config["reference_tag_id"])
        self.cube_id = int(config["cube_tag_id"])
        self.tag_size = float(config["tag_size_m"])
        self.bounds = config["board_bounds_m"]
        normalize_xy([0, 0], self.bounds)
        if self.reference_id == self.cube_id or not math.isfinite(self.tag_size) or self.tag_size <= 0:
            raise ValueError("Distinct reference/cube IDs and positive tag size are required")
        if detector is None:
            from pupil_apriltags import Detector
            detector = Detector(families=config["family"], nthreads=4,
                                quad_decimate=1.0, quad_sigma=0.0, refine_edges=True)
        self.detector = detector
        self._calibration_key = None
        self._maps = None

    def _rectify(self, rgb, intr):
        width, height = int(intr["width"]), int(intr["height"])
        if rgb.dtype != np.uint8 or rgb.shape != (height, width, 3):
            raise ValueError("RGB dimensions do not match active color intrinsics")
        params = np.array([intr[k] for k in ("fx", "fy", "ppx", "ppy")], float)
        coeffs = np.asarray(intr["coeffs"], float)
        if not np.isfinite(params).all() or not np.isfinite(coeffs).all() or np.any(params[:2] <= 0):
            raise ValueError("Invalid color calibration")
        model = str(intr["distortion_model"]).split(".")[-1].lower()
        key = (width, height, *params, model, *coeffs)
        if key != self._calibration_key:
            K = np.array([[params[0], 0, params[2]], [0, params[1], params[3]], [0, 0, 1]])
            self._maps = None
            if model == "none" or np.all(coeffs == 0):
                pass
            elif model == "brown_conrady":
                self._maps = cv2.initUndistortRectifyMap(K, coeffs, None, K, (width, height), cv2.CV_32FC1)
            elif model == "modified_brown_conrady":
                # Use SDK forward projection to map rectified pixels into the
                # source image; modified Brown is not OpenCV's Brown model.
                import pyrealsense2 as rs
                sdk = rs.intrinsics()
                sdk.width, sdk.height = width, height
                sdk.fx, sdk.fy, sdk.ppx, sdk.ppy = map(float, params)
                sdk.model = rs.distortion.modified_brown_conrady
                sdk.coeffs = coeffs.tolist()
                points = [rs.rs2_project_point_to_pixel(sdk, [(u - params[2]) / params[0],
                           (v - params[3]) / params[1], 1.0]) for v in range(height) for u in range(width)]
                mapping = np.asarray(points, np.float32).reshape(height, width, 2)
                self._maps = (mapping[:, :, 0].copy(), mapping[:, :, 1].copy())
            else:
                raise ValueError(f"Nonzero distortion model {model!r} requires a verified rectification implementation")
            self._calibration_key = key
        image = rgb if self._maps is None else cv2.remap(rgb, *self._maps, cv2.INTER_LINEAR)
        return image, tuple(float(p) for p in params)

    def process(self, frame, *, max_age_s=0.5):
        started = time.monotonic_ns()
        if not math.isfinite(max_age_s) or max_age_s <= 0:
            raise ValueError("max_age_s must be positive and finite")
        result = {"schema_version": 1, "detected": False, "reason": None,
                  "reference_frame": f"apriltag_{self.reference_id}",
                  "tracked_point": f"apriltag_{self.cube_id}_center",
                  "position_reference_m": None, "position_board_normalized": None,
                  "yaw_rad": None, "inside_board": None, "T_reference_cube_tag": None,
                  "camera_serial": frame["serial"], "source_color": dict(frame["streams"]["color"]),
                  "source_host_received_monotonic_ns": frame["host_received_monotonic_ns"],
                  "source_host_received_unix_ns": frame["host_received_unix_ns"],
                  "calibration": frame["calibration"]["color_intrinsics"],
                  "tags": []}
        image, camera_params = self._rectify(frame["rgb"], result["calibration"])
        preview = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        transforms = {}
        counts = {}
        if (started - frame["host_received_monotonic_ns"]) / 1e9 > max_age_s:
            result["reason"] = "stale_source_frame"
        else:
            tags = self.detector.detect(cv2.cvtColor(image, cv2.COLOR_RGB2GRAY),
                                       estimate_tag_pose=True, camera_params=camera_params,
                                       tag_size=self.tag_size)
            for tag in tags:
                tag_id = int(tag.tag_id)
                if tag_id not in (self.reference_id, self.cube_id):
                    continue
                counts[tag_id] = counts.get(tag_id, 0) + 1
                R, t = np.asarray(tag.pose_R, float), np.asarray(tag.pose_t, float).reshape(-1)
                margin, pose_error = float(tag.decision_margin), float(tag.pose_err)
                accepted = (R.shape == (3, 3) and t.shape == (3,) and np.isfinite(R).all()
                            and np.isfinite(t).all() and t[2] > 0 and math.isfinite(margin)
                            and math.isfinite(pose_error) and pose_error >= 0
                            and int(tag.hamming) <= self.config.get("max_hamming", 0)
                            and margin >= self.config.get("min_decision_margin", 0)
                            and np.allclose(R.T @ R, np.eye(3), atol=1e-4, rtol=0)
                            and np.isclose(np.linalg.det(R), 1, atol=1e-4, rtol=0))
                result["tags"].append({"id": tag_id, "accepted": bool(accepted),
                    "hamming": int(tag.hamming), "decision_margin": margin if math.isfinite(margin) else None,
                    "pose_error": pose_error if math.isfinite(pose_error) else None})
                corners = np.asarray(tag.corners)
                if corners.shape == (4, 2) and np.isfinite(corners).all():
                    cv2.polylines(preview, [corners.astype(np.int32)], True,
                                  (0, 255, 0) if accepted else (0, 0, 255), 2)
                    cv2.putText(preview, str(tag_id), tuple(corners[0].astype(int)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                if accepted:
                    T = np.eye(4)
                    T[:3, :3], T[:3, 3] = R, t
                    transforms[tag_id] = T
            if any(count > 1 for count in counts.values()):
                result["reason"] = "duplicate_required_tag_id"
            elif self.reference_id not in transforms or self.cube_id not in transforms:
                result["reason"] = "missing_or_rejected_required_tag"
            else:
                T = np.linalg.inv(transforms[self.reference_id]) @ transforms[self.cube_id]
                normalized = normalize_xy(T[:2, 3], self.bounds)
                result.update({"detected": True, "T_reference_cube_tag": T.tolist(),
                    "position_reference_m": T[:3, 3].tolist(), "position_board_normalized": normalized,
                    "inside_board": all(0 <= v <= 1 for v in normalized),
                    "yaw_rad": math.atan2(T[1, 0], T[0, 0]) if math.hypot(T[0, 0], T[1, 0]) > 1e-8 else None})
        finished = time.monotonic_ns()
        result["processing_ms"] = (finished - started) / 1e6
        result["source_age_at_result_ms"] = (finished - frame["host_received_monotonic_ns"]) / 1e6
        if result["source_age_at_result_ms"] > max_age_s * 1000:
            result.update({"detected": False, "reason": "stale_source_frame",
                           "position_reference_m": None, "position_board_normalized": None,
                           "yaw_rad": None, "inside_board": None, "T_reference_cube_tag": None})
        label = (f"xy_norm={result['position_board_normalized'][0]:.3f}, {result['position_board_normalized'][1]:.3f}"
                 if result["detected"] else f"No valid cube pose: {result['reason']}")
        cv2.putText(preview, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
        return result, preview
