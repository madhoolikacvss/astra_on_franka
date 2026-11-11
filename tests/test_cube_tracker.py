import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from astra_on_franka.perception.cube_tracker import CubeTracker, normalize_xy

CONFIG = {"reference_tag_id": 0, "cube_tag_id": 1, "tag_size_m": 0.05,
          "family": "tag36h11", "board_bounds_m": {"x_min": -0.5, "x_max": -0.07,
          "y_min": -0.71, "y_max": -0.12}}


def make_frame():
    return {"rgb": np.zeros((48, 64, 3), np.uint8), "serial": "145422070656",
            "host_received_monotonic_ns": time.monotonic_ns(), "host_received_unix_ns": time.time_ns(),
            "streams": {"color": {"frame_number": 1, "timestamp_ms": 100, "timestamp_domain": "hardware_clock"}},
            "calibration": {"color_intrinsics": {"width": 64, "height": 48, "fx": 50, "fy": 50,
                "ppx": 32, "ppy": 24, "coeffs": [0] * 5, "distortion_model": "distortion.inverse_brown_conrady"}}}


def tag(tag_id, xyz, R=None):
    return SimpleNamespace(tag_id=tag_id, pose_R=np.eye(3) if R is None else R,
        pose_t=np.asarray(xyz).reshape(3, 1), hamming=0, decision_margin=80., pose_err=0.001,
        corners=np.array([[10, 10], [20, 10], [20, 20], [10, 20]]))


class Detector:
    def __init__(self, tags):
        self.tags = tags
        self.calls = []

    def detect(self, gray, **kwargs):
        self.calls.append(kwargs)
        return self.tags


class CubeTests(unittest.TestCase):
    def test_reference_transform_and_radian_yaw(self):
        R0 = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
        yaw = 0.4
        R1 = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
        relative = np.array([-0.285, -0.415, 0.05])
        detector = Detector([tag(0, [0, 0, 1], R0), tag(1, R0 @ relative + [0, 0, 1], R0 @ R1)])
        result, image = CubeTracker(CONFIG, detector=detector).process(make_frame())
        self.assertTrue(result["detected"])
        np.testing.assert_allclose(result["position_reference_m"], relative)
        np.testing.assert_allclose(result["position_board_normalized"], [0.5, 0.5])
        self.assertAlmostEqual(result["yaw_rad"], yaw)
        self.assertEqual(detector.calls[0]["tag_size"], 0.05)
        self.assertEqual(detector.calls[0]["camera_params"], (50., 50., 32., 24.))
        self.assertEqual(image.shape, (48, 64, 3))

    def test_normalization_is_unclipped(self):
        self.assertEqual(normalize_xy([-0.5, -0.71], CONFIG["board_bounds_m"]), [0, 0])
        np.testing.assert_allclose(normalize_xy([-0.07, -0.12], CONFIG["board_bounds_m"]), [1, 1])
        self.assertGreater(normalize_xy([0, 0], CONFIG["board_bounds_m"])[0], 1)

    def test_missing_duplicate_and_rejected_tags_have_no_position(self):
        bad = tag(1, [0, 0, 1]); bad.hamming = 1
        for tags in ([], [tag(1, [0, 0, 1])], [tag(0, [0, 0, 1]), bad],
                     [tag(0, [0, 0, 1]), tag(1, [0, 0, 1]), tag(1, [0, 0, 1])]):
            with self.subTest(tags=len(tags)):
                result, _ = CubeTracker(CONFIG, detector=Detector(tags)).process(make_frame())
                self.assertFalse(result["detected"])
                self.assertIsNone(result["position_board_normalized"])

    def test_stale_input_is_not_detected(self):
        frame = make_frame()
        frame["host_received_monotonic_ns"] -= 1_000_000_000
        detector = Detector([])
        result, _ = CubeTracker(CONFIG, detector=detector).process(frame)
        self.assertEqual(result["reason"], "stale_source_frame")
        self.assertEqual(detector.calls, [])

    def test_resolution_mismatch_rejected(self):
        frame = make_frame()
        frame["calibration"]["color_intrinsics"]["width"] = 1280
        with self.assertRaises(ValueError):
            CubeTracker(CONFIG, detector=Detector([])).process(frame)

    def test_unsupported_nonzero_distortion_is_not_silently_ignored(self):
        frame = make_frame()
        frame["calibration"]["color_intrinsics"]["coeffs"][0] = 0.1
        with self.assertRaisesRegex(ValueError, "rectification"):
            CubeTracker(CONFIG, detector=Detector([])).process(frame)


if __name__ == "__main__":
    unittest.main()
