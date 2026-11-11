from pathlib import Path
import sys
import time
import unittest
from unittest.mock import patch
import importlib.util
import tempfile
import json
import contextlib
import io

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from astra_on_franka.hardware.cameras import CameraRig, StreamSettings


def frame(number=1, received=None):
    return {"streams": {k: {"frame_number": number, "timestamp_ms": number * 33.,
                            "timestamp_domain": "hardware_clock"} for k in ("color", "depth")},
            "host_received_monotonic_ns": received if received is not None else time.monotonic_ns(),
            "rgb": np.zeros((2, 2, 3), np.uint8),
            "depth_raw": np.ones((2, 2), np.uint16),
            "depth_aligned": np.ones((2, 2), np.uint16)}


class FakeSource:
    def __init__(self, serial, settings, *, repeat=False, fail=False):
        self.serial, self.repeat, self.fail = serial, repeat, fail
        self.number = 0
        self.closed = False

    def start(self):
        if self.fail:
            raise RuntimeError("camera disconnected")

    def read(self):
        time.sleep(0.002)
        self.number += 1
        return frame(1 if self.repeat else self.number)

    def close(self):
        self.closed = True


class CameraTests(unittest.TestCase):
    def test_duplicate_serials_rejected(self):
        with self.assertRaises(ValueError):
            CameraRig({"a": "1", "b": "1"})

    def test_three_workers_new_frames_and_cleanup(self):
        sources = []
        def factory(serial, settings):
            source = FakeSource(serial, settings)
            sources.append(source)
            return source
        with CameraRig({"top_down": "1", "wrist": "2", "side": "3"}, source_factory=factory) as rig:
            first, second = rig.read(), rig.read()
            self.assertEqual(len(first["cameras"]), 3)
            for name in first["cameras"]:
                self.assertGreater(second["cameras"][name]["sequence"], first["cameras"][name]["sequence"])
            self.assertGreaterEqual(second["host_receipt_spread_ms"], 0)
        self.assertTrue(all(source.closed for source in sources))
        self.assertFalse(any(t.is_alive() for t in rig.threads))

    def test_stale_frames_rejected(self):
        rig = CameraRig({"a": "1"}, StreamSettings(stale_after_s=0.01))
        rig.latest["a"] = {**frame(received=time.monotonic_ns() - 100_000_000), "sequence": 1}
        with self.assertRaisesRegex(RuntimeError, "Stale"):
            rig.read()

    def test_repeated_device_frames_do_not_refresh_host_clock(self):
        sources = []
        def factory(serial, settings):
            source = FakeSource(serial, settings, repeat=True)
            sources.append(source)
            return source
        with CameraRig({"a": "1"}, StreamSettings(stale_after_s=0.03), source_factory=factory) as rig:
            first = rig.read()
            with self.assertRaisesRegex(RuntimeError, "Stale"):
                rig.read(timeout_s=1)
            self.assertEqual(rig.latest["a"]["host_received_monotonic_ns"],
                             first["cameras"]["a"]["host_received_monotonic_ns"])
        self.assertTrue(sources[0].closed)

    def test_failed_start_closes_every_source(self):
        sources = []
        def factory(serial, settings):
            source = FakeSource(serial, settings, fail=serial == "2")
            sources.append(source)
            return source
        with self.assertRaisesRegex(RuntimeError, "camera disconnected"):
            with CameraRig({"a": "1", "b": "2"}, source_factory=factory) as rig:
                rig.read()
        self.assertTrue(all(source.closed for source in sources))

    def test_backwards_counter_is_reported(self):
        class BackwardsSource(FakeSource):
            def read(self):
                time.sleep(0.01)
                self.number += 1
                return frame(5 if self.number == 1 else 4)
        with CameraRig({"a": "1"}, source_factory=BackwardsSource) as rig:
            rig.read()
            with self.assertRaisesRegex(RuntimeError, "backwards"):
                rig.read()

    def test_missing_camera_times_out(self):
        rig = CameraRig({"a": "1"})
        with self.assertRaises(TimeoutError):
            rig.read(timeout_s=0.01)

    def test_cleanup_error_reported(self):
        class BadCleanup(FakeSource):
            def close(self):
                raise RuntimeError("stop failed")
        with self.assertRaisesRegex(RuntimeError, "cleanup failed"):
            with CameraRig({"a": "1"}, source_factory=BadCleanup) as rig:
                rig.read()

    def test_recorder_writes_three_rgbd_files_and_metadata(self):
        root = Path(__file__).resolve().parents[1]
        sys.path.insert(0, str(root / "scripts"))
        spec = importlib.util.spec_from_file_location("camera_recorder", root / "scripts" / "record_cameras.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        class RecordingSource(FakeSource):
            def read(self):
                result = super().read()
                result.update({"calibration": {"depth_scale_m_per_unit": 0.001},
                               "aligned_depth_intrinsics": {"width": 2, "height": 2},
                               "host_received_unix_ns": time.time_ns(), "depth_valid_fraction": 1.0})
                return result
        with tempfile.TemporaryDirectory() as directory:
            with patch.object(module.importlib.metadata, "version", return_value="test"), \
                 patch.object(module, "CameraRig", side_effect=lambda cameras, settings:
                              CameraRig(cameras, settings, source_factory=RecordingSource)), \
                 contextlib.redirect_stdout(io.StringIO()):
                code = module.main(["--output-root", directory])
            self.assertEqual(code, 0)
            run = next(Path(directory).iterdir())
            self.assertTrue(run.name.startswith("cameras_"))
            row = json.loads((run / "observations.jsonl").read_text())
            self.assertEqual(set(row["cameras"]), {"top_down", "wrist", "side"})
            for camera in row["cameras"].values():
                with np.load(run / camera["file"], allow_pickle=False) as arrays:
                    self.assertEqual(arrays["rgb"].shape, (2, 2, 3))
                    self.assertEqual(arrays["depth_raw"].dtype, np.uint16)
                    self.assertEqual(arrays["depth_aligned"].dtype, np.uint16)
                self.assertEqual(camera["calibration"]["depth_scale_m_per_unit"], 0.001)
            self.assertEqual(json.loads((run / "summary.json").read_text())["valid_samples"], 1)


if __name__ == "__main__":
    unittest.main()
