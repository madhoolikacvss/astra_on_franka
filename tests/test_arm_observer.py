import importlib.util
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import contextlib
import io
import json

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from astra_on_franka.hardware.arm_observer import ArmObserver

spec = importlib.util.spec_from_file_location("recorder", ROOT / "scripts" / "record_observations.py")
recorder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(recorder)


class FakeRobot:
    def __init__(self):
        self.reads = 0
        self.ms = 100
        matrix = np.eye(4)
        matrix[:3, 3] = [0.3, -0.2, 0.5]
        matrix[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
        self.snapshot = SimpleNamespace(
            q=np.zeros(7), dq=np.zeros(7), O_T_EE=SimpleNamespace(matrix=matrix),
            F_T_EE=SimpleNamespace(matrix=np.eye(4)),
            time=SimpleNamespace(to_msec=lambda: self.ms),
            current_errors=False, robot_mode="idle",
        )

    @property
    def state(self):
        self.reads += 1
        return self.snapshot


class ObserverTests(unittest.TestCase):
    def test_single_snapshot_and_transform_direction(self):
        robot = FakeRobot()
        with ArmObserver("unused", robot_factory=lambda _: robot) as observer:
            sample = observer.read()
        self.assertEqual(robot.reads, 1)
        self.assertEqual(sample["ee_position_base_m"], [0.3, -0.2, 0.5])
        self.assertEqual(sample["ee_rotation_base"][0], [0, -1, 0])
        self.assertGreaterEqual(sample["host_read_end_monotonic_ns"], sample["host_read_start_monotonic_ns"])
        with self.assertRaises(RuntimeError):
            observer.read()

    def test_bad_samples_rejected(self):
        for fault in ("nan", "shape", "reflection", "bottom_row", "flat"):
            with self.subTest(fault=fault):
                robot = FakeRobot()
                if fault == "nan":
                    robot.snapshot.q[0] = np.nan
                elif fault == "shape":
                    robot.snapshot.dq = np.zeros(6)
                elif fault == "reflection":
                    robot.snapshot.O_T_EE.matrix[:3, :3] = np.diag([-1, 1, 1])
                elif fault == "bottom_row":
                    robot.snapshot.O_T_EE.matrix[3, 0] = 1
                else:
                    robot.snapshot.O_T_EE.matrix = np.eye(4).ravel()
                with ArmObserver("unused", robot_factory=lambda _: robot) as observer:
                    with self.assertRaises(ValueError):
                        observer.read()

    def test_duplicate_and_backward_robot_time_rejected(self):
        for next_ms in (100, 99):
            robot = FakeRobot()
            with ArmObserver("unused", robot_factory=lambda _: robot) as observer:
                observer.read()
                robot.ms = next_ms
                with self.assertRaises(ValueError):
                    observer.read()

    def test_new_runs_preserve_existing_data(self):
        with tempfile.TemporaryDirectory() as directory:
            first = recorder.create_run(Path(directory))
            recorder.write_json(first / "metadata.json", {"keep": True})
            second = recorder.create_run(Path(directory))
            self.assertNotEqual(first, second)
            with self.assertRaises(FileExistsError):
                recorder.write_json(first / "metadata.json", {})
            self.assertIn('"keep": true', (first / "metadata.json").read_text())

    def test_cli_records_success_and_retains_failure_summary(self):
        for broken in (False, True):
            with self.subTest(broken=broken), tempfile.TemporaryDirectory() as directory:
                robot = FakeRobot()
                if broken:
                    robot.snapshot.q[0] = float("inf")
                with patch.object(recorder, "ArmObserver", side_effect=lambda host:
                                  ArmObserver(host, robot_factory=lambda _: robot)):
                    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                        code = recorder.main(["--output-root", directory])
                run = next(Path(directory).iterdir())
                summary = json.loads((run / "summary.json").read_text())
                self.assertEqual(code, 1 if broken else 0)
                self.assertEqual(summary["valid_samples"], 0 if broken else 1)
                rows = (run / "observations.jsonl").read_text().splitlines()
                self.assertEqual(len(rows), 0 if broken else 1)
                if not broken:
                    self.assertEqual(json.loads(rows[0])["robot_time_ms"], 100)

    def test_rejected_rotation_saved_without_normalizing(self):
        with tempfile.TemporaryDirectory() as directory:
            robot = FakeRobot()
            robot.snapshot.O_T_EE.matrix[0, 1] = -0.999
            original = robot.snapshot.O_T_EE.matrix.copy()
            with patch.object(recorder, "ArmObserver", side_effect=lambda host:
                              ArmObserver(host, robot_factory=lambda _: robot)):
                with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                    code = recorder.main(["--output-root", directory])
            run = next(Path(directory).iterdir())
            details = json.loads((run / "rejected_sample.json").read_text())
            self.assertEqual(code, 1)
            self.assertFalse(details["valid"])
            self.assertEqual(details["O_T_EE"]["matrix"], original.tolist())
            self.assertGreater(details["O_T_EE"]["rotation_orthogonality_max_abs_error"], 1e-5)
            self.assertEqual(details["F_T_EE"]["rotation_determinant"], 1.0)
            self.assertEqual(robot.reads, 1)
            self.assertEqual((run / "observations.jsonl").read_text(), "")

    def test_nonfinite_transform_diagnostics_are_json_safe(self):
        from astra_on_franka.hardware.arm_observer import ObservationValidationError
        robot = FakeRobot()
        robot.snapshot.O_T_EE.matrix[0, 0] = np.nan
        with ArmObserver("unused", robot_factory=lambda _: robot) as observer:
            with self.assertRaises(ObservationValidationError) as caught:
                observer.read()
        diagnostics = caught.exception.diagnostics
        json.dumps(diagnostics, allow_nan=False)
        self.assertFalse(diagnostics["O_T_EE"]["all_finite"])
        self.assertIsNone(diagnostics["O_T_EE"]["matrix"][0][0])


if __name__ == "__main__":
    unittest.main()
