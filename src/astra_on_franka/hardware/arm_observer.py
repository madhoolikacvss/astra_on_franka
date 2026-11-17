"""Non-motion observations of Franky's configured EE, not a calibrated grasp TCP."""
from __future__ import annotations

import time
import threading
from typing import Any

import numpy as np

ROTATION_ATOL = 1e-4
BOTTOM_ROW_ATOL = 1e-6
# Allow brief cached repeats at reader handoff, within the 100 ms motion watchdog.
NEW_STATE_TIMEOUT_S = 0.020
# The lab's configured F_T_EE uses rounded 0.7071 coefficients, giving
# ~1.93e-5 orthogonality/determinant error. Keep originals; do not normalize.


class ObservationValidationError(ValueError):
    """Rejected sample with JSON-safe evidence; never a valid observation."""

    def __init__(self, message: str, diagnostics: dict):
        super().__init__(message)
        self.diagnostics = diagnostics


def transform_diagnostics(value: Any) -> dict:
    matrix = np.asarray(value.matrix, dtype=np.float64)
    result = {
        "shape": list(matrix.shape),
        "all_finite": bool(np.isfinite(matrix).all()),
        "matrix": np.where(np.isfinite(matrix), matrix, None).tolist(),
        "nonfinite_encoding": "Nonfinite elements are null; matrix is not repaired",
        "rotation_atol": ROTATION_ATOL,
        "bottom_row_atol": BOTTOM_ROW_ATOL,
    }
    if matrix.shape == (4, 4) and np.isfinite(matrix).all():
        rotation = matrix[:3, :3]
        result.update({
            "rotation_determinant": float(np.linalg.det(rotation)),
            "rotation_orthogonality_max_abs_error": float(np.max(np.abs(rotation.T @ rotation - np.eye(3)))),
            "rotation_singular_values": np.linalg.svd(rotation, compute_uv=False).tolist(),
            "bottom_row_max_abs_error": float(np.max(np.abs(matrix[3] - [0, 0, 0, 1]))),
        })
    return result


def finite_array(value: Any, shape: tuple[int, ...], name: str) -> np.ndarray:
    array = np.asarray(value, dtype=np.float64)
    if array.shape != shape or not np.isfinite(array).all():
        raise ValueError(f"{name}: expected finite array {shape}, got {array.shape}")
    return array.copy()


def error_flags(errors: Any) -> dict[str, bool]:
    """Read bound boolean error properties, not Python object truthiness."""
    if isinstance(errors, (bool, np.bool_)):
        return {"unspecified_error": bool(errors)}
    flags = {}
    for name in dir(errors):
        if not name.startswith("_"):
            value = getattr(errors, name)
            if isinstance(value, (bool, np.bool_)):
                flags[name] = bool(value)
    if not flags:
        raise ValueError("current_errors exposes no boolean flags; error status is unknown")
    return flags


def transform(value: Any, name: str) -> np.ndarray:
    # Franky 2.0 Affine.matrix is a 4x4 property. Never guess flat-array order.
    matrix = finite_array(value.matrix, (4, 4), name)
    if not np.allclose(matrix[3], [0, 0, 0, 1], atol=BOTTOM_ROW_ATOL, rtol=0):
        raise ValueError(f"{name}: invalid homogeneous bottom row")
    rotation = matrix[:3, :3]
    if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=ROTATION_ATOL, rtol=0)
            or not np.isclose(np.linalg.det(rotation), 1, atol=ROTATION_ATOL, rtol=0)):
        error = float(np.max(np.abs(rotation.T @ rotation - np.eye(3))))
        determinant = float(np.linalg.det(rotation))
        raise ValueError(f"{name}: rotation is not a proper orthonormal matrix "
                         f"(max |R.T R - I|={error:.9g}, det(R)={determinant:.9g}, atol={ROTATION_ATOL:g})")
    return matrix


class ArmObserver:
    """Own one connection; no motion, recovery, or tool-setting commands.

    Franky's constructor applies its default collision thresholds. This is
    authorized for this lab, but means initialization is not read-only.
    """

    def __init__(self, host: str, *, robot_factory=None):
        if robot_factory is None:
            from franky import Robot
            robot_factory = Robot
        self._robot = robot_factory(host)
        self._previous_robot_ms: int | None = None
        self._read_lock = threading.Lock()

    def read(self) -> dict:
        start = time.monotonic_ns()
        if not self._read_lock.acquire(timeout=NEW_STATE_TIMEOUT_S):
            raise RuntimeError('Arm observation reader busy for 20 ms')
        try:
            return self._read_locked(start)
        finally:
            self._read_lock.release()

    def _read_locked(self, start: int) -> dict:
        if self._robot is None:
            raise RuntimeError("Observer is closed")
        deadline = start + int(NEW_STATE_TIMEOUT_S * 1e9)
        repeated = 0
        while True:
            state = self._robot.state
            end = time.monotonic_ns()
            robot_ms = int(state.time.to_msec())
            previous = self._previous_robot_ms
            if robot_ms < 0 or (previous is not None and robot_ms < previous):
                raise ValueError(f'Robot timestamp moved backwards or is negative; '
                                 f'previous_ms={previous}, current_ms={robot_ms}; sample rejected')
            if end > deadline:
                raise ValueError(f'Robot fresh-state deadline exceeded (20 ms); '
                                 f'previous_ms={previous}, current_ms={robot_ms}, '
                                 f'repeated_reads={repeated}; sample rejected')
            if previous is None or robot_ms > previous:
                break
            repeated += 1
            # A duplicate is never published, nor given a new receipt timestamp.
            # All pose/joint/error fields below come from the one advancing snapshot.
            time.sleep(.001)
        wall = time.time_ns()
        joints = finite_array(state.q, (7,), "q")
        velocities = finite_array(state.dq, (7,), "dq")
        try:
            base_ee = transform(state.O_T_EE, "O_T_EE")
            flange_ee = transform(state.F_T_EE, "F_T_EE")
        except ValueError as exc:
            raise ObservationValidationError(str(exc), {
                "valid": False,
                "reason": str(exc),
                "host_read_start_monotonic_ns": start,
                "host_read_end_monotonic_ns": end,
                "host_received_unix_ns": wall,
                "robot_time_ms": int(state.time.to_msec()),
                "joint_positions_rad": joints.tolist(),
                "joint_velocities_rad_s": velocities.tolist(),
                "O_T_EE": transform_diagnostics(state.O_T_EE),
                "F_T_EE": transform_diagnostics(state.F_T_EE),
            }) from exc
        flags = error_flags(state.current_errors)
        self._previous_robot_ms = robot_ms
        return {
            "schema_version": 1,
            "valid": True,
            "host_read_start_monotonic_ns": start,
            "host_read_end_monotonic_ns": end,
            "host_received_unix_ns": wall,
            "read_duration_ms": (end - start) / 1e6,
            "robot_time_ms": robot_ms,
            "repeated_state_reads_skipped": repeated,
            "joint_positions_rad": joints.tolist(),
            "joint_velocities_rad_s": velocities.tolist(),
            "O_T_EE": base_ee.tolist(),
            "F_T_EE": flange_ee.tolist(),
            "transform_validation": {
                "rotation_atol": ROTATION_ATOL,
                "bottom_row_atol": BOTTOM_ROW_ATOL,
                "matrices_normalized": False,
                **{name: {
                    "rotation_determinant": float(np.linalg.det(matrix[:3, :3])),
                    "rotation_orthogonality_max_abs_error": float(np.max(
                        np.abs(matrix[:3, :3].T @ matrix[:3, :3] - np.eye(3)))),
                } for name, matrix in (("O_T_EE", base_ee), ("F_T_EE", flange_ee))},
            },
            "ee_position_base_m": base_ee[:3, 3].tolist(),
            "ee_rotation_base": base_ee[:3, :3].tolist(),
            "has_errors": any(flags.values()),
            "error_flags": flags,
            "current_errors": str(state.current_errors),
            "robot_mode": str(state.robot_mode),
        }

    def close(self) -> None:
        # Release the owned Franky object; this observer never starts control.
        self._robot = None

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
