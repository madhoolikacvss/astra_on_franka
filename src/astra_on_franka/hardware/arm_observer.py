"""Non-motion observations of Franky's configured EE, not a calibrated grasp TCP."""
from __future__ import annotations

import time
from typing import Any

import numpy as np


def finite_array(value: Any, shape: tuple[int, ...], name: str) -> np.ndarray:
    array = np.asarray(value, dtype=np.float64)
    if array.shape != shape or not np.isfinite(array).all():
        raise ValueError(f"{name}: expected finite array {shape}, got {array.shape}")
    return array.copy()


def transform(value: Any, name: str) -> np.ndarray:
    # Franky 2.0 Affine.matrix is a 4x4 property. Never guess flat-array order.
    matrix = finite_array(value.matrix, (4, 4), name)
    if not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-6, rtol=0):
        raise ValueError(f"{name}: invalid homogeneous bottom row")
    rotation = matrix[:3, :3]
    if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5, rtol=0)
            or not np.isclose(np.linalg.det(rotation), 1, atol=1e-5, rtol=0)):
        raise ValueError(f"{name}: rotation is not a proper orthonormal matrix")
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

    def read(self) -> dict:
        if self._robot is None:
            raise RuntimeError("Observer is closed")
        start = time.monotonic_ns()
        state = self._robot.state  # Exactly one robot snapshot per observation.
        end = time.monotonic_ns()
        wall = time.time_ns()
        joints = finite_array(state.q, (7,), "q")
        velocities = finite_array(state.dq, (7,), "dq")
        base_ee = transform(state.O_T_EE, "O_T_EE")
        flange_ee = transform(state.F_T_EE, "F_T_EE")
        robot_ms = int(state.time.to_msec())
        if robot_ms < 0:
            raise ValueError("Negative robot timestamp")
        if self._previous_robot_ms is not None and robot_ms <= self._previous_robot_ms:
            raise ValueError("Robot timestamp repeated or moved backwards; sample rejected")
        self._previous_robot_ms = robot_ms
        return {
            "schema_version": 1,
            "valid": True,
            "host_read_start_monotonic_ns": start,
            "host_read_end_monotonic_ns": end,
            "host_received_unix_ns": wall,
            "read_duration_ms": (end - start) / 1e6,
            "robot_time_ms": robot_ms,
            "joint_positions_rad": joints.tolist(),
            "joint_velocities_rad_s": velocities.tolist(),
            "O_T_EE": base_ee.tolist(),
            "F_T_EE": flange_ee.tolist(),
            "ee_position_base_m": base_ee[:3, 3].tolist(),
            "ee_rotation_base": base_ee[:3, :3].tolist(),
            "has_errors": bool(state.current_errors),
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
