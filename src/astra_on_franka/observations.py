"""Compose observations on host time without claiming exposure synchronization."""
import math
import time

import numpy as np


class CombinedObserver:
    """Uses caller-managed connections; never opens a second camera pipeline."""

    def __init__(self, arm, cameras, tracker, *, cube_view="top_down", max_age_s=0.5):
        if not math.isfinite(max_age_s) or max_age_s <= 0:
            raise ValueError("max_age_s must be positive and finite")
        self.arm, self.cameras, self.tracker = arm, cameras, tracker
        self.cube_view, self.max_age_s = cube_view, max_age_s

    def read(self):
        started = time.monotonic_ns()
        capture = self.cameras.read()
        arm = self.arm.read()
        frame = capture["cameras"][self.cube_view]
        cube, preview = self.tracker.process(frame, max_age_s=self.max_age_s)
        cube = dict(cube, source_view=self.cube_view, source_sequence=frame["sequence"])
        now = time.monotonic_ns()
        ages = {name: (now - f["host_received_monotonic_ns"]) / 1e6
                for name, f in capture["cameras"].items()}
        arm_age = (now - arm["host_read_end_monotonic_ns"]) / 1e6
        reasons = [f"stale_camera:{name}" for name, age in ages.items()
                   if age < 0 or age > self.max_age_s * 1000]
        if arm_age < 0 or arm_age > self.max_age_s * 1000:
            reasons.append("stale_arm")
        if not arm["valid"] or arm["has_errors"]:
            reasons.append("arm_invalid_or_has_errors")
        if not cube["detected"]:
            reasons.append("cube_not_detected")
        return {"schema_version": 1, "valid": not reasons, "invalid_reasons": reasons,
                "arm": arm, "cube": cube, **capture,
                "timing": {"assembly_started_monotonic_ns": started,
                    "assembly_finished_monotonic_ns": now,
                    "assembly_duration_ms": (now - started) / 1e6,
                    "camera_age_at_assembly_ms": ages, "arm_age_at_assembly_ms": arm_age,
                    "arm_receipt_minus_camera_receipt_ms": {
                        name: (arm["host_read_end_monotonic_ns"] - f["host_received_monotonic_ns"]) / 1e6
                        for name, f in capture["cameras"].items()}}}, preview


def save_frames(run, sample, index):
    """Return JSON metadata, preserving the source arrays and exact frame linkage."""
    result = dict(sample, sample_index=index)
    result["cameras"] = {}
    for name, frame in sample["cameras"].items():
        if not name or any(c not in "abcdefghijklmnopqrstuvwxyz0123456789_" for c in name):
            raise ValueError("Invalid camera filename prefix")
        relative = f"frames/{name}_{index:06d}.npz"
        with (run / relative).open("xb") as output:
            np.savez_compressed(output, **{k: frame[k] for k in ("rgb", "depth_raw", "depth_aligned")})
        result["cameras"][name] = {k: v for k, v in frame.items()
                                   if k not in ("rgb", "depth_raw", "depth_aligned")}
        result["cameras"][name]["file"] = relative
    result["cube"] = dict(sample["cube"], source_file=result["cameras"][sample["cube"]["source_view"]]["file"])
    return result
