"""Independent RGB-D workers; host receipt times are not exposure synchronization."""
from __future__ import annotations

from dataclasses import dataclass
from collections import deque
import copy
import math
import threading
import time
from typing import Any

import numpy as np


@dataclass(frozen=True)
class StreamSettings:
    width: int = 640
    height: int = 480
    fps: int = 30
    stale_after_s: float = 0.5
    frame_timeout_ms: int = 1000
    startup_valid_pairs: int = 5
    max_rgb_depth_skew_ms: float = 70.0

    def __post_init__(self):
        if any(type(x) is not int or x <= 0 for x in
               (self.width, self.height, self.fps, self.frame_timeout_ms, self.startup_valid_pairs)):
            raise ValueError("Stream dimensions, fps and timeout must be positive integers")
        if not math.isfinite(self.stale_after_s) or self.stale_after_s <= 0:
            raise ValueError("stale_after_s must be finite and positive")
        if not math.isfinite(self.max_rgb_depth_skew_ms) or self.max_rgb_depth_skew_ms <= 0:
            raise ValueError("max_rgb_depth_skew_ms must be finite and positive")


def intrinsics(profile) -> dict:
    i = profile.as_video_stream_profile().get_intrinsics()
    return {"width": i.width, "height": i.height, "fx": i.fx, "fy": i.fy,
            "ppx": i.ppx, "ppy": i.ppy, "distortion_model": str(i.model),
            "coeffs": list(i.coeffs)}


def device_info(device, rs) -> dict:
    result = {}
    for key in ("serial_number", "asic_serial_number", "name", "firmware_version",
                "usb_type_descriptor", "physical_port"):
        field = getattr(rs.camera_info, key, None)
        result[key] = device.get_info(field) if field is not None and device.supports(field) else None
    return result


def inventory() -> list[dict]:
    import pyrealsense2 as rs
    # Enumeration only: no pipeline starts or hardware resets.
    return [device_info(device, rs) for device in rs.context().query_devices()]


def classify_frame_pair(previous, current):
    """Drop unusable zero counters/repeats; never silently rebase real resets."""
    if any(current[k]["frame_number"] == 0 for k in ("color", "depth")):
        return "zero_counter"
    for name in ("color", "depth"):
        b = current[name]
        if b["frame_number"] < 0 or not math.isfinite(b["timestamp_ms"]):
            raise ValueError(f"{name}: invalid frame metadata")
        if previous is not None:
            a = previous[name]
            if a["timestamp_domain"] != b["timestamp_domain"]:
                raise RuntimeError(f"{name}: timestamp domain changed {a['timestamp_domain']} -> {b['timestamp_domain']}")
            if b["frame_number"] < a["frame_number"]:
                raise RuntimeError(f"{name}: frame counter moved backwards {a['frame_number']} -> {b['frame_number']}")
            if b["timestamp_ms"] < a["timestamp_ms"]:
                raise RuntimeError(f"{name}: timestamp moved backwards {a['timestamp_ms']} -> {b['timestamp_ms']}")
    if previous is not None and any(
            current[k]["frame_number"] == previous[k]["frame_number"]
            or current[k]["timestamp_ms"] == previous[k]["timestamp_ms"] for k in ("color", "depth")):
        return "repeat"
    return "accept"


class RealSenseSource:
    def __init__(self, serial: str, settings: StreamSettings):
        self.serial, self.settings = serial, settings
        self.pipeline = None

    def start(self) -> None:
        import pyrealsense2 as rs
        self.rs = rs
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(self.serial)
        s = self.settings
        config.enable_stream(rs.stream.color, s.width, s.height, rs.format.rgb8, s.fps)
        config.enable_stream(rs.stream.depth, s.width, s.height, rs.format.z16, s.fps)
        profile = pipeline.start(config)
        self.pipeline = pipeline  # Own cleanup even if subsequent metadata fails.
        device = profile.get_device()
        scale = float(device.first_depth_sensor().get_depth_scale())
        if not math.isfinite(scale) or scale <= 0:
            raise ValueError("Camera returned invalid depth scale")
        color = profile.get_stream(rs.stream.color)
        depth = profile.get_stream(rs.stream.depth)
        extrinsics = depth.get_extrinsics_to(color)
        self.metadata = {
            **device_info(device, rs), "depth_scale_m_per_unit": scale,
            "color_intrinsics": intrinsics(color), "depth_intrinsics": intrinsics(depth),
            "depth_to_color_extrinsics": {
                "rotation_column_major": list(extrinsics.rotation),
                "translation_m": list(extrinsics.translation),
            },
            "color_format": str(color.format()), "depth_format": str(depth.format()),
            "color_fps": color.fps(), "depth_fps": depth.fps(),
            "image_rotation_applied": False,
        }
        self.align = rs.align(rs.stream.color)

    def read(self) -> dict:
        frames = self.pipeline.wait_for_frames(self.settings.frame_timeout_ms)
        received_ns, unix_ns = time.monotonic_ns(), time.time_ns()
        color, depth = frames.get_color_frame(), frames.get_depth_frame()
        if not color or not depth:
            raise RuntimeError("Frameset missing RGB or depth")
        aligned = self.align.process(frames).get_depth_frame()
        if not aligned:
            raise RuntimeError("Depth-to-color alignment returned no frame")
        streams = {}
        for name, frame in (("color", color), ("depth", depth)):
            timestamp = float(frame.get_timestamp())
            if not math.isfinite(timestamp):
                raise ValueError(f"Invalid {name} timestamp")
            streams[name] = {"frame_number": frame.get_frame_number(),
                             "timestamp_ms": timestamp,
                             "timestamp_domain": str(frame.get_frame_timestamp_domain())}
        rgb = np.asanyarray(color.get_data()).copy()
        raw = np.asanyarray(depth.get_data()).copy()
        aligned_depth = np.asanyarray(aligned.get_data()).copy()
        if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError("Unexpected RGB format")
        if raw.dtype != np.uint16 or raw.ndim != 2:
            raise ValueError("Unexpected raw depth format")
        if aligned_depth.dtype != np.uint16 or aligned_depth.shape != rgb.shape[:2]:
            raise ValueError("Unexpected aligned depth format")
        return {"rgb": rgb, "depth_raw": raw, "depth_aligned": aligned_depth,
                "host_received_monotonic_ns": received_ns, "host_received_unix_ns": unix_ns,
                "streams": streams, "calibration": self.metadata,
                "aligned_depth_intrinsics": intrinsics(aligned.profile),
                "depth_valid_fraction": float(np.count_nonzero(raw) / raw.size)}

    def close(self) -> None:
        if self.pipeline is not None:
            self.pipeline.stop()
            self.pipeline = None


class CameraRig:
    """One latest-frame slot per camera; no unbounded queues or silent reuse.

    read() requires a newly captured RGB and depth frame from every camera.
    A failed camera invalidates the rig. No reconnect/reset is attempted.
    """
    def __init__(self, cameras: dict[str, str], settings=StreamSettings(), *, source_factory=RealSenseSource):
        if not cameras or len(set(cameras.values())) != len(cameras):
            raise ValueError("Camera serials must be nonempty and unique")
        if any(not isinstance(serial, str) or not serial.strip() for serial in cameras.values()):
            raise ValueError("Camera serials must be nonempty strings")
        self.cameras, self.settings, self.factory = dict(cameras), settings, source_factory
        self.condition = threading.Condition()
        self.stop = threading.Event()
        self.latest, self.errors, self.delivered = {}, {}, {}
        self.cleanup_errors = {}
        self.health = {name: {"phase": "not_started", "accepted_pairs": 0,
                              "repeated_pairs": 0, "zero_counter_pairs": 0,
                              "startup_resets": 0, "startup_streak": 0,
                              "skew_rejected_pairs": 0} for name in cameras}
        self.transitions = {name: deque(maxlen=16) for name in cameras}
        self.threads = []

    def _worker(self, name, serial):
        source = None
        try:
            source = self.factory(serial, self.settings)
            with self.condition:
                self.health[name]["phase"] = "starting"
            source.start()
            previous = None
            sequence = 0
            startup_streak = 0
            while not self.stop.is_set():
                read_started = time.monotonic_ns()
                with self.condition:
                    self.health[name]["phase"] = "reading"
                    self.health[name]["read_started_monotonic_ns"] = read_started
                frame = source.read()
                current = frame["streams"]
                with self.condition:
                    self.health[name]["last_read_duration_ms"] = (time.monotonic_ns() - read_started) / 1e6
                    self.health[name]["last_returned_streams"] = current
                event = {"host_monotonic_ns": time.monotonic_ns(),
                         "previous_accepted_streams": copy.deepcopy(previous),
                         "current_streams": copy.deepcopy(current)}
                try:
                    decision = classify_frame_pair(previous, current)
                    event["decision"] = decision
                except RuntimeError as exc:
                    # The observed SDK counters can reset during startup.
                    # Never publish until a fresh consecutive run is established.
                    event.update({"decision": "startup_rebase" if sequence == 0 else "fail", "error": str(exc)})
                    with self.condition:
                        self.transitions[name].append(event)
                        if sequence == 0:
                            self.health[name]["startup_resets"] += 1
                            self.health[name]["startup_streak"] = 0
                    if sequence == 0:
                        previous, startup_streak = None, 0
                        continue
                    raise
                color, depth = current["color"], current["depth"]
                # Device timestamp subtraction is only valid within a common domain.
                same_domain = color["timestamp_domain"] == depth["timestamp_domain"]
                skew = abs(color["timestamp_ms"] - depth["timestamp_ms"]) if same_domain else None
                frame["rgb_depth_timestamp_skew_ms"] = skew
                if decision == "accept" and (skew is None or skew > self.settings.max_rgb_depth_skew_ms):
                    decision = "rgb_depth_skew"
                event.update({"decision": decision, "rgb_depth_timestamp_skew_ms": skew})
                with self.condition:
                    self.transitions[name].append(event)
                if decision != "accept":
                    # Keep the previous accepted pair and its receipt time.
                    with self.condition:
                        key = {"zero_counter": "zero_counter_pairs", "repeat": "repeated_pairs",
                               "rgb_depth_skew": "skew_rejected_pairs"}[decision]
                        self.health[name][key] += 1
                        if sequence == 0:
                            self.health[name]["startup_streak"] = 0
                    if sequence == 0:
                        previous, startup_streak = None, 0
                    self.stop.wait(0.001)
                    continue
                previous = current
                if sequence == 0:
                    startup_streak += 1
                    with self.condition:
                        self.health[name]["phase"] = "warming_up"
                        self.health[name]["startup_streak"] = startup_streak
                        event["decision"] = "warmup" if startup_streak < self.settings.startup_valid_pairs else "accept"
                    if startup_streak < self.settings.startup_valid_pairs:
                        continue
                sequence += 1
                frame.update({"sequence": sequence, "serial": serial})
                with self.condition:
                    self.latest[name] = frame
                    self.health[name]["accepted_pairs"] = sequence
                    self.condition.notify_all()
        except Exception as exc:
            with self.condition:
                self.errors[name] = f"{type(exc).__name__}: {exc}"
                self.health[name]["phase"] = "failed"
                self.health[name]["failure_monotonic_ns"] = time.monotonic_ns()
                self.condition.notify_all()
        finally:
            if source is not None:
                try:
                    source.close()
                except Exception as exc:
                    with self.condition:
                        self.errors[name] = f"Cleanup failed: {exc}"
                        self.cleanup_errors[name] = str(exc)
                        self.condition.notify_all()

    def __enter__(self):
        if self.threads or self.stop.is_set():
            raise RuntimeError("CameraRig cannot be restarted")
        for name, serial in self.cameras.items():
            worker = threading.Thread(target=self._worker, args=(name, serial), daemon=True)
            self.threads.append(worker)
            worker.start()
        return self

    def read(self, timeout_s: float = 10.0) -> dict:
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("timeout_s must be finite and positive")
        deadline = time.monotonic() + timeout_s
        with self.condition:
            while True:
                if self.stop.is_set():
                    raise RuntimeError("Camera rig is closed")
                if self.errors:
                    raise RuntimeError(f"Camera failure: {self.errors}")
                now = time.monotonic_ns()
                stale = [name for name, frame in self.latest.items()
                         if (now - frame["host_received_monotonic_ns"]) / 1e9 > self.settings.stale_after_s]
                # During initial startup, wait for a fresh replacement rather
                # than rejecting the first old frame while other devices start.
                # No stale frame is ever delivered, including during startup.
                if stale and self.delivered:
                    raise RuntimeError(f"Stale camera frames: {stale}")
                if not stale and all(name in self.latest and self.latest[name]["sequence"] > self.delivered.get(name, 0)
                       for name in self.cameras):
                    result = {name: dict(self.latest[name]) for name in self.cameras}
                    for name, frame in result.items():
                        frame["host_age_ms"] = (now - frame["host_received_monotonic_ns"]) / 1e6
                        frame["skipped_capture_pairs"] = max(0, frame["sequence"] - self.delivered.get(name, 0) - 1)
                        self.delivered[name] = frame["sequence"]
                    times = [x["host_received_monotonic_ns"] for x in result.values()]
                    return {"cameras": result, "host_snapshot_monotonic_ns": now,
                            "host_receipt_spread_ms": (max(times) - min(times)) / 1e6}
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("Timed out waiting for fresh RGB-D from every camera")
                self.condition.wait(min(remaining, 0.05))

    def diagnostics(self):
        with self.condition:
            now = time.monotonic_ns()
            return {"phase": "startup" if not self.delivered else "streaming",
                    "stale_after_s": self.settings.stale_after_s,
                    "errors": dict(self.errors),
                    "cameras": {name: {**self.health[name], "serial": serial,
                        "recent_transitions": copy.deepcopy(list(self.transitions[name])),
                        "host_age_ms": ((now - self.latest[name]["host_received_monotonic_ns"]) / 1e6
                                        if name in self.latest else None),
                        "last_delivered_sequence": self.delivered.get(name),
                    } for name, serial in self.cameras.items()}}

    def close(self):
        self.stop.set()
        with self.condition:
            self.condition.notify_all()
        for thread in self.threads:
            thread.join(timeout=self.settings.frame_timeout_ms / 1000 + 2)
        if any(thread.is_alive() for thread in self.threads):
            raise RuntimeError("Camera worker did not stop within its timeout")
        if self.cleanup_errors:
            raise RuntimeError(f"Camera cleanup failed: {self.cleanup_errors}")

    def __exit__(self, *_):
        self.close()
