"""Camera-side optics reads, calibration summaries, and live FOV snapshots."""

from __future__ import annotations

import math
import threading
from collections.abc import Iterable, Mapping

import numpy as np

from navpy.modules.vision.camera_mount_ports import (
    CameraCalibrationPort,
    CameraOpticsPort,
    CameraZoomPort,
    LiveOpticsPort,
    LiveZoomReadbackPort,
)
from navpy.modules.vision.camera_mount_types import CameraMountOptics
from navpy.modules.vision.vision_class_profile import (
    MIN_CONFIRM_PIXELS,
    MIN_DETECT_PIXELS,
)


class MountOpticsReader:
    """Serialize camera intrinsic reads against zoom writers."""

    def __init__(
        self,
        camera: CameraOpticsPort,
        lock: threading.RLock,
    ) -> None:
        self._camera = camera
        self._lock = lock

    def get_k(self) -> np.ndarray:
        with self._lock:
            return self._camera.get_k()

    def get_k_for_frame(self, frame_w: int, frame_h: int) -> np.ndarray:
        with self._lock:
            return self.get_k_for_frame_locked(frame_w, frame_h)

    def get_k_for_frame_locked(self, frame_w: int, frame_h: int) -> np.ndarray:
        matrix = self._camera.get_k()
        width = self.image_width
        height = self.image_height
        if (
            not width
            or not height
            or frame_w <= 0
            or frame_h <= 0
            or (int(width) == int(frame_w) and int(height) == int(frame_h))
        ):
            return matrix
        scale_x = float(frame_w) / float(width)
        scale_y = float(frame_h) / float(height)
        scaled = np.array(matrix, dtype=np.float64, copy=True)
        scaled[0, 0] *= scale_x
        scaled[0, 2] *= scale_x
        scaled[1, 1] *= scale_y
        scaled[1, 2] *= scale_y
        return scaled

    def get_dist(self) -> np.ndarray:
        with self._lock:
            return self.get_dist_locked()

    def get_dist_locked(self) -> np.ndarray:
        return self._camera.get_dist()

    @property
    def image_width(self) -> int | None:
        return self._camera.image_width

    @property
    def image_height(self) -> int | None:
        return self._camera.image_height

    def is_valid(self, u: float, v: float) -> bool:
        return self._camera.is_valid(u, v)


class MountCalibrationSummary:
    """Expose focal-span planning facts from legacy calibration storage."""

    def __init__(self, camera: CameraCalibrationPort) -> None:
        self._camera = camera

    def zoom_levels(self) -> list[str]:
        zoom_map = self._camera.zoom_map()
        if not zoom_map:
            return []
        return sorted(zoom_map.keys(), key=lambda value: float(value))

    @property
    def zoom_ratio(self) -> float:
        zoom_map = self._camera.zoom_map()
        if not zoom_map or len(zoom_map) < 2:
            return 1.0
        focals = self._positive_focals(zoom_map.values())
        if len(focals) < 2:
            return 1.0
        smallest = min(focals)
        return 1.0 if smallest <= 0 else max(focals) / smallest

    @property
    def base_fy(self) -> float | None:
        zoom_map = self._camera.zoom_map()
        if not zoom_map:
            return None
        focals = self._positive_focals(zoom_map.values())
        return min(focals) if focals else None

    @property
    def has_zoom(self) -> bool:
        required = MIN_CONFIRM_PIXELS / MIN_DETECT_PIXELS
        return self.zoom_ratio >= required

    @staticmethod
    def _positive_focals(
        entries: Iterable[object],
    ) -> list[float]:
        focals: list[float] = []
        for entry in entries:
            if not isinstance(entry, Mapping):
                continue
            focal = entry.get("fy")
            if isinstance(focal, (int, float)) and focal > 0:
                focals.append(float(focal))
        return focals


class MountLiveOptics:
    """Bind one fresh zoom sample to the corresponding camera FOV."""

    def __init__(
        self,
        camera: CameraZoomPort,
        optics: LiveOpticsPort,
        readback: LiveZoomReadbackPort,
        lock: threading.RLock,
    ) -> None:
        self._camera = camera
        self._optics = optics
        self._readback = readback
        self._lock = lock

    def get(self) -> CameraMountOptics | None:
        with self._lock:
            return self._get_locked()

    def _get_locked(self) -> CameraMountOptics | None:
        sample = self._readback.live_sample()
        if sample is not None:
            command = self._readback.zoom_key_from_level(sample.level)
            if command is None:
                return None
            if self._readback.camera_zoom_key() != command:
                if not self._camera.apply_zoom(command).accepted:
                    return None
            return self._from_intrinsics(
                zoom_command=command,
                zoom_level=str(sample.level),
                sample_id=sample.sample_id,
                sample_from_hardware=True,
            )

        if self._readback.has_zoom_readback():
            return None
        command = self._readback.camera_zoom_key()
        return self._from_intrinsics(
            zoom_command=command,
            zoom_level=command,
            sample_id=f"cam:{command}",
            sample_from_hardware=False,
        )

    def _from_intrinsics(
        self,
        *,
        zoom_command: str | None,
        zoom_level: str | None,
        sample_id: str,
        sample_from_hardware: bool,
    ) -> CameraMountOptics | None:
        matrix = self._optics.get_k()
        if getattr(matrix, "shape", None) != (3, 3):
            return None
        width = self._optics.image_width
        height = self._optics.image_height
        if width is None or height is None:
            return None
        fx = float(matrix[0, 0])
        fy = float(matrix[1, 1])
        if fx <= 0.0 or fy <= 0.0 or width <= 0 or height <= 0:
            return None
        if not all(math.isfinite(value) for value in (fx, fy)):
            return None
        return CameraMountOptics(
            zoom_command=zoom_command,
            zoom_level=zoom_level,
            sample_id=str(sample_id),
            fov_h_rad=2.0 * math.atan(float(width) / (2.0 * fx)),
            fov_v_rad=2.0 * math.atan(float(height) / (2.0 * fy)),
            sample_from_hardware=sample_from_hardware,
        )


__all__ = ["MountCalibrationSummary", "MountLiveOptics", "MountOpticsReader"]
