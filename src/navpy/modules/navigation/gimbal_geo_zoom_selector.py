"""Discrete zoom candidate selection for known-geo acquisition."""

from __future__ import annotations

import math
from collections.abc import Callable, Sequence
from typing import TYPE_CHECKING

import numpy as np

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.navigation.geo.geo_ref_calc import GeoRefCalc
from navpy.modules.navigation.gimbal_navigation_state import GimbalHardware
from navpy.modules.vision.vision_camera_calibration import (
    CameraZoomCalibration,
)

if TYPE_CHECKING:
    from navpy.modules.vision.peripheral.gimbal_abc import GimbalData


class GeoZoomSelector:
    """Pure candidate selection over immutable calibrated intrinsics."""

    def __init__(
        self,
        hardware: GimbalHardware,
        class_detect_size: Callable[[int], float],
        calibrations: Callable[[], tuple[CameraZoomCalibration, ...]],
    ) -> None:
        self._hardware = hardware
        self._class_detect_size = class_detect_size
        self._calibrations = calibrations

    def select(
        self,
        poi_ned: Sequence[float] | np.ndarray,
        gimbal_data: GimbalData,
        uav_att: Attitude,
        geo_ref: GeoRefCalc,
        class_id: int,
        min_pixels: float,
        slant_m: float,
    ) -> tuple[str | None, float]:
        class_size_m = float(self._class_detect_size(class_id))
        min_pixels = float(min_pixels)
        slant_m = float(slant_m)
        if not self._positive_finite(class_size_m):
            return None, 0.0
        if not self._positive_finite(min_pixels):
            return None, 0.0
        if not self._positive_finite(slant_m):
            return None, 0.0

        candidates: list[tuple[float, str, float]] = []
        for calibration in self._calibrations():
            matrix = calibration.intrinsic_matrix()
            focal_y = float(calibration.fy)
            projected_px = focal_y * class_size_m / slant_m
            if not math.isfinite(projected_px) or projected_px <= 0.0:
                continue
            if projected_px < min_pixels:
                continue
            pixel = geo_ref.calc_uv(
                poi_ned,
                matrix,
                gimbal_data,
                uav_att,
            )
            if pixel[0] is None or pixel[1] is None:
                continue
            pixel_u = float(pixel[0])
            pixel_v = float(pixel[1])
            if not math.isfinite(pixel_u) or not math.isfinite(pixel_v):
                continue
            if not self._hardware.mount.is_valid(pixel_u, pixel_v):
                continue
            candidates.append(
                (focal_y, calibration.zoom, projected_px)
            )
        if not candidates:
            return None, 0.0
        _, command_zoom, projected_px = min(
            candidates,
            key=lambda item: item[0],
        )
        return command_zoom, projected_px

    @staticmethod
    def _positive_finite(value: float) -> bool:
        return math.isfinite(value) and value > 0.0


__all__ = ["GeoZoomSelector"]
