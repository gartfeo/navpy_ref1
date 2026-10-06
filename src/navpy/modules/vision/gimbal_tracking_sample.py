"""Primitive angular boundary for gimbal rate navigation."""

from __future__ import annotations

import math
from dataclasses import dataclass

from navpy.modules.vision.models.detect_data import DetectedObject


@dataclass(frozen=True)
class GimbalAngularSample:
    yaw_error_rad: float
    pitch_error_rad: float
    source_timestamp_s: float


class GimbalTargetProjector:
    """Project one rich detection into an immutable angular sample."""

    @staticmethod
    def project(
        target: DetectedObject,
        source_timestamp_s: float,
        principal_point: tuple[float, float] | None = None,
    ) -> GimbalAngularSample | None:
        try:
            calibration = target.pixel.calibration
            focal_x = float(calibration.fx_px)
            focal_y = float(calibration.fy_px)
            center_x = float(calibration.cx_px)
            center_y = float(calibration.cy_px)
            pixel_x = float(target.pixel.u_px)
            pixel_y = float(target.pixel.v_px)
            timestamp_s = float(source_timestamp_s)
            if principal_point is not None:
                center_x = float(principal_point[0])
                center_y = float(principal_point[1])
        except (AttributeError, TypeError, ValueError):
            return None
        values = (
            focal_x,
            focal_y,
            center_x,
            center_y,
            pixel_x,
            pixel_y,
            timestamp_s,
        )
        if (
            not all(math.isfinite(value) for value in values)
            or focal_x == 0.0
            or focal_y == 0.0
        ):
            return None
        return GimbalAngularSample(
            yaw_error_rad=(pixel_x - center_x) / focal_x,
            pitch_error_rad=(pixel_y - center_y) / focal_y,
            source_timestamp_s=timestamp_s,
        )


__all__ = ["GimbalAngularSample", "GimbalTargetProjector"]
