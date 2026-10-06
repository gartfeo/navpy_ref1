"""Sight/delivery metrics and measurement math for the SIYI pixel source."""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np
import pymap3d

from navpy.modules.common.models.location import Location
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.sim.ideal_target_projector import (
    UasFrameConvention,
    target_los_uas,
)
from navpy.modules.vision.visual_ray_projection import observation_body_ray


@dataclass(frozen=True)
class SiyiPixelSourceMetrics:
    visible_frames: int
    sight_losses: int
    delivered_frames: int
    delivery_rejections: int
    first_sight_loss_distance_m: float | None
    longest_sight_loss_run: int
    max_gimbal_age_s: float
    max_reference_attitude_delta_deg: float
    max_visual_truth_ray_error_deg: float


class SiyiSightRecorder:
    """Accumulate sight, delivery, and pose-pair diagnostics for one source.

    Callers hold their own lock around every method; the recorder itself is
    plain mutable state.
    """

    def __init__(
        self,
        geo_ref: object,
        target_location: Location,
    ) -> None:
        self._geo_ref = geo_ref
        self._target_location = target_location
        self._visible_frames = 0
        self._sight_losses = 0
        self._delivered_frames = 0
        self._delivery_rejections = 0
        self._first_sight_loss_distance_m: float | None = None
        self._current_sight_loss_run = 0
        self._longest_sight_loss_run = 0
        self._max_gimbal_age_s = 0.0
        self._max_reference_attitude_delta_deg = 0.0
        self._max_visual_truth_ray_error_deg = 0.0

    def snapshot(self) -> SiyiPixelSourceMetrics:
        return SiyiPixelSourceMetrics(
            self._visible_frames,
            self._sight_losses,
            self._delivered_frames,
            self._delivery_rejections,
            self._first_sight_loss_distance_m,
            self._longest_sight_loss_run,
            self._max_gimbal_age_s,
            self._max_reference_attitude_delta_deg,
            self._max_visual_truth_ray_error_deg,
        )

    def record_sight_loss(self, camera_location: Location) -> None:
        self._sight_losses += 1
        self._current_sight_loss_run += 1
        self._longest_sight_loss_run = max(
            self._longest_sight_loss_run,
            self._current_sight_loss_run,
        )
        if self._first_sight_loss_distance_m is None:
            self._first_sight_loss_distance_m = distance_m(
                camera_location,
                self._target_location,
            )

    def reset_sight_loss_run(self) -> None:
        self._current_sight_loss_run = 0

    def record_visible(self) -> None:
        self._visible_frames += 1

    def record_delivery(self, delivered: bool) -> None:
        if delivered:
            self._delivered_frames += 1
        else:
            self._delivery_rejections += 1

    def record_frame_pair(
        self,
        gimbal_data: object,
        current_attitude: object,
        wall_now_s: Callable[[], float],
    ) -> None:
        timestamp_s = getattr(gimbal_data, "timestamp_s", None)
        if timestamp_s is not None:
            try:
                age_s = max(0.0, wall_now_s() - float(timestamp_s))
            except (TypeError, ValueError):
                pass
            else:
                self._max_gimbal_age_s = max(self._max_gimbal_age_s, age_s)
        reference = getattr(gimbal_data, "reference_aircraft_attitude", None)
        if reference is None:
            return
        try:
            delta_deg = max(
                abs(angle_delta_deg(float(getattr(reference, name)),
                                    float(getattr(current_attitude, name))))
                for name in ("roll", "pitch", "yaw")
            )
        except (AttributeError, TypeError, ValueError):
            return
        self._max_reference_attitude_delta_deg = max(
            self._max_reference_attitude_delta_deg,
            delta_deg,
        )

    def record_visual_truth_error(
        self,
        detection: DetectedObject,
        location: Location,
        attitude: object,
    ) -> None:
        try:
            target_ned = pymap3d.geodetic2ned(
                self._target_location.lat,
                self._target_location.lng,
                self._target_location.alt,
                location.lat,
                location.lng,
                location.alt,
            )
            expected = target_los_uas(
                target_ned,
                attitude,
                detection.pose.gimbal_data,
                UasFrameConvention(self._geo_ref.uas_seq, self._geo_ref.degrees),
            )
            measured = observation_body_ray(detection.pixel)
            cosine = float(np.clip(np.dot(expected, measured), -1.0, 1.0))
            error_deg = math.degrees(math.acos(cosine))
        except (AttributeError, TypeError, ValueError):
            return
        self._max_visual_truth_ray_error_deg = max(
            self._max_visual_truth_ray_error_deg,
            error_deg,
        )


def center_error_px(target: DetectedObject) -> float:
    calibration = target.pixel.calibration
    return math.hypot(
        target.pixel.u_px - calibration.cx_px,
        target.pixel.v_px - calibration.cy_px,
    )


def projected_target_pixels(
    target: DetectedObject,
    camera: Location,
    target_location: Location,
) -> float:
    from navpy.modules.vision.vision_class_profile import get_class_detect_size
    distance = distance_m(camera, target_location)
    return target.pixel.calibration.fy_px * get_class_detect_size(0) / max(
        distance,
        1.0,
    )


def distance_m(camera: Location, target_location: Location) -> float:
    ned = pymap3d.geodetic2ned(
        target_location.lat,
        target_location.lng,
        target_location.alt,
        camera.lat,
        camera.lng,
        camera.alt,
    )
    return float(np.linalg.norm(ned))


def angle_delta_deg(left: float, right: float) -> float:
    return (left - right + 180.0) % 360.0 - 180.0


__all__ = [
    "SiyiPixelSourceMetrics",
    "SiyiSightRecorder",
    "angle_delta_deg",
    "center_error_px",
    "distance_m",
    "projected_target_pixels",
]
