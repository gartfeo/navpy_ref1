"""Simulator-only ideal pixel sensor for the static point-mass diagnostic."""

from __future__ import annotations

import math
import os
from contextlib import contextmanager
from typing import Iterator

import numpy as np

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.navigation.nav.vision_nav.lateral_rate import (
    LAT_GYRO_DELAY_ENV,
)
from navpy.modules.vision.models.pixel_observation import (
    CameraToBodyTransform,
    PixelCalibration,
    PixelObservation,
    PixelProjectionKind,
    VisualDetection,
)
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from navpy.modules.vision.visual_ray_projection import (
    body_ray_to_pixel,
    camera_to_body_from_gimbal,
)
from navpy.utils.euler_utils import get_euler_by_sequence
from navpy.utils.simple_rotation import Rotation


FRAME_WIDTH_PX = 2560.0
FRAME_HEIGHT_PX = 1440.0
FOCAL_X_PX = FRAME_WIDTH_PX / math.pi
FOCAL_Y_PX = FRAME_HEIGHT_PX / math.pi
PRINCIPAL_X_PX = FRAME_WIDTH_PX / 2.0
PRINCIPAL_Y_PX = FRAME_HEIGHT_PX / 2.0

_IDEAL_CALIBRATION = PixelCalibration(
    FOCAL_X_PX,
    FOCAL_Y_PX,
    PRINCIPAL_X_PX,
    PRINCIPAL_Y_PX,
)
_STATIC_CAMERA_TO_BODY = camera_to_body_from_gimbal(
    GimbalData(att=Attitude(0.0, 0.0, 0.0)),
)


@contextmanager
def undelayed_truth_gyro() -> Iterator[None]:
    """Build the law under a zero gyro-delay model, then restore the env.

    This sensor publishes the INSTANTANEOUS truth turn rate — no
    INS_GYRO_FILTER exists in the point-mass world — while the law's default
    delay model (AAS_LAT_GYRO_DELAY_S = 11.25 ms) describes the real filtered
    gyro. A law built outside this context would "undo" a delay that never
    happened and see a false lateral LOS rate whenever the turn rate changes.
    """
    previous = os.environ.get(LAT_GYRO_DELAY_ENV)
    os.environ[LAT_GYRO_DELAY_ENV] = "0"
    try:
        yield
    finally:
        if previous is None:
            os.environ.pop(LAT_GYRO_DELAY_ENV, None)
        else:
            os.environ[LAT_GYRO_DELAY_ENV] = previous


def build_static_detection(
    *,
    position_ned_m: np.ndarray,
    t_s: float,
    pitch_deg: float,
    roll_deg: float,
    yaw_deg: float,
    bearing_noise_rad: float = 0.0,
    vertical_pixel_bias_px: float = 0.0,
    aircraft_yaw_rate_rad_s: float = 0.0,
) -> VisualDetection:
    """Use simulator truth only to publish one frame-local visual detection."""
    u_px, v_px = render_poi_pixel(
        position_ned_m=position_ned_m,
        pitch_deg=pitch_deg,
        roll_deg=roll_deg,
        yaw_deg=yaw_deg,
        calibration=_IDEAL_CALIBRATION,
        camera_to_body=_STATIC_CAMERA_TO_BODY,
        projection=PixelProjectionKind.SPHERICAL_EQUIANGULAR,
        bearing_noise_rad=bearing_noise_rad,
    )
    v_px += float(vertical_pixel_bias_px)
    return VisualDetection(
        task_id=1,
        obj_id=1,
        observation=PixelObservation(
            u_px=u_px,
            v_px=v_px,
            calibration=_IDEAL_CALIBRATION,
            camera_to_body=_STATIC_CAMERA_TO_BODY,
            projection=PixelProjectionKind.SPHERICAL_EQUIANGULAR,
            aircraft_pitch_deg=pitch_deg,
            aircraft_roll_deg=roll_deg,
            source_timestamp_s=t_s,
            pose_is_frame_atomic=True,
            source_name="static-ideal",
            aircraft_yaw_rate_rad_s=aircraft_yaw_rate_rad_s,
        ),
    )


def render_poi_pixel(
    *,
    position_ned_m: np.ndarray,
    pitch_deg: float,
    roll_deg: float,
    yaw_deg: float,
    calibration: PixelCalibration,
    camera_to_body: CameraToBodyTransform,
    projection: PixelProjectionKind,
    bearing_noise_rad: float = 0.0,
) -> tuple[float, float]:
    """Use simulator truth only to synthesize one frame-local pixel."""
    poi_from_ownship_ned = -np.asarray(position_ned_m, dtype=float)
    body_ray = _unit(
        _uas_to_ned_matrix(pitch_deg, yaw_deg, roll_deg).T
        @ poi_from_ownship_ned
    )
    if bearing_noise_rad != 0.0:
        cos_noise = math.cos(bearing_noise_rad)
        sin_noise = math.sin(bearing_noise_rad)
        body_ray = _unit(
            np.asarray(
                [
                    [cos_noise, -sin_noise, 0.0],
                    [sin_noise, cos_noise, 0.0],
                    [0.0, 0.0, 1.0],
                ],
                dtype=float,
            )
            @ body_ray
        )
    return body_ray_to_pixel(
        body_ray,
        calibration,
        camera_to_body,
        projection,
    )


def _uas_to_ned_matrix(
    pitch_deg: float,
    yaw_deg: float,
    roll_deg: float,
) -> np.ndarray:
    attitude = Attitude(pitch_deg, yaw_deg, roll_deg)
    return Rotation.from_euler(
        "ZYX",
        get_euler_by_sequence(attitude, "ZYX"),
        degrees=True,
    ).as_matrix()


def _unit(value: np.ndarray) -> np.ndarray:
    vector = np.asarray(value, dtype=float)
    norm = float(np.linalg.norm(vector))
    if not math.isfinite(norm) or norm <= 0.0:
        raise ValueError("zero vector")
    return vector / norm


__all__ = [
    "FOCAL_X_PX",
    "FOCAL_Y_PX",
    "FRAME_HEIGHT_PX",
    "FRAME_WIDTH_PX",
    "PRINCIPAL_X_PX",
    "PRINCIPAL_Y_PX",
    "build_static_detection",
    "render_poi_pixel",
]
