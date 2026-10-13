"""Shared pixel/ray math for real and simulator vision observations."""

from __future__ import annotations

import math

import numpy as np

from navpy.core.ports.pixel_observation import (
    CameraToBodyTransform,
    PixelCalibration,
    PixelObservation,
    PixelProjectionKind,
)
from navpy.core.ports.gimbal_abc import GimbalData
from navpy.core.kernel.euler_utils import get_euler_by_sequence
from navpy.core.kernel.simple_rotation import Rotation


def camera_to_body_from_gimbal(gimbal: GimbalData) -> CameraToBodyTransform:
    camera_to_gimbal = Rotation.from_euler(
        gimbal.setup_seq,
        get_euler_by_sequence(gimbal.setup_att, gimbal.setup_seq),
        degrees=gimbal.setup_degrees,
    ).as_matrix()
    gimbal_to_body = Rotation.from_euler(
        gimbal.g_seq,
        get_euler_by_sequence(gimbal.att, gimbal.g_seq),
        degrees=gimbal.degrees,
    ).as_matrix()
    return CameraToBodyTransform.from_matrix(gimbal_to_body @ camera_to_gimbal)


def body_ray_to_pixel(
    body_ray: object,
    calibration: PixelCalibration,
    camera_to_body: CameraToBodyTransform,
    projection: PixelProjectionKind,
) -> tuple[float, float]:
    ray = unit_vector(camera_to_body.matrix().T @ unit_vector(body_ray))
    if projection is PixelProjectionKind.SPHERICAL_EQUIANGULAR:
        alpha = math.atan2(float(ray[0]), float(ray[2]))
        beta = math.atan2(float(ray[1]), math.hypot(float(ray[0]), float(ray[2])))
        return (
            calibration.cx_px + calibration.fx_px * alpha,
            calibration.cy_px + calibration.fy_px * beta,
        )
    if projection is PixelProjectionKind.PINHOLE:
        if float(ray[2]) <= 0.0:
            raise ValueError("pinhole ray is behind the camera")
        return (
            calibration.cx_px + calibration.fx_px * float(ray[0]) / float(ray[2]),
            calibration.cy_px + calibration.fy_px * float(ray[1]) / float(ray[2]),
        )
    raise ValueError("unsupported pixel projection")


def observation_body_ray(observation: PixelObservation) -> np.ndarray:
    camera_ray = observation_camera_ray(observation)
    return unit_vector(observation.camera_to_body.matrix() @ camera_ray)


def observation_camera_ray(observation: PixelObservation) -> np.ndarray:
    """Unit ray of the measured pixel in the camera frame (+z = optical axis)."""
    values = (
        observation.u_px,
        observation.v_px,
        observation.aircraft_pitch_deg,
        observation.aircraft_roll_deg,
        observation.source_timestamp_s,
    )
    if not all(_finite(value) for value in values):
        raise ValueError("visual observation must be finite")
    calibration = observation.calibration
    alpha = (float(observation.u_px) - calibration.cx_px) / calibration.fx_px
    beta = (float(observation.v_px) - calibration.cy_px) / calibration.fy_px
    if observation.projection is PixelProjectionKind.SPHERICAL_EQUIANGULAR:
        camera_ray = np.asarray(
            [
                math.cos(beta) * math.sin(alpha),
                math.sin(beta),
                math.cos(beta) * math.cos(alpha),
            ],
            dtype=float,
        )
    elif observation.projection is PixelProjectionKind.PINHOLE:
        camera_ray = np.asarray([alpha, beta, 1.0], dtype=float)
    else:
        raise ValueError("unsupported pixel projection")
    return unit_vector(camera_ray)


def camera_off_axis_deg(camera_ray: np.ndarray) -> float:
    """Angle between a camera-frame ray and the optical axis (+z)."""
    cosine = float(np.clip(unit_vector(camera_ray)[2], -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def camera_half_fov_deg(
    calibration: PixelCalibration,
    projection: PixelProjectionKind,
    image_size_px: tuple[float, float] | None,
) -> float:
    """Smallest angle from the optical axis to an image edge (deg).

    Uses the real image bounds, so an off-centre principal point narrows the
    side it is closer to. Unknown or inconsistent bounds return 0.0, which a
    completion check treats as "never centred" (fail closed).
    """
    if image_size_px is None:
        return 0.0
    image_width_px, image_height_px = image_size_px
    edges_px = (
        (calibration.cx_px, calibration.fx_px),
        (float(image_width_px) - calibration.cx_px, calibration.fx_px),
        (calibration.cy_px, calibration.fy_px),
        (float(image_height_px) - calibration.cy_px, calibration.fy_px),
    )
    if not all(math.isfinite(d) and d > 0.0 for d, _f in edges_px):
        return 0.0
    nearest = min(distance / focal for distance, focal in edges_px)
    if projection is PixelProjectionKind.SPHERICAL_EQUIANGULAR:
        return min(180.0, math.degrees(nearest))
    if projection is PixelProjectionKind.PINHOLE:
        return math.degrees(math.atan(nearest))
    raise ValueError("unsupported pixel projection")


def unit_vector(value: object) -> np.ndarray:
    try:
        vector = np.asarray(value, dtype=float).reshape(-1)
    except (TypeError, ValueError) as exc:
        raise ValueError("ray must be numeric") from exc
    if vector.size != 3 or not np.all(np.isfinite(vector)):
        raise ValueError("ray must be a finite vector")
    norm = float(np.linalg.norm(vector))
    if not math.isfinite(norm) or norm <= 0.0:
        raise ValueError("ray must be non-zero")
    return vector / norm


def _finite(value: object) -> bool:
    if isinstance(value, bool):
        return False
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


__all__ = [
    "body_ray_to_pixel",
    "camera_half_fov_deg",
    "camera_off_axis_deg",
    "camera_to_body_from_gimbal",
    "observation_body_ray",
    "observation_camera_ray",
    "unit_vector",
]
