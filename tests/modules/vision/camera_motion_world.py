"""Synthetic world for the gimbal feedforward tests (HITL step 24).

NED world. The aircraft has heading psi, pitch and roll; the gimbal is in
LOCK, its camera at world yaw gamma and elevation epsilon with zero roll. The
target LOS is a world azimuth/elevation. The test side knows the heading; the
sample handed to the tracker carries only pixels, the camera-to-body
transform, pitch/roll and the gyro turn rate.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from navpy.core.kernel.simple_rotation import Rotation
from navpy.core.perception.pointing.camera_motion_state import CameraMotionState
from navpy.core.perception.pointing.gimbal_tracking_sample import GimbalAngularSample

# Camera axes (x right, y down, z optical) in a forward/right/down frame.
_CAMERA_AXES = np.array(
    [[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=float
)
FRAME_DT_S = 1.0 / 30.0


@dataclass(frozen=True)
class WorldState:
    heading_deg: float
    pitch_deg: float
    roll_deg: float
    turn_rate_dps: float
    camera_yaw_deg: float
    camera_elevation_deg: float
    los_azimuth_deg: float
    los_elevation_deg: float


def camera_world(yaw_deg: float, elevation_deg: float) -> np.ndarray:
    return (
        Rotation.from_euler("ZYX", [yaw_deg, elevation_deg, 0.0], degrees=True)
        .as_matrix()
        @ _CAMERA_AXES
    )


def body_world(state: WorldState) -> np.ndarray:
    return Rotation.from_euler(
        "ZYX",
        [state.heading_deg, state.pitch_deg, state.roll_deg],
        degrees=True,
    ).as_matrix()


def los_world(azimuth_deg: float, elevation_deg: float) -> np.ndarray:
    azimuth = math.radians(azimuth_deg)
    elevation = math.radians(elevation_deg)
    return np.array(
        [
            math.cos(elevation) * math.cos(azimuth),
            math.cos(elevation) * math.sin(azimuth),
            -math.sin(elevation),
        ]
    )


def sample(
    state: WorldState,
    timestamp_s: float,
    *,
    readback_error_deg: tuple[float, float] = (0.0, 0.0),
    pixel_error_rad: tuple[float, float] = (0.0, 0.0),
    gyro_error_dps: float = 0.0,
) -> GimbalAngularSample:
    """The tracker's view of one frame: pixels plus the frame's camera state."""
    true_camera = camera_world(state.camera_yaw_deg, state.camera_elevation_deg)
    ray = true_camera.T @ los_world(state.los_azimuth_deg, state.los_elevation_deg)
    if ray[2] <= 0.0:
        raise ValueError("target behind the camera")
    reported_camera = camera_world(
        state.camera_yaw_deg + readback_error_deg[0],
        state.camera_elevation_deg + readback_error_deg[1],
    )
    camera_to_body = body_world(state).T @ reported_camera
    return GimbalAngularSample(
        float(ray[0] / ray[2]) + pixel_error_rad[0],
        float(ray[1] / ray[2]) + pixel_error_rad[1],
        timestamp_s,
        CameraMotionState(
            tuple(float(value) for value in camera_to_body.reshape(9)),
            state.pitch_deg,
            state.roll_deg,
            math.radians(state.turn_rate_dps + gyro_error_dps),
        ),
    )


def wrap_deg(angle_deg: float) -> float:
    return (angle_deg + 180.0) % 360.0 - 180.0


__all__ = [
    "FRAME_DT_S",
    "WorldState",
    "body_world",
    "camera_world",
    "los_world",
    "sample",
    "wrap_deg",
]
