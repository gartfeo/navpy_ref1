"""Convert stabilized simulated-gimbal state to body-frame readback.

The readback is yaw-outer, as the real SIYI gimbal (siyi_convention.py): body = Rz(yaw) Ry(pitch) Rx(roll).
"""

from __future__ import annotations

import math

import numpy as np

from navpy.core.kernel.attitude import Attitude
from navpy.sim.gimbal.siyi.gimbal_angles import (
    GimbalAngularSnapshot,
    MODE_FOLLOW,
    MODE_FPV,
    clamp,
)


def _rot_x(degrees: float) -> np.ndarray:
    radians = math.radians(degrees)
    cosine, sine = math.cos(radians), math.sin(radians)
    return np.array([[1, 0, 0], [0, cosine, -sine], [0, sine, cosine]])


def _rot_y(degrees: float) -> np.ndarray:
    radians = math.radians(degrees)
    cosine, sine = math.cos(radians), math.sin(radians)
    return np.array([[cosine, 0, sine], [0, 1, 0], [-sine, 0, cosine]])


def _rot_z(degrees: float) -> np.ndarray:
    radians = math.radians(degrees)
    cosine, sine = math.cos(radians), math.sin(radians)
    return np.array([[cosine, -sine, 0], [sine, cosine, 0], [0, 0, 1]])


def zyx_to_matrix(yaw: float, pitch: float, roll: float) -> np.ndarray:
    return _rot_z(yaw) @ _rot_y(pitch) @ _rot_x(roll)


def matrix_to_zyx(matrix: np.ndarray) -> tuple[float, float, float]:
    """The inverse of zyx_to_matrix: (yaw, pitch, roll) in degrees, pitch in [-90, 90]."""
    pitch = math.degrees(math.asin(clamp(-float(matrix[2, 0]), -1.0, 1.0)))
    roll = math.degrees(math.atan2(matrix[2, 1], matrix[2, 2]))
    yaw = math.degrees(math.atan2(matrix[1, 0], matrix[0, 0]))
    return yaw, pitch, roll


def body_readback(snapshot: GimbalAngularSnapshot) -> Attitude:
    if snapshot.motion_mode == MODE_FPV:
        return Attitude(snapshot.pitch, snapshot.yaw, snapshot.roll)
    vehicle = snapshot.vehicle_attitude
    world_yaw = snapshot.yaw
    if snapshot.motion_mode == MODE_FOLLOW:
        world_yaw = vehicle.yaw + snapshot.yaw
    vehicle_rotation = zyx_to_matrix(
        vehicle.yaw,
        vehicle.pitch,
        vehicle.roll,
    )
    world_rotation = zyx_to_matrix(world_yaw, snapshot.pitch, 0.0)
    yaw, pitch, roll = matrix_to_zyx(vehicle_rotation.T @ world_rotation)
    return Attitude(pitch, yaw, roll)


__all__ = ["body_readback", "matrix_to_zyx", "zyx_to_matrix"]
