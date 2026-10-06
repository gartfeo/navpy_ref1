"""SIYI world-frame gimbal LOCK geometry."""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import replace
from typing import TYPE_CHECKING

import numpy as np
import pymap3d
from pymavlink.mavextra import wrap_180

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.geo.uas_frame_transform import UasFrameTransform
from navpy.utils.euler_utils import get_att_by_sequence, get_euler_by_sequence
from navpy.utils.simple_rotation import Rotation

if TYPE_CHECKING:
    from navpy.modules.vision.peripheral.gimbal_abc import GimbalData


class GimbalLockCalculator:
    """Build LOCK commands and convert them back to body-frame readback."""

    def __init__(self, frame: UasFrameTransform) -> None:
        self._frame = frame

    def calc_att_loc(
        self,
        current_loc: Location,
        target_loc: Location,
        uas_att: Attitude,
        g_data: GimbalData,
    ) -> Attitude:
        target_ned = pymap3d.geodetic2ned(
            target_loc.lat,
            target_loc.lng,
            target_loc.alt,
            current_loc.lat,
            current_loc.lng,
            current_loc.alt,
        )
        return self.calc_att_ned(target_ned, uas_att, g_data)

    def calc_att_ned(
        self,
        target_ned: Sequence[float] | np.ndarray,
        uas_att: Attitude,
        g_data: GimbalData,
    ) -> Attitude:
        target_direction = self._target_direction(
            target_ned,
            uas_att,
            g_data,
        )
        camera_axis = self._camera_axis(g_data)
        pitch, yaw = _solve_lock_yaw_pitch(target_direction, camera_axis)
        return Attitude(pitch, yaw, 0.0)

    def calc_readback(
        self,
        command_att: Attitude,
        uas_att: Attitude,
        g_data: GimbalData,
    ) -> GimbalData:
        uas_to_ned = self._frame.rotation_to_ned(uas_att)
        gimbal_to_ned = Rotation.from_euler(
            "ZYX",
            [command_att.yaw, command_att.pitch, command_att.roll],
            degrees=True,
        ).as_matrix()
        gimbal_to_uas = np.transpose(uas_to_ned) @ gimbal_to_ned
        euler = Rotation.from_matrix(gimbal_to_uas).as_euler(
            g_data.args.g_seq,
            degrees=g_data.args.degrees,
        )
        return replace(
            g_data,
            att=get_att_by_sequence(euler, g_data.args.g_seq),
        )

    def _target_direction(
        self,
        target_ned: Sequence[float] | np.ndarray,
        uas_att: Attitude,
        g_data: GimbalData,
    ) -> np.ndarray:
        target = _vec3(target_ned, "target_ned")
        gimbal_offset = np.asarray(
            g_data.args.setup_dist,
            dtype=float,
        ).reshape(3)
        from_gimbal = target - (
            self._frame.rotation_to_ned(uas_att) @ gimbal_offset
        )
        return _unit_vector(from_gimbal, "target_from_gimbal_ned")

    @staticmethod
    def _camera_axis(g_data: GimbalData) -> np.ndarray:
        camera_to_gimbal = Rotation.from_euler(
            g_data.args.setup_seq,
            get_euler_by_sequence(
                g_data.args.setup_att,
                g_data.args.setup_seq,
            ),
            degrees=g_data.args.setup_degrees,
        ).as_matrix()
        return _unit_vector(
            camera_to_gimbal @ np.array([0.0, 0.0, 1.0]),
            "camera_center_axis_gimbal",
        )


def _solve_lock_yaw_pitch(
    target_dir_ned: np.ndarray,
    optical_axis_gimbal: np.ndarray,
) -> tuple[float, float]:
    target_dir_ned = _unit_vector(target_dir_ned, "target_dir_ned")
    optical_axis_gimbal = _unit_vector(
        optical_axis_gimbal,
        "optical_axis_gimbal",
    )
    axis_x, axis_y, axis_z = (
        float(optical_axis_gimbal[index]) for index in range(3)
    )
    xz_norm = math.hypot(axis_x, axis_z)
    if xz_norm <= 1e-9:
        raise ValueError(
            "camera optical axis cannot be aimed by yaw/pitch LOCK command"
        )

    target_down = float(target_dir_ned[2])
    if abs(target_down) > xz_norm + 1e-9:
        raise ValueError(
            "target direction is outside yaw/pitch LOCK command manifold"
        )
    target_down = max(-xz_norm, min(xz_norm, target_down))

    phase = math.atan2(axis_x, axis_z)
    candidates = []
    solution_angle = math.acos(target_down / xz_norm)
    for solution in (solution_angle, -solution_angle):
        pitch_rad = _wrap_pi(solution - phase)
        horizontal_x = (
            math.cos(pitch_rad) * axis_x
            + math.sin(pitch_rad) * axis_z
        )
        yaw_rad = math.atan2(target_dir_ned[1], target_dir_ned[0])
        yaw_rad = _wrap_pi(yaw_rad - math.atan2(axis_y, horizontal_x))
        aimed = Rotation.from_euler(
            "ZYX",
            [math.degrees(yaw_rad), math.degrees(pitch_rad), 0.0],
            degrees=True,
        ).apply(optical_axis_gimbal)
        error = float(np.linalg.norm(aimed - target_dir_ned))
        candidates.append((error, abs(pitch_rad), pitch_rad, yaw_rad))

    min_error = min(item[0] for item in candidates)
    if min_error > 1e-6:
        raise ValueError(
            "could not solve yaw/pitch LOCK command for target direction"
        )
    valid = [item for item in candidates if item[0] <= min_error + 1e-9]
    _, _, pitch_rad, yaw_rad = min(valid, key=lambda item: item[1])
    return wrap_180(math.degrees(pitch_rad)), wrap_180(math.degrees(yaw_rad))


def _unit_vector(
    value: Sequence[float] | np.ndarray,
    name: str,
) -> np.ndarray:
    vector = _vec3(value, name)
    norm = float(np.linalg.norm(vector))
    if not math.isfinite(norm) or norm <= 0.0:
        raise ValueError(f"{name} must be non-zero")
    return vector / norm


def _vec3(
    value: Sequence[float] | np.ndarray,
    name: str,
) -> np.ndarray:
    vector = np.asarray(value, dtype=float).reshape(-1)
    if vector.size != 3:
        raise ValueError(f"{name} must be a 3-vector")
    return vector


def _wrap_pi(angle: float) -> float:
    wrapped = (angle + math.pi) % (2.0 * math.pi) - math.pi
    return math.pi if abs(wrapped + math.pi) < 1e-12 else wrapped
