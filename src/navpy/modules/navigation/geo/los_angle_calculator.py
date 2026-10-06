"""Aircraft-relative line-of-sight angle calculations."""

from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np
import pymap3d
from pymavlink.mavextra import wrap_180

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.geo.rotation_utils import (
    calculate_yaw_pitch,
    get_euler_rotation_angles,
)
from navpy.modules.navigation.geo.uas_frame_transform import UasFrameTransform


class LosAngleCalculator:
    """Convert NED rays and geo locations into legacy angle contracts."""

    def __init__(self, frame: UasFrameTransform) -> None:
        self._frame = frame

    def calc_yaw_pitch_proj_att(
        self,
        target_att: Attitude,
        uas_att: Attitude,
    ) -> tuple[float, float]:
        return self.calc_yaw_pitch_proj(self.calc_uas_ned(target_att), uas_att)

    def calc_yaw_pitch_proj(
        self,
        target_ned: Sequence[float] | np.ndarray,
        uas_att: Attitude,
    ) -> tuple[float, float]:
        level_attitude = Attitude(uas_att.pitch, uas_att.yaw, 0)
        ned_to_uas = np.transpose(
            self._frame.rotation_to_ned(level_attitude)
        )
        target_uas = ned_to_uas @ target_ned
        yaw, pitch = calculate_yaw_pitch(target_uas, [1, 0, 0])
        return wrap_180(yaw), wrap_180(pitch)

    def calc_pitch_los(
        self,
        target_ned: Sequence[float] | np.ndarray,
        uas_att: Attitude,
    ) -> float:
        """Return true LOS elevation error, positive nose-down."""
        north = float(target_ned[0])
        east = float(target_ned[1])
        down = float(target_ned[2])
        elevation = math.degrees(math.atan2(down, math.hypot(north, east)))
        return wrap_180(elevation + uas_att.pitch)

    def calc_yaw_pitch(
        self,
        target_ned: Sequence[float] | np.ndarray,
        uas_att: Attitude,
    ) -> tuple[float, float]:
        uas_ned = self.calc_uas_ned(uas_att)
        euler = get_euler_rotation_angles(target_ned, uas_ned, seq="ZYZ")
        pitch = wrap_180(euler[1])
        yaw = wrap_180(euler[0] + euler[2])
        return -yaw, pitch

    def calc_uas_ned(self, attitude: Attitude) -> np.ndarray:
        return self._frame.rotation_to_ned(attitude) @ [1, 0, 0]

    def calc_yaw_pitch_loc(
        self,
        current_loc: Location,
        target_loc: Location,
        uas_att: Attitude,
    ) -> tuple[float, float]:
        uas_ned = self.calc_uas_ned(
            Attitude(uas_att.pitch, uas_att.yaw, 0)
        )
        target_ned = pymap3d.geodetic2ned(
            target_loc.lat,
            target_loc.lng,
            target_loc.alt,
            current_loc.lat,
            current_loc.lng,
            current_loc.alt,
        )
        euler = get_euler_rotation_angles(
            uas_ned,
            target_ned,
            seq="ZYZ",
            degrees=True,
        )
        pitch = wrap_180(euler[1])
        yaw = wrap_180(euler[0] + euler[2])
        return pitch, yaw
