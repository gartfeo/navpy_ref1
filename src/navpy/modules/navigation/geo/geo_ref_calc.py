"""Stable geo-reference API composed from focused geometry services."""

from __future__ import annotations

from typing import TYPE_CHECKING

from navpy.args.uas_args import UasArgs
from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.geo.camera_ned_projector import CameraNedProjector
from navpy.modules.navigation.geo.geo_location_metrics import (
    calculate_bearing,
    calculate_distance,
)
from navpy.modules.navigation.geo.gimbal_lock_calculator import GimbalLockCalculator
from navpy.modules.navigation.geo.los_angle_calculator import LosAngleCalculator
from navpy.modules.navigation.geo.uas_frame_transform import UasFrameTransform

if TYPE_CHECKING:
    from navpy.modules.vision.peripheral.gimbal_abc import GimbalData


class GeoRefCalc:
    """Public composition boundary for camera and legacy geo geometry."""

    def __init__(self, args: UasArgs, enable_log: bool = False) -> None:
        self.uas_seq = args.uas_seq
        self.degrees = args.degrees
        self.utm_x = args.utm_x
        self.utm_y = args.utm_y
        self.utm_z = args.utm_z
        self.enable_log = enable_log
        frame = UasFrameTransform(
            sequence=self.uas_seq,
            degrees=self.degrees,
            translation_ned=(self.utm_x, self.utm_y, self.utm_z),
        )
        self._camera_projector = CameraNedProjector(frame, enable_log)
        self._los_angles = LosAngleCalculator(frame)
        self._gimbal_lock = GimbalLockCalculator(frame)

    def calc_ned(self, u, v, k, g_data: GimbalData, uas_att: Attitude):
        return self._camera_projector.calc_ned(u, v, k, g_data, uas_att)

    def calc_uv(self, p_ned, k, g_data: GimbalData, uas_att: Attitude):
        return self._camera_projector.calc_uv(p_ned, k, g_data, uas_att)

    def calc_yaw_pitch_proj_att(self, poi_att, uas_att: Attitude):
        return self._los_angles.calc_yaw_pitch_proj_att(poi_att, uas_att)

    def calc_yaw_pitch_proj(self, poi_ned, uas_att: Attitude):
        return self._los_angles.calc_yaw_pitch_proj(poi_ned, uas_att)

    def calc_pitch_los(self, poi_ned, uas_att: Attitude):
        return self._los_angles.calc_pitch_los(poi_ned, uas_att)

    def calc_yaw_pitch(self, poi_ned, uas_att: Attitude):
        return self._los_angles.calc_yaw_pitch(poi_ned, uas_att)

    def calc_uas_ned(self, att: Attitude):
        return self._los_angles.calc_uas_ned(att)

    def calc_yaw_pitch_loc(
        self,
        current_loc: Location,
        poi_loc: Location,
        uas_att: Attitude,
    ):
        return self._los_angles.calc_yaw_pitch_loc(
            current_loc,
            poi_loc,
            uas_att,
        )

    def calc_gimbal_lock_att_loc(
        self,
        current_loc: Location,
        poi_loc: Location,
        uas_att: Attitude,
        g_data: GimbalData,
    ) -> Attitude:
        return self._gimbal_lock.calc_att_loc(
            current_loc,
            poi_loc,
            uas_att,
            g_data,
        )

    def calc_gimbal_lock_att_ned(
        self,
        poi_ned,
        uas_att: Attitude,
        g_data: GimbalData,
    ) -> Attitude:
        return self._gimbal_lock.calc_att_ned(poi_ned, uas_att, g_data)

    def calc_gimbal_lock_readback(
        self,
        command_att: Attitude,
        uas_att: Attitude,
        g_data: GimbalData,
    ) -> GimbalData:
        return self._gimbal_lock.calc_readback(command_att, uas_att, g_data)

    @staticmethod
    def calculate_distance(
        current_loc: Location | None,
        poi_loc: Location | None,
    ):
        return calculate_distance(current_loc, poi_loc)

    @staticmethod
    def calculate_bearing(current_loc: Location, poi_loc: Location):
        return calculate_bearing(current_loc, poi_loc)
