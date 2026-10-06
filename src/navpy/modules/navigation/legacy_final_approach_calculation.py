"""Legacy geo-assisted final-approach command calculation."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Optional

import numpy as np

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.geo.geo_ref_calc import GeoRefCalc
from navpy.modules.navigation.nav.nav_law import NavCommandMode, NavContext, NavLaw


NavPoiCalculator = Callable[
    [Optional[Location], Optional[np.ndarray], Optional[float], Optional[Location]],
    Optional[Location],
]


@dataclass(frozen=True)
class LegacyFinalApproachCalculation:
    """Command values and diagnostics produced by one legacy nav calculation."""

    nav_poi_location: Optional[Location]
    yaw_error: float
    pitch_error: float
    cmd_roll_deg: Optional[float]
    cmd_pitch_deg: Optional[float]
    cmd_throttle: Optional[float]


class LegacyFinalApproachCalculator:
    """Calculate one legacy actuator command without issuing or logging it."""

    def __init__(
            self,
            geo_ref: GeoRefCalc,
            nav: NavLaw,
            get_mission_item_location: Callable[[int], Location],
            adjust_nav: Callable[[Location, float], bool],
            is_adjusted: Callable[[], bool],
            calc_nav_poi: NavPoiCalculator,
    ) -> None:
        self._geo_ref = geo_ref
        self._nav = nav
        self._get_mission_item_location = get_mission_item_location
        self._adjust_nav = adjust_nav
        self._is_adjusted = is_adjusted
        self._calc_nav_poi = calc_nav_poi

    def calculate(
            self,
            d_poi_ned: np.ndarray,
            current_location: Location,
            current_attitude: Attitude,
            locked_poi: Optional[Location],
    ) -> LegacyFinalApproachCalculation | None:
        command_distance = 100.0
        yaw_error, _ = self._geo_ref.calc_yaw_pitch_proj(
            d_poi_ned,
            current_attitude,
        )
        pitch_error = self._geo_ref.calc_pitch_los(
            d_poi_ned,
            current_attitude,
        )
        target_yaw = current_attitude.yaw - yaw_error
        if self._adjust_nav(current_location, target_yaw):
            return None

        previous_location = self._previous_location(current_location)
        nav_poi_location = self._calc_nav_poi(
            current_location,
            d_poi_ned,
            command_distance,
            locked_poi,
        )
        nav_command = self._nav.calc(
            NavContext(
                prev_loc=previous_location,
                current_loc=current_location,
                next_loc=nav_poi_location,
                target_bearing_cd=target_yaw * 100,
                poi_ned=d_poi_ned,
                pitch_error=pitch_error,
                distance=command_distance,
            )
        )
        assert nav_command.mode == NavCommandMode.ACTUATOR, (
            "Legacy final approach expects ACTUATOR NavCommand, "
            f"got {nav_command.mode}"
        )
        return LegacyFinalApproachCalculation(
            nav_poi_location=nav_poi_location,
            yaw_error=yaw_error,
            pitch_error=pitch_error,
            cmd_roll_deg=nav_command.cmd_roll_deg,
            cmd_pitch_deg=nav_command.cmd_pitch_deg,
            cmd_throttle=nav_command.cmd_thr,
        )

    def _previous_location(self, current_location: Location) -> Location:
        if not self._is_adjusted():
            return current_location
        mission_location = self._get_mission_item_location(1)
        return Location(
            mission_location.lat,
            mission_location.lng,
            current_location.alt,
            is_absolute=True,
        )


__all__ = [
    "LegacyFinalApproachCalculation",
    "LegacyFinalApproachCalculator",
    "NavPoiCalculator",
]
