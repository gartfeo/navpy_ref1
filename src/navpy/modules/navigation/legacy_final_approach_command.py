"""Legacy geo-assisted final-approach command execution.

This module is intentionally separate from the pure-vision nav runtime.
Its coordinate and vehicle-pose inputs must never become dependencies of the
vision command path.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional, Protocol

import numpy as np

from navpy.logger.cache_logger import ILogger
from navpy.logger.navigation_logger import NavigationLogger
from navpy.modules.common.models.location import Location
from navpy.modules.common.models.attitude import Attitude
from navpy.modules.navigation.calc_data import CalcData
from navpy.modules.navigation.geo.geo_ref_calc import GeoRefCalc
from navpy.modules.navigation.legacy_final_approach_actuation import LegacyFinalApproachActuation
from navpy.modules.navigation.legacy_final_approach_calculation import (
    LegacyFinalApproachCalculator,
)
from navpy.modules.navigation.legacy_final_approach_diagnostics import (
    LegacyFinalApproachDiagnostics,
)
from navpy.modules.navigation.nav.nav_law import NavLaw
from navpy.modules.vision.models.detect_data import DetectedObject


class LegacyFinalApproachVehicle(Protocol):
    @property
    def attitude(self) -> Attitude: ...

    def location(self, is_relative: bool) -> Location | None: ...
    def get_mission_item_location(self, sequence: int) -> Location: ...
    def set_attitude(
        self,
        roll: float | None,
        pitch: float | None,
        yaw: float | None = None,
        thr: float | None = None,
    ) -> object: ...


@dataclass(frozen=True)
class LegacyFinalApproachCommandPorts:
    """Narrow dependencies required by the legacy final-approach command."""

    vehicle: LegacyFinalApproachVehicle
    geo_ref: GeoRefCalc
    logger: ILogger
    navigation_logger: NavigationLogger
    nav: NavLaw
    get_locked_poi: Callable[[], Optional[Location]]
    adjust_nav: Callable[[Location, float], bool]
    is_adjusted: Callable[[], bool]
    calc_nav_poi: Callable[
        [Optional[Location], Optional[np.ndarray], Optional[float], Optional[Location]],
        Optional[Location],
    ]


class LegacyFinalApproachCommand:
    """Execute one command for PID/PN geo-assisted final approach."""

    def __init__(self, ports: LegacyFinalApproachCommandPorts) -> None:
        self._ports = ports
        self._calculator = LegacyFinalApproachCalculator(
            ports.geo_ref,
            ports.nav,
            ports.vehicle.get_mission_item_location,
            ports.adjust_nav,
            ports.is_adjusted,
            ports.calc_nav_poi,
        )
        self._actuation = LegacyFinalApproachActuation(
            ports.vehicle.set_attitude,
        )
        self._diagnostics = LegacyFinalApproachDiagnostics(
            ports.navigation_logger,
            ports.get_locked_poi,
        )

    def execute(
        self,
        d_poi_ned: np.ndarray,
        detect_data: DetectedObject,
    ) -> Optional[CalcData]:
        ports = self._ports
        current_location = ports.vehicle.location(False)
        current_attitude = ports.vehicle.attitude
        locked_poi = ports.get_locked_poi()

        if current_location is None:
            ports.logger.warning("FAILED TO GET CURRENT LOCATION.")
            return None
        if d_poi_ned is None:
            ports.logger.warning("POI NED IS NONE.")
            return None

        calculation = self._calculator.calculate(
            d_poi_ned,
            current_location,
            current_attitude,
            locked_poi,
        )
        if calculation is None:
            return None

        self._actuation.execute(calculation)
        self._diagnostics.log(
            calculation,
            current_location,
            current_attitude,
            detect_data,
        )
        return CalcData(
            calculation.yaw_error,
            calculation.pitch_error,
            calculation.cmd_roll_deg,
            calculation.cmd_pitch_deg,
            calculation.cmd_throttle,
        )
