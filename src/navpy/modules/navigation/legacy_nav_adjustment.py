"""One-shot mission adjustment used only by legacy geo-assisted navigation."""

from __future__ import annotations

from navpy.modules.common.models.location import Location
from navpy.modules.navigation.legacy_destination_resolver import LegacyNavigationState
from navpy.modules.navigation.mission_planner import MissionPlanner
from navpy.modules.vehicle.vehicle_interface import IVehicle


class LegacyNavAdjustment:
    """Apply the legacy mission adjustment once per navigation phase."""

    def __init__(
        self,
        vehicle: IVehicle,
        mission_planner: MissionPlanner,
        state: LegacyNavigationState,
    ) -> None:
        self._vehicle = vehicle
        self._mission_planner = mission_planner
        self._state = state

    def adjust(self, current: Location, target_bearing: float) -> bool:
        locked_poi = self._state.begin_adjustment()
        if locked_poi is None:
            return False
        adjusted = self._mission_planner.adjust_nav(
            self._vehicle,
            target_bearing,
            current,
            locked_poi,
        )
        self._state.finish_adjustment(
            bool(adjusted) if adjusted is not None else False
        )
        return self._state.is_adjusted()


__all__ = ["LegacyNavAdjustment"]
