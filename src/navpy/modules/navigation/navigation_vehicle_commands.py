"""Non-final-approach vehicle commands owned outside the Navigation root."""

from __future__ import annotations

from typing import Optional, Protocol

from navpy.modules.common.models.location import Location


class NavigationCommandVehicle(Protocol):
    def location(self, is_relative: bool) -> Location | None: ...
    def goto(self, target: Location) -> object: ...
    def goto_loiter(self, target: Location, radius: float) -> object: ...


class NavigationVehicleCommands:
    """Issue peer navigation and climb commands through explicit vehicle IO."""

    def __init__(self, vehicle: NavigationCommandVehicle) -> None:
        self._vehicle = vehicle

    def peer_poi(self, poi: Location) -> None:
        relative_poi = Location(
            poi.lat,
            poi.lng,
            self._vehicle.location(True).alt,
            is_absolute=False,
        )
        self._vehicle.goto(relative_poi)

    def peer_poi_loiter(
        self,
        poi: Location,
        radius: float,
        alt_rel_m: Optional[float] = None,
    ) -> None:
        altitude = (
            self._vehicle.location(True).alt
            if alt_rel_m is None
            else alt_rel_m
        )
        relative_poi = Location(
            poi.lat,
            poi.lng,
            altitude,
            is_absolute=False,
        )
        self._vehicle.goto_loiter(relative_poi, radius)
