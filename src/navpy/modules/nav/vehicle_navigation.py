from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional, Protocol

from navpy.logger.cache_logger import ILogger
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.approach_strategy import ApproachPlan
from navpy.modules.navigation.orbit_geometry import OrbitNavigationLimits
from navpy.modules.nav.approach_planner import ApproachPlanner
from navpy.modules.vehicle.flight_mode import FlightMode
from navpy.modules.vehicle.vehicle_capability_interfaces import VehicleParameters


class NavigationModeLocation(Protocol):
    @property
    def get_mode(self) -> FlightMode | None: ...

    @property
    def home_location(self) -> Location | None: ...

    def set_mode(self, flight_mode: FlightMode) -> bool: ...


@dataclass(frozen=True)
class ApproachCommandPorts:
    """Narrow peer-approach command surface owned by Navigation composition."""

    goto_poi: Callable[[Location], None]
    loiter_poi: Callable[[Location, float, Optional[float]], None]


class LoiterRadiusLease:
    """Capture and restore WP_LOITER_RAD exactly once per navigation_task."""

    def __init__(self, vehicle: VehicleParameters, logger: ILogger) -> None:
        self._vehicle = vehicle
        self._logger = logger
        self.original: Optional[float] = None

    def acquire(self) -> bool:
        if self.original is not None:
            return True
        value = self._vehicle.get_parameter("WP_LOITER_RAD")
        if value is None:
            return False
        self.original = value
        return True

    def restore(self) -> None:
        if self.original is None:
            return
        previous = self.original
        self._vehicle.set_parameter("WP_LOITER_RAD", previous)
        self._logger.info(
            f"RESTORE WP_LOITER_RAD={previous:.0f}m",
            key="nav",
        )
        self.original = None


class VehicleNavigationCommands:
    """Vehicle-facing approach, loiter, and flight-mode commands."""

    def __init__(
        self,
        *,
        vehicle: NavigationModeLocation,
        commands: ApproachCommandPorts,
        approach_planner: ApproachPlanner,
        loiter_radius: LoiterRadiusLease,
    ) -> None:
        self._vehicle = vehicle
        self._commands = commands
        self._approach_planner = approach_planner
        self._loiter_radius = loiter_radius

    def orbit_limits(self) -> Optional[OrbitNavigationLimits]:
        return self._approach_planner.orbit_limits()

    def dispatch_approach(
        self,
        plan: ApproachPlan,
        loiter_alt_rel: Optional[float] = None,
    ) -> None:
        radius = plan.orbit_radius or 0.0
        if radius > 0:
            self._commands.loiter_poi(
                plan.approach_location,
                radius,
                loiter_alt_rel,
            )
        else:
            self._commands.goto_poi(plan.approach_location)

    def save_loiter_radius(self) -> bool:
        return self._loiter_radius.acquire()

    def restore_loiter_radius(self) -> None:
        self._loiter_radius.restore()

    def request_guided(self) -> bool:
        if self._vehicle.get_mode == FlightMode.GUIDED:
            return False
        self._vehicle.set_mode(FlightMode.GUIDED)
        return True

    def absolute_location(self, location: Optional[Location]) -> Optional[Location]:
        if location is None or location.is_absolute:
            return location
        home = self._vehicle.home_location
        if home is None:
            return None
        return Location(
            location.lat,
            location.lng,
            home.alt + location.alt,
            heading=location.heading,
            is_absolute=True,
        )
