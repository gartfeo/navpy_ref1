"""Pure planning policy for target approach geometry."""

from __future__ import annotations

import math
from typing import Callable, Optional, Protocol, Sequence

from navpy.args.logger_args import LogStatusDest
from navpy.logger.cache_logger import ILogger
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.approach_strategy import ApproachKind, ApproachPlan
from navpy.modules.navigation.orbit_geometry import OrbitNavigationLimits, r_nav_min
from navpy.modules.navigation.peer_offset import (
    MIN_APPROACH_STANDOFF_M,
    calc_peer_approach_offset,
)
from navpy.modules.vision.vision_class_profile import get_min_pixels_for_class


_ORBIT_AIRSPEED_FALLBACK_MPS = 25.0


class ApproachVehicle(Protocol):
    home_location: Optional[Location]
    lim_roll: float
    min_pitch: float
    air_speed: float


class ApproachPlanner:
    """Compute an approach without owning navigation or vehicle side effects."""

    def __init__(
            self,
            vehicle: ApproachVehicle,
            mounts: Callable[[], Sequence],
            terminal_navigation_enabled: Callable[[], bool],
            vision_profile: dict,
            logger: ILogger,
            plan_offset: Callable = calc_peer_approach_offset,
    ) -> None:
        self._vehicle = vehicle
        self._mounts = mounts
        self._terminal_navigation_enabled = terminal_navigation_enabled
        self._vision_profile = vision_profile
        self._logger = logger
        self._plan_offset = plan_offset

    def orbit_limits(self) -> Optional[OrbitNavigationLimits]:
        """Return the usable fixed-wing envelope, or None if it is unknown."""
        try:
            roll = float(self._vehicle.lim_roll)
            min_pitch = float(self._vehicle.min_pitch)
        except (TypeError, ValueError):
            return None
        if not (math.isfinite(roll) and 0.0 < roll < 90.0):
            return None
        if not (math.isfinite(min_pitch) and 0.0 < abs(min_pitch) < 90.0):
            return None
        try:
            airspeed = float(self._vehicle.air_speed)
        except (TypeError, ValueError):
            airspeed = 0.0
        if not (math.isfinite(airspeed) and airspeed > 0.0):
            airspeed = _ORBIT_AIRSPEED_FALLBACK_MPS
        return OrbitNavigationLimits(
            airspeed_mps=airspeed,
            roll_limit_deg=roll,
            min_pitch_deg=min_pitch,
        )

    def plan(
            self,
            target_loc: Location,
            class_id: int,
            drone_loc: Optional[Location],
            *,
            approach_kind: ApproachKind,
            approach_alt_rel: Optional[float],
    ) -> tuple[ApproachPlan, Optional[float]]:
        """Return an approach plan and any home-relative loiter altitude."""
        orbit_limits = self.orbit_limits()
        use_nav_orbit = (
            approach_kind == ApproachKind.ORBIT
            and approach_alt_rel is not None
            and drone_loc is not None
            and orbit_limits is not None
        )
        plan_drone_loc = drone_loc
        if use_nav_orbit:
            plan_drone_loc, use_nav_orbit = self._planning_location(
                target_loc,
                drone_loc,
                approach_alt_rel,
            )

        recognition_px = get_min_pixels_for_class(
            self._vision_profile,
            class_id,
        )
        plan = self._plan_offset(
            target_loc,
            plan_drone_loc,
            self._mounts(),
            class_id=class_id,
            kind=approach_kind,
            orbit_limits=orbit_limits if use_nav_orbit else None,
            recognition_px=recognition_px,
        )
        plan = self._prefer_terminal_radius(
            plan,
            target_loc=target_loc,
            plan_drone_loc=plan_drone_loc,
            orbit_limits=orbit_limits,
            use_nav_orbit=use_nav_orbit,
        )
        return plan, (approach_alt_rel if use_nav_orbit else None)

    def _planning_location(
            self,
            target_loc: Location,
            drone_loc: Location,
            approach_alt_rel: float,
    ) -> tuple[Location, bool]:
        if target_loc.is_absolute:
            home = self._vehicle.home_location
            if home is None:
                return drone_loc, False
            altitude = home.alt + approach_alt_rel
            is_absolute = True
        else:
            altitude = approach_alt_rel
            is_absolute = False
        return Location(
            drone_loc.lat,
            drone_loc.lng,
            altitude,
            is_absolute=is_absolute,
        ), True

    def _prefer_terminal_radius(
            self,
            plan: ApproachPlan,
            *,
            target_loc: Location,
            plan_drone_loc: Optional[Location],
            orbit_limits: Optional[OrbitNavigationLimits],
            use_nav_orbit: bool,
    ) -> ApproachPlan:
        if not (
            use_nav_orbit
            and plan_drone_loc is not None
            and orbit_limits is not None
            and plan.kind == ApproachKind.ORBIT
            and self._terminal_navigation_enabled()
        ):
            return plan

        altitude_agl = max(plan_drone_loc.alt - target_loc.alt, 0.0)
        terminal_radius = r_nav_min(
            orbit_limits,
            altitude_agl,
            floor_m=MIN_APPROACH_STANDOFF_M,
        )
        camera_radius = float(plan.orbit_radius or 0.0)
        if not math.isclose(
                camera_radius,
                terminal_radius,
                rel_tol=0.0,
                abs_tol=0.5,
        ):
            self._logger.info(
                f"Terminal ORBIT: camera-sized {camera_radius:.0f}m -> "
                f"navigation-feasible {terminal_radius:.0f}m",
                key="nav",
                dest=LogStatusDest.DRONE,
            )
        return ApproachPlan(
            kind=plan.kind,
            approach_location=plan.approach_location,
            offset_distance=plan.offset_distance,
            orbit_radius=terminal_radius,
        )
