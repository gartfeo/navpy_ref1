"""Default delivery hub (DDH) fallback mission navigation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional, Protocol

from navpy.core.kernel.config.logger_args import LogStatusDest
from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.kernel.location import Location
from navpy.core.skills.acquire.approach_planner import ApproachPlanner
from navpy.core.skills.acquire.approach_strategy import ApproachKind
from navpy.products.delivery.mission_catalog import MissionCatalog
from navpy.core.flow.nav_state import NavigationTaskState
from navpy.core.skills.acquire.vehicle_navigation import VehicleNavigationCommands
from navpy.core.perception.profile.vision_class_profile import DOCK_DETECT_CLASS_ID


@dataclass(frozen=True)
class FallbackMissionPorts:
    next_waypoint: Callable[[], int]
    mission_item_count: Callable[[], int]
    current_relative: Callable[[], Optional[Location]]
    set_sim_poi: Callable[..., None]
    detector_is_simulation: Callable[[], bool]
    #: NavPy's GUIDED request, told to NAV recovery first (request_guided_owned): the transit's GUIDED is NavPy's.
    request_guided: Callable[[], None]


class HubCamera(Protocol):
    """Aims the camera at a known dock while the UAV circles it (core's PeerGeoTracker)."""

    def start(self, poi: Location) -> bool: ...

    def prime(self) -> bool: ...

    def stop(self) -> None: ...


class _NoCamera:
    def start(self, poi: Location) -> bool:
        return False

    def prime(self) -> bool:
        return False

    def stop(self) -> None:
        return None


class FallbackMissionNavigation:
    """Route the aircraft to the appended default delivery hub.

    The hub's circle aims the camera at the hub, as a peer task's circle does: a UAV that starts the circle over the
    hub (after a failed park there) never flies at it, and a forward-looking camera keeps it 90 degrees off the nose
    (SITL 2026-11-10; navpy_recovery.md step 10, item 3).
    """

    def __init__(
        self,
        ports: FallbackMissionPorts,
        catalog: MissionCatalog,
        navigation_task: NavigationTaskState,
        approach_planner: ApproachPlanner,
        commands: VehicleNavigationCommands,
        logger: ILogger,
        camera: Optional[HubCamera] = None,
    ) -> None:
        self._ports = ports
        self._camera = camera or _NoCamera()
        self._catalog = catalog
        self._navigation_task = navigation_task
        self._approach_planner = approach_planner
        self._commands = commands
        self._logger = logger

    def should_nav_to_fallback(self) -> bool:
        if (
            self._catalog.default_delivery_hub is None
            or self._catalog.default_delivery_hub_active
        ):
            return False
        return self._ports.next_waypoint() >= self._ports.mission_item_count() - 1

    def can_return(self) -> bool:
        """A failed one-shot sortie's return (step 10, item 3): the DDH, not yet flown to, wherever the mission is."""
        return (
            self._catalog.default_delivery_hub is not None
            and not self._catalog.default_delivery_hub_active
        )

    def setup(self) -> None:
        poi = self._catalog.default_delivery_hub
        self._catalog.mark_fallback_active()
        self._navigation_task.navigation_poi_location = (
            self._commands.absolute_location(poi)
        )
        if self._ports.detector_is_simulation():
            self._ports.set_sim_poi(
                self._ports.mission_item_count() - 1,
                poi,
                location_type=self._catalog.default_delivery_hub_type,
            )
        plan, loiter_alt = self._approach_planner.plan_orbit_approach(
            poi,
            DOCK_DETECT_CLASS_ID,
            self._ports.current_relative(),
        )
        self._navigation_task.peer_approach_distance_m = plan.offset_distance
        self._navigation_task.orbit_radius_m = plan.orbit_radius or 0.0
        self._navigation_task.orbit_approach_alt_rel_m = loiter_alt
        if plan.orbit_radius and not self._commands.save_loiter_radius():
            self._logger.warning(
                f"Default delivery hub {plan.kind.value}: WP_LOITER_RAD save failed; "
                "radius will not be restored when the navigation task ends",
                key="nav",
            )
        self._ports.request_guided()
        aimed = plan.kind == ApproachKind.ORBIT and self._aim_camera()
        try:
            self._commands.dispatch_approach(plan, loiter_alt_rel=loiter_alt)
        except Exception:
            if aimed:
                self._camera.stop()
            raise
        self._logger.info(
            f"Default delivery hub navigation: {poi} "
            f"({plan.kind.value}, offset={plan.offset_distance:.0f}m"
            f"{f', orbit_r={plan.orbit_radius:.0f}m' if plan.orbit_radius else ''})",
            key="nav",
            dest=LogStatusDest.DRONE,
        )

    def _aim_camera(self) -> bool:
        hub = self._navigation_task.navigation_poi_location
        return hub is not None and self._camera.start(hub) and self._camera.prime()
