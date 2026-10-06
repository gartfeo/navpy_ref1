"""Compose mission, peer, and self-detected navigation task workflows."""

from __future__ import annotations

from navpy.args.nav_args import NavArgs
from navpy.logger.cache_logger import ILogger
from navpy.modules.navigation.approach_strategy import ApproachKind
from navpy.modules.nav.navigation_task_action import NavigationTaskAction
from navpy.modules.nav.peer_poi_notification import (
    PeerPoiNotifier,
    PeerPoiNotifierPorts,
)
from navpy.modules.nav.self_detected_approach import (
    SelfDetectedApproach,
    SelfDetectedApproachPorts,
)
from navpy.modules.nav.poi_selection import PoiSelector
from navpy.modules.nav.mission_navigation import (
    FallbackMissionNavigation,
    FallbackMissionPorts,
    MissionPassPolicy,
    MissionPassPorts,
)
from navpy.modules.nav.nav_composition_types import (
    DetectionReviewOwnership,
    NavigationTaskWorkflows,
    MissionNavigationOwnership,
    NavCapabilities,
    NavStateOwnership,
    PoiMissionOwnership,
    VehicleApproachOwnership,
)
from navpy.modules.nav.peer_geo import PeerGeoAcquisition, PeerGeoTracker
from navpy.modules.nav.peer_approach_start import (
    PeerApproachPorts,
    PeerApproachStarter,
    PeerAssignmentPorts,
    PeerAssignmentSetup,
    PeerSimulationPoiSetup,
)
from navpy.modules.nav.peer_approach_ready_gate import (
    PeerApproachReadyGate,
    PeerApproachReadyPorts,
)
from navpy.modules.nav.peer_navigation import PeerNavigationCoordinator
from navpy.modules.vehicle.vehicle_interface import IVehicle
from navpy.modules.vision.detection_coordination import DetectionCoordination


def compose_mission_navigation_ownership(
    vehicle: IVehicle,
    detection: DetectionCoordination,
    args: NavArgs,
    logger: ILogger,
    approach_kind: ApproachKind,
    state: NavStateOwnership,
    poi: PoiMissionOwnership,
    approach: VehicleApproachOwnership,
    navigation: NavCapabilities,
) -> MissionNavigationOwnership:
    fallback_navigation = FallbackMissionNavigation(
        FallbackMissionPorts(
            next_waypoint=lambda: vehicle.mission_items_next,
            mission_item_count=lambda: vehicle.mission_items_count,
            current_relative=lambda: vehicle.location(True),
            set_sim_poi=detection.simulation.set_sim_poi,
            detector_is_simulation=lambda: detection.simulation.is_simulation,
        ),
        poi.mission,
        state.navigation_task,
        approach.planner,
        approach.commands,
        approach_kind,
        logger,
    )
    mission_pass = MissionPassPolicy(
        MissionPassPorts(
            final_approach_active=lambda: navigation.final_approach.is_active,
            poi_passed_override=navigation.final_approach.poi_passed_override,
            last_visual_bearing_deg=(
                navigation.final_approach.last_measured_lateral_bearing_deg
            ),
            current_absolute=lambda: vehicle.location(False),
            locked_poi_distance=navigation.legacy_pois.locked_distance,
        ),
        args,
        state.navigation_task,
        poi.pass_tracker,
        logger,
    )
    return MissionNavigationOwnership(fallback_navigation, mission_pass)


def compose_navigation_task_workflows(
    vehicle: IVehicle,
    detection: DetectionCoordination,
    logger: ILogger,
    profile: dict,
    approach_kind: ApproachKind,
    state: NavStateOwnership,
    poi: PoiMissionOwnership,
    approach: VehicleApproachOwnership,
    observation: DetectionReviewOwnership,
    navigation: NavCapabilities,
    mission_navigation: MissionNavigationOwnership,
) -> NavigationTaskWorkflows:
    peer_geo_tracker = PeerGeoTracker(
        detection.geo_pointing,
        state.geo_hold,
        navigation.legacy_pois.geo_ref,
        lambda: vehicle.location(False),
        lambda: vehicle.attitude,
        logger,
    )
    peer_geo_acquisition = PeerGeoAcquisition(
        detection.mounts,
        navigation.legacy_pois.geo_ref,
        profile,
        state.geo_hold,
        state.navigation_task,
        observation.network.selected_poi,
        logger,
    )
    peer_navigation = PeerNavigationCoordinator(
        PeerAssignmentSetup(
            PeerAssignmentPorts(
                selected_poi=observation.network.selected_poi,
                absolute_location=approach.commands.absolute_location,
            ),
            state.navigation_task,
            state.geo_hold,
            PeerSimulationPoiSetup(
                lambda: detection.simulation.is_simulation,
                detection.simulation.set_sim_poi,
                lambda: vehicle.mission_items_count,
            ),
        ),
        PeerApproachStarter(
            PeerApproachPorts(
                current_absolute=lambda: vehicle.location(False),
                plan_orbit=lambda *pos, **kw: (
                    mission_navigation.fallback_navigation.plan_orbit_approach(
                        *pos,
                        **kw,
                    )
                ),
                request_guided=lambda: approach.commands.request_guided(),
                dispatch_approach=lambda *pos, **kw: (
                    approach.commands.dispatch_approach(*pos, **kw)
                ),
                save_loiter_radius=(
                    lambda: approach.commands.save_loiter_radius()
                ),
            ),
            state.navigation_task,
            peer_geo_tracker,
            lambda: navigation.final_approach.is_active,
            logger,
        ),
        PeerApproachReadyGate(
            PeerApproachReadyPorts(
                selected_poi=observation.network.selected_poi,
                current_absolute=lambda: vehicle.location(False),
                current_relative=lambda: vehicle.location(True),
            ),
            state.navigation_task,
            approach_kind,
            logger,
        ),
    )
    selector = PoiSelector(
        poi.confirmation_manager,
        observation.retry,
        observation.network.has_task_actor,
    )
    peer_notifier = PeerPoiNotifier(
        PeerPoiNotifierPorts(
            detector_is_simulation=lambda: detection.simulation.is_simulation,
            ground_location=navigation.legacy_pois.ground_location,
            notify=observation.network.notify_pois,
        ),
        poi.confirmation_manager,
        logger,
    )
    self_approach = SelfDetectedApproach(
        SelfDetectedApproachPorts(
            final_approach_active=lambda: navigation.final_approach.is_active,
            detector_is_simulation=lambda: detection.simulation.is_simulation,
            ground_location=navigation.legacy_pois.ground_location,
            current_relative=lambda: vehicle.location(True),
            plan_orbit=lambda *pos, **kw: (
                mission_navigation.fallback_navigation.plan_orbit_approach(
                    *pos,
                    **kw,
                )
            ),
            absolute_location=approach.commands.absolute_location,
            save_loiter_radius=lambda: approach.commands.save_loiter_radius(),
            request_guided=lambda: approach.commands.request_guided(),
            loiter_poi=navigation.vehicle_commands.peer_poi_loiter,
        ),
        state.navigation_task,
        approach_kind,
        logger,
    )
    navigation_task_action = NavigationTaskAction(
        state.navigation_task,
        state.final_approach,
        poi.confirmation_manager,
        observation.retry,
        detection.tracking_commands,
        approach.speedup,
        peer_navigation,
        mission_navigation.fallback_navigation,
        self_approach,
        lambda: navigation.final_approach.is_active,
        logger,
    )
    return NavigationTaskWorkflows(
        mission_navigation=mission_navigation,
        peer_geo_acquisition=peer_geo_acquisition,
        selector=selector,
        peer_notifier=peer_notifier,
        navigation_task_action=navigation_task_action,
    )


__all__ = [
    "compose_navigation_task_workflows",
    "compose_mission_navigation_ownership",
]
