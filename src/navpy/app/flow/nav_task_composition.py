"""Compose mission, peer, and self-detected navigation task workflows."""

from __future__ import annotations

from navpy.core.kernel.config.nav_args import NavArgs
from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.skills.acquire.approach_strategy import ApproachKind
from navpy.core.skills.search.navigation_task_action import NavigationTaskAction
from navpy.core.swarm.peer_poi_notification import (
    PeerPoiNotifier,
    PeerPoiNotifierPorts,
)
from navpy.app.flow.nav_decision_composition import request_guided_owned
from navpy.app.flow.nav_acquisition_composition import compose_acquisition
from navpy.core.skills.search.poi_selection import PoiSelector
from navpy.core.skills.search.task_focus import TaskFocus, TaskFocusPorts
from navpy.app.flow.nav_delivery_composition import (
    compose_home_dock_rules,
    compose_work_source,
)
from navpy.products.delivery.fallback_mission_navigation import (
    FallbackMissionNavigation,
    FallbackMissionPorts,
)
from navpy.core.flow.mission_navigation import (
    MissionPassPolicy,
    MissionPassPorts,
)
from navpy.app.flow.nav_composition_types import (
    DetectionReviewOwnership,
    NavigationTaskWorkflows,
    MissionNavigationOwnership,
    NavCapabilities,
    NavStateOwnership,
    PoiMissionOwnership,
    VehicleApproachOwnership,
)
from navpy.core.skills.acquire.peer_geo import PeerGeoAcquisition, PeerGeoTracker
from navpy.core.skills.acquire.peer_approach_start import (
    PeerApproachPorts,
    PeerApproachStarter,
    PeerAssignmentPorts,
    PeerAssignmentSetup,
    PeerSimulationPoiSetup,
)
from navpy.core.skills.acquire.peer_approach_ready_gate import (
    PeerApproachReadyGate,
    PeerApproachReadyPorts,
)
from navpy.core.skills.acquire.peer_navigation import PeerNavigationCoordinator
from navpy.core.swarm.peer_task_priority import (
    PeerTaskPriority,
    PeerTaskPriorityPorts,
)
from navpy.app.flow.nav_peer_reask_composition import compose_peer_reask
from navpy.core.ports.vehicle_contract import IVehicle
from navpy.core.flow.nav_state import NavState
from navpy.core.perception.detection_coordination import DetectionCoordination
from navpy.core.perception.tracking_geo_hand_off import GeoReleasingTracking


def compose_mission_navigation_ownership(
    vehicle: IVehicle,
    detection: DetectionCoordination,
    args: NavArgs,
    logger: ILogger,
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
            request_guided=lambda: request_guided_owned(state, approach),
        ),
        poi.mission,
        state.navigation_task,
        approach.planner,
        approach.commands,
        logger,
    )
    mission_pass = MissionPassPolicy(
        MissionPassPorts(
            guidance_active=lambda: navigation.guidance.is_active,
            poi_passed_override=navigation.guidance.poi_passed_override,
            last_visual_bearing_deg=(
                navigation.guidance.last_measured_lateral_bearing_deg
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
                    approach.planner.plan_orbit_approach(*pos, **kw)
                ),
                request_guided=lambda: request_guided_owned(state, approach),
                dispatch_approach=lambda *pos, **kw: (
                    approach.commands.dispatch_approach(*pos, **kw)
                ),
                save_loiter_radius=(
                    lambda: approach.commands.save_loiter_radius()
                ),
            ),
            state.navigation_task,
            peer_geo_tracker,
            lambda: state.phase.is_current(NavState.NAV),
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
    peer_notifier = PeerPoiNotifier(
        PeerPoiNotifierPorts(
            detector_is_simulation=lambda: detection.simulation.is_simulation,
            ground_location=navigation.legacy_pois.ground_location,
            notify=observation.network.notify_pois,
        ),
        poi.confirmation_manager,
        logger,
    )
    peer_task_priority = PeerTaskPriority(
        PeerTaskPriorityPorts(
            selected_poi=observation.network.selected_poi,
            stop_tracking=detection.tracking_commands.stop_tracking,
        ),
        state.navigation_task,
        poi.confirmation_manager,
        logger,
    )
    focus = _task_focus(detection, state, navigation, peer_task_priority)
    home_dock = compose_home_dock_rules(focus, poi)
    acquisition = compose_acquisition(
        vehicle, detection, profile, state, approach, observation, navigation, home_dock,
        approach_kind, logger, poi.confirmation_manager,
    )
    selector = PoiSelector(
        poi.confirmation_manager,
        observation.retry,
        observation.network.has_task_actor,
        focus,
        home_dock.offer_allowed,
        home_dock.order,
    )
    navigation_task_action = NavigationTaskAction(
        state.navigation_task,
        state.guidance,
        poi.confirmation_manager,
        observation.retry,
        GeoReleasingTracking(
            detection.tracking_commands,
            detection.geo_pointing.stop_geo_tracking,
        ),
        approach.speedup,
        peer_navigation,
        acquisition.task_start,
        lambda: navigation.guidance.is_active,
        logger,
    )
    return NavigationTaskWorkflows(
        mission_navigation=mission_navigation,
        peer_geo_acquisition=peer_geo_acquisition,
        selector=selector,
        peer_notifier=peer_notifier,
        peer_task_priority=peer_task_priority,
        peer_reask=compose_peer_reask(
            state, observation, peer_geo_tracker, approach_kind, logger,
        ),
        work_source=compose_work_source(
            state, peer_task_priority, peer_navigation,
            mission_navigation.fallback_navigation, navigation_task_action,
        ),
        acquisition=acquisition.gate,
    )


def _task_focus(
    detection: DetectionCoordination,
    state: NavStateOwnership,
    navigation: NavCapabilities,
    peer_task: PeerTaskPriority,
) -> TaskFocus:
    task = state.navigation_task
    return TaskFocus(
        TaskFocusPorts(
            assigned=lambda: task.peer_navigation or peer_task.pending(),
            assigned_dock=lambda: (
                task.peer_poi_location if task.peer_navigation else None
            ),
            detector_is_simulation=lambda: detection.simulation.is_simulation,
            ground_location=navigation.legacy_pois.ground_location,
        )
    )


__all__ = [
    "compose_navigation_task_workflows",
    "compose_mission_navigation_ownership",
]
