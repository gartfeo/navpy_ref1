"""Compose navigation state owners and foundational capabilities."""

from __future__ import annotations

from typing import Callable, Optional

from navpy.core.kernel.config.nav_args import NavArgs
from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.kernel.scheduler_cadence import SchedulerCadence
from navpy.core.skills.acquire.approach_strategy import ApproachKind, ApproachPlan
from navpy.core.skills.acquire.approach_planner import ApproachPlanner
from navpy.core.skills.confirm.confirmation_policy import (
    ConfirmationTimingPolicy,
    PoiRetryPolicy,
)
from navpy.core.skills.confirm.confirmation_reporting import (
    ConfirmBlockedReporter,
    ConfirmDebugReporter,
)
from navpy.core.skills.confirm.detection_freshness import DetectionFreshnessPolicy
from navpy.core.flow.detection_snapshot import DetectionSnapshot
from navpy.core.flow.navigation_speedup import (
    NavigationSpeedupLease,
    SimSpeedupParameterPort,
)
from navpy.products.delivery.mission_catalog import MissionCatalog
from navpy.core.flow.nav_clock import NavClock
from navpy.plugins.vehicle.ardupilot.mission_metadata import MissionMetadata
from navpy.app.flow.nav_composition_types import (
    DetectionReviewOwnership,
    NavCapabilities,
    NavStateOwnership,
    PoiMissionOwnership,
    VehicleApproachOwnership,
)
from navpy.core.flow.nav_constants import (
    CONFIRM_FRESH_DETECTION_MAX_AGE_S,
    CONFIRM_REASK_MAX_ATTEMPTS,
    DIST_EPS,
    DIST_INC_MAX,
    PASSED_POI_BEHIND_MIN_DEG,
    POI_CLOSE_DIST,
    POI_REACQUIRE_COOLDOWN_SEC,
)
from navpy.core.swarm.nav_network import NavNetworkRuntime
from navpy.core.flow.nav_state import (
    ConfirmGateState,
    ConfirmOverrideInbox,
    PostDockClimb,
    NavigationTaskState,
    GeoHoldState,
    NavigationFailureLatch,
    NavPhaseState,
    FinalApproachNavState,
)
from navpy.core.flow.nav_status import NavigationStatusReporter
from navpy.core.flow.navigation_zoom import ZoomController
from navpy.core.skills.geo_approach.pass_tracker import LegacyPassTracker
from navpy.core.review.confirmation_manager import ConfirmationManager
from navpy.core.skills.confirm.poi_retry import PoiRetryState
from navpy.core.approach.final_approach_source_admission import (
    FinalApproachSourceAdmission,
    FinalApproachSourcePorts,
)
from navpy.core.approach.final_approach_publication_admission import (
    FinalApproachPublicationAdmission,
    FinalApproachPublicationAdmissionPorts,
)
from navpy.core.approach.final_approach_source_session import FinalApproachSourceSession
from navpy.core.skills.acquire.vehicle_navigation import (
    ApproachCommandPorts,
    LoiterRadiusLease,
    VehicleNavigationCommands,
)
from navpy.core.ports.vehicle_contract import IVehicle
from navpy.core.perception.detection_coordination import DetectionCoordination


ApproachCalculator = Callable[..., ApproachPlan]


def compose_state_ownership(
    scheduler_cadence: Optional[SchedulerCadence],
    admit_failure: Optional[Callable[[], bool]] = None,
) -> NavStateOwnership:
    return NavStateOwnership(
        clock=NavClock(scheduler_cadence),
        phase=NavPhaseState(),
        detections=DetectionSnapshot(),
        navigation_task=NavigationTaskState(),
        navigation_failures=NavigationFailureLatch(admit_failure),
        final_approach=FinalApproachNavState(),
        geo_hold=GeoHoldState(),
        confirm=ConfirmGateState(),
        overrides=ConfirmOverrideInbox(),
        post_dock_climb=PostDockClimb(),
    )


def compose_poi_mission_ownership(
    vehicle: IVehicle,
    detection: DetectionCoordination,
    args: NavArgs,
    logger: ILogger,
    state: NavStateOwnership,
    metadata: MissionMetadata,
) -> PoiMissionOwnership:
    mission = MissionCatalog()
    mission.update(metadata.markers)
    pass_tracker = LegacyPassTracker(
        close_distance_m=POI_CLOSE_DIST,
        distance_epsilon_m=DIST_EPS,
        increase_samples=DIST_INC_MAX,
        behind_bearing_deg=PASSED_POI_BEHIND_MIN_DEG,
    )
    retry_state = PoiRetryState(
        wall_clock_s=lambda: state.clock.wall_s(),
        cooldown_s=POI_REACQUIRE_COOLDOWN_SEC,
        max_reask_attempts=CONFIRM_REASK_MAX_ATTEMPTS,
    )
    confirmation_manager = ConfirmationManager(
        vehicle.source_system,
        args,
        logger,
        is_simulation=detection.simulation.is_simulation,
    )
    return PoiMissionOwnership(
        mission=mission,
        pass_tracker=pass_tracker,
        retry_state=retry_state,
        confirmation_manager=confirmation_manager,
        source_session=FinalApproachSourceSession(detection.events),
    )


def compose_vehicle_approach_ownership(
    vehicle: IVehicle,
    detection: DetectionCoordination,
    args: NavArgs,
    logger: ILogger,
    profile: dict,
    approach_kind: ApproachKind,
    scan_altitude_rel: Optional[float],
    state: NavStateOwnership,
    navigation: NavCapabilities,
    calculate_approach: ApproachCalculator,
) -> VehicleApproachOwnership:
    planner = ApproachPlanner(
        vehicle,
        lambda: detection.mounts.mounts,
        lambda: navigation.final_approach.is_active,
        profile,
        logger,
        approach_kind,
        scan_altitude_rel,
        calculate_approach,
    )
    loiter_radius = LoiterRadiusLease(vehicle, logger)
    commands = VehicleNavigationCommands(
        vehicle=vehicle,
        commands=ApproachCommandPorts(
            goto_poi=navigation.vehicle_commands.peer_poi,
            loiter_poi=navigation.vehicle_commands.peer_poi_loiter,
        ),
        approach_planner=planner,
        loiter_radius=loiter_radius,
    )
    speedup = NavigationSpeedupLease(
        parameter=SimSpeedupParameterPort(
            read=lambda: vehicle.get_parameter("SIM_SPEEDUP"),
            write=lambda value: vehicle.set_parameter("SIM_SPEEDUP", value),
        ),
        requested_speedup=lambda: args.nav_sim_speedup,
        sync_scheduler_cadence=lambda: state.clock.sync_scheduler_cadence(),
        logger=logger,
    )
    logger.info(f"Approach strategy: {approach_kind.value}")
    return VehicleApproachOwnership(
        planner=planner,
        loiter_radius=loiter_radius,
        commands=commands,
        speedup=speedup,
    )


def compose_detection_review_ownership(
    vehicle: IVehicle,
    detection: DetectionCoordination,
    args: NavArgs,
    logger: ILogger,
    profile: dict,
    state: NavStateOwnership,
    poi: PoiMissionOwnership,
    navigation: NavCapabilities,
) -> DetectionReviewOwnership:
    network = NavNetworkRuntime(
        vehicle,
        detection.simulation,
        navigation.legacy_pois.ground_location,
        poi.confirmation_manager,
        state.overrides,
        logger,
    )
    publication_admission = FinalApproachPublicationAdmission(
        FinalApproachPublicationAdmissionPorts(
            clear_discontinuity=(
                navigation.final_approach.clear_source_discontinuity
            ),
            mark_failed=state.navigation_failures.mark_failed,
            logger=logger,
        )
    )
    source = FinalApproachSourceAdmission(
        FinalApproachSourcePorts(
            source_session=poi.source_session,
            active_poi=lambda: poi.confirmation_manager.active_poi,
            event_inbox=state.detections,
            detections=state.detections.pois,
        ),
        publication_admission,
    )
    freshness = DetectionFreshnessPolicy(
        state.detections,
        source,
        lambda: state.clock.source_fallback_s(),
        lambda: state.clock.scheduler_wall_period(
            CONFIRM_FRESH_DETECTION_MAX_AGE_S
        ),
    )
    poi.confirmation_manager.set_poi_freshness_check(
        freshness.is_poi_fresh_for_confirm
    )
    status = NavigationStatusReporter(
        logger,
        args,
        lambda: vehicle.home_location,
    )
    status.initial()
    debug = ConfirmDebugReporter(logger, lambda: state.clock.decision_s())
    blocked = ConfirmBlockedReporter(logger, lambda: state.clock.decision_s())
    return DetectionReviewOwnership(
        network=network,
        source=source,
        publication_admission=publication_admission,
        freshness=freshness,
        retry=PoiRetryPolicy(poi.retry_state, poi.confirmation_manager),
        status=status,
        debug=debug,
        blocked=blocked,
        timing=ConfirmationTimingPolicy(
            state.confirm,
            state.geo_hold,
            args,
            detection.zoom,
            poi.confirmation_manager,
            logger,
            lambda: state.clock.decision_s(),
        ),
        zoom=ZoomController(detection.zoom, logger),
        vision_profile=profile,
    )


__all__ = [
    "ApproachCalculator",
    "compose_detection_review_ownership",
    "compose_state_ownership",
    "compose_poi_mission_ownership",
    "compose_vehicle_approach_ownership",
]
