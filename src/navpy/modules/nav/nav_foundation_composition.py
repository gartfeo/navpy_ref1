"""Compose navigation state owners and foundational capabilities."""

from __future__ import annotations

from typing import Callable, Optional

from navpy.args.nav_args import NavArgs
from navpy.logger.cache_logger import ILogger
from navpy.modules.common.scheduler_cadence import SchedulerCadence
from navpy.modules.navigation.approach_strategy import ApproachKind, ApproachPlan
from navpy.modules.nav.approach_planner import ApproachPlanner
from navpy.modules.nav.confirmation_policy import (
    ConfirmationTimingPolicy,
    TargetRetryPolicy,
)
from navpy.modules.nav.confirmation_reporting import (
    ConfirmBlockedReporter,
    ConfirmDebugReporter,
)
from navpy.modules.nav.detection_freshness import DetectionFreshnessPolicy
from navpy.modules.nav.detection_snapshot import DetectionSnapshot
from navpy.modules.nav.navigation_speedup import (
    NavigationSpeedupLease,
    SimSpeedupParameterPort,
)
from navpy.modules.nav.mission_catalog import MissionCatalog
from navpy.modules.nav.nav_clock import NavClock
from navpy.modules.nav.nav_composition_types import (
    DetectionReviewOwnership,
    NavCapabilities,
    NavStateOwnership,
    TargetMissionOwnership,
    VehicleApproachOwnership,
)
from navpy.modules.nav.nav_constants import (
    CONFIRM_FRESH_DETECTION_MAX_AGE_S,
    CONFIRM_REASK_MAX_ATTEMPTS,
    DIST_EPS,
    DIST_INC_MAX,
    PASSED_TARGET_BEHIND_MIN_DEG,
    TARGET_CLOSE_DIST,
    TARGET_REACQUIRE_COOLDOWN_SEC,
)
from navpy.modules.nav.nav_network import NavNetworkRuntime
from navpy.modules.nav.nav_state import (
    ConfirmGateState,
    ConfirmOverrideInbox,
    NavigationTaskState,
    GeoHoldState,
    NavigationFailureLatch,
    NavPhaseState,
    TerminalNavState,
)
from navpy.modules.nav.nav_status import NavigationStatusReporter
from navpy.modules.nav.navigation_zoom import ZoomController
from navpy.modules.nav.pass_tracker import LegacyPassTracker
from navpy.modules.nav.confirmation_manager import ConfirmationManager
from navpy.modules.nav.target_retry import TargetRetryState
from navpy.modules.nav.terminal_source_admission import (
    TerminalSourceAdmission,
    TerminalSourcePorts,
)
from navpy.modules.nav.terminal_publication_admission import (
    TerminalPublicationAdmission,
    TerminalPublicationAdmissionPorts,
)
from navpy.modules.nav.terminal_source_session import TerminalSourceSession
from navpy.modules.nav.vehicle_navigation import (
    ApproachCommandPorts,
    LoiterRadiusLease,
    VehicleNavigationCommands,
)
from navpy.modules.vehicle.vehicle_interface import IVehicle
from navpy.modules.vision.detection_coordination import DetectionCoordination


ApproachCalculator = Callable[..., ApproachPlan]


def compose_state_ownership(
    scheduler_cadence: Optional[SchedulerCadence],
) -> NavStateOwnership:
    return NavStateOwnership(
        clock=NavClock(scheduler_cadence),
        phase=NavPhaseState(),
        detections=DetectionSnapshot(),
        navigation_task=NavigationTaskState(),
        navigation_failures=NavigationFailureLatch(),
        terminal=TerminalNavState(),
        geo_hold=GeoHoldState(),
        confirm=ConfirmGateState(),
        overrides=ConfirmOverrideInbox(),
    )


def compose_target_mission_ownership(
    vehicle: IVehicle,
    detection: DetectionCoordination,
    args: NavArgs,
    logger: ILogger,
    state: NavStateOwnership,
) -> TargetMissionOwnership:
    mission = MissionCatalog(vehicle)
    mission.refresh()
    pass_tracker = LegacyPassTracker(
        close_distance_m=TARGET_CLOSE_DIST,
        distance_epsilon_m=DIST_EPS,
        increase_samples=DIST_INC_MAX,
        behind_bearing_deg=PASSED_TARGET_BEHIND_MIN_DEG,
    )
    retry_state = TargetRetryState(
        wall_clock_s=lambda: state.clock.wall_s(),
        cooldown_s=TARGET_REACQUIRE_COOLDOWN_SEC,
        max_reask_attempts=CONFIRM_REASK_MAX_ATTEMPTS,
    )
    confirmation_manager = ConfirmationManager(
        vehicle.source_system,
        args,
        logger,
        is_simulation=detection.simulation.is_simulation,
    )
    return TargetMissionOwnership(
        mission=mission,
        pass_tracker=pass_tracker,
        retry_state=retry_state,
        confirmation_manager=confirmation_manager,
        source_session=TerminalSourceSession(detection.events),
    )


def compose_vehicle_approach_ownership(
    vehicle: IVehicle,
    detection: DetectionCoordination,
    args: NavArgs,
    logger: ILogger,
    profile: dict,
    approach_kind: ApproachKind,
    state: NavStateOwnership,
    navigation: NavCapabilities,
    calculate_approach: ApproachCalculator,
) -> VehicleApproachOwnership:
    planner = ApproachPlanner(
        vehicle,
        lambda: detection.mounts.mounts,
        lambda: navigation.terminal.is_active,
        profile,
        logger,
        calculate_approach,
    )
    loiter_radius = LoiterRadiusLease(vehicle, logger)
    commands = VehicleNavigationCommands(
        vehicle=vehicle,
        commands=ApproachCommandPorts(
            goto_target=navigation.vehicle_commands.peer_target,
            loiter_target=navigation.vehicle_commands.peer_target_loiter,
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
    target: TargetMissionOwnership,
    navigation: NavCapabilities,
) -> DetectionReviewOwnership:
    network = NavNetworkRuntime(
        vehicle,
        detection.simulation,
        navigation.legacy_targets.ground_location,
        target.confirmation_manager,
        state.overrides,
        logger,
    )
    publication_admission = TerminalPublicationAdmission(
        TerminalPublicationAdmissionPorts(
            clear_discontinuity=(
                navigation.terminal.clear_source_discontinuity
            ),
            mark_failed=state.navigation_failures.mark_failed,
            logger=logger,
        )
    )
    source = TerminalSourceAdmission(
        TerminalSourcePorts(
            source_session=target.source_session,
            active_target=lambda: target.confirmation_manager.active_target,
            event_inbox=state.detections,
            detections=state.detections.targets,
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
    target.confirmation_manager.set_target_freshness_check(
        freshness.is_target_fresh_for_confirm
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
        retry=TargetRetryPolicy(target.retry_state, target.confirmation_manager),
        status=status,
        debug=debug,
        blocked=blocked,
        timing=ConfirmationTimingPolicy(
            state.confirm,
            state.geo_hold,
            args,
            detection.zoom,
            target.confirmation_manager,
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
    "compose_target_mission_ownership",
    "compose_vehicle_approach_ownership",
]
