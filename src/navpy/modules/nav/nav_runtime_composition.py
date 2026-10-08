"""Compose final-approach NAV dispatch and the navigation application runtime."""

from __future__ import annotations

from typing import Optional

from navpy.args.nav_args import NavArgs
from navpy.logger.cache_logger import ILogger
from navpy.modules.nav.detect_action import DetectAction, DetectPointingPorts
from navpy.modules.nav.nav_entry import NavEntry, NavEntryPorts
from navpy.modules.nav.nav_exit_cleanup import (
    NavExitCleanup,
    NavExitCleanupPorts,
)
from navpy.modules.nav.nav_oneshot_completion import (
    OneShotCompletion,
    OneShotCompletionPorts,
)
from navpy.modules.nav.nav_snap_reporter import NavSnapReporter
from navpy.modules.nav.peer_geo_tick import PeerGeoTick
from navpy.modules.nav.nav_application import NavApplication, NavShutdown
from navpy.modules.nav.nav_composition_types import (
    ConfirmationWorkflows,
    DecisionWorkflows,
    DetectionReviewOwnership,
    NavigationTaskWorkflows,
    NavCapabilities,
    NavStateOwnership,
    PoiMissionOwnership,
    VehicleApproachOwnership,
)
from navpy.modules.nav.nav_state import NavState
from navpy.modules.nav.nav_task_availability import is_approaching
from navpy.modules.nav.navigation_loop import (
    DetectionSensor,
    NavigationCycle,
    NavigationLoop,
    StateActionDispatcher,
)
from navpy.modules.nav.nav_transition import NavTransition
from navpy.modules.nav.navigation_transition_handler import (
    NavigationTransitionHandler,
    TransitionPorts,
)
from navpy.modules.nav.final_approach_nav_workflow import FinalApproachNavWorkflow
from navpy.modules.vehicle.vehicle_interface import IVehicle
from navpy.modules.vision.detection_coordination import DetectionCoordination


def compose_nav_application(
    vehicle: IVehicle,
    detection: DetectionCoordination,
    args: NavArgs,
    logger: ILogger,
    state: NavStateOwnership,
    poi: PoiMissionOwnership,
    approach: VehicleApproachOwnership,
    observation: DetectionReviewOwnership,
    navigation_workflows: NavigationTaskWorkflows,
    confirmation: ConfirmationWorkflows,
    decision: DecisionWorkflows,
    navigation: NavCapabilities,
    final_approach_nav: FinalApproachNavWorkflow,
) -> NavApplication:
    loop: Optional[NavigationLoop] = None
    nav_transition = NavTransition(
        NavEntry(
            NavEntryPorts(
                request_guided=lambda: approach.commands.request_guided(),
                navigation_init=navigation.init,
                final_approach_active=lambda: navigation.final_approach.is_active,
            ),
            state.navigation_task,
            observation.zoom,
            logger,
        ),
        NavExitCleanup(
            NavExitCleanupPorts(
                close_final_approach_source=final_approach_nav.close_source_admission,
                navigation_reset=navigation.reset,
                detector_stop=detection.tracking_commands.stop_tracking,
            ),
            approach.speedup,
            logger,
        ),
        NavSnapReporter(navigation.algorithm_info, logger),
        OneShotCompletion(
            OneShotCompletionPorts(
                is_simulated_autopilot=vehicle.is_simulated_autopilot,
                disarm=vehicle.disarm,
                is_stopping=(
                    lambda: loop is not None and loop.is_stopping()
                ),
            ),
            confirmation.reset.reset,
            logger,
        ),
        state.navigation_task,
        lambda: args.is_oneshot,
    )
    transitions = NavigationTransitionHandler(
        TransitionPorts(
            decision_clock_s=lambda: state.clock.decision_s(),
            restart_auto_mission=decision.recovery.restart_auto_mission,
            enter_recovery=decision.recovery.enter,
            start_task_actor=observation.network.start_task_actor,
        ),
        state.confirm,
        nav_transition,
        confirmation.reset.reset,
        logger,
    )
    detect_action = DetectAction(
        state.detections,
        navigation_workflows.selector,
        navigation_workflows.navigation_task_action,
        navigation_workflows.peer_notifier,
        poi.confirmation_manager,
        PeerGeoTick(
            state.navigation_task,
            state.geo_hold,
            DetectPointingPorts(
                is_detection_armed=(
                    lambda: detection.tracking_status.is_detection_armed
                ),
                prepare_geo_acquisition=(
                    detection.geo_pointing.prepare_geo_acquisition
                ),
                update_geo=detection.geo_pointing.update_geo,
            ),
            navigation_workflows.peer_geo_acquisition,
            lambda: navigation.final_approach.is_active,
            lambda: vehicle.location(False),
            lambda: vehicle.attitude,
        ),
    )
    actions = StateActionDispatcher(
        state.phase,
        transitions,
        {
            NavState.ONHOLD: navigation.pause,
            NavState.DETECT: detect_action.act,
            NavState.CONFIRM: confirmation.confirmation_action.act,
            NavState.NAV: final_approach_nav.act_nav,
            NavState.RESET: navigation.pause,
            NavState.RECOVERY: decision.recovery.act,
        },
        lambda committed: observation.network.publish_approaching(
            is_approaching(committed)
        ),
    )
    sensor = DetectionSensor(
        detection.snapshot,
        detection.events,
        state.detections,
        state.phase,
    )
    cycle = NavigationCycle(
        lambda: vehicle.is_armed,
        state.detections,
        sensor,
        decision.decision,
        actions,
    )
    loop = NavigationLoop(cycle, state.clock, logger)
    shutdown = NavShutdown(
        final_approach_nav.close_source_admission,
        navigation.reset,
        confirmation.reset.reset,
        observation.network,
        logger,
    )
    return NavApplication(loop, observation.network, shutdown, state.overrides)


__all__ = ["compose_nav_application"]
