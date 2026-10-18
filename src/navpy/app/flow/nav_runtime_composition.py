"""Compose final-approach NAV dispatch and the navigation application runtime."""

from __future__ import annotations

from typing import Optional

from navpy.core.kernel.config.nav_args import NavArgs
from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.skills.search.detect_action import DetectAction
from navpy.core.skills.acquire.peer_geo_tick import DetectPointingPorts
from navpy.core.approach.nav_entry import NavEntry, NavEntryPorts
from navpy.core.approach.nav_exit_cleanup import (
    NavExitCleanup,
    NavExitCleanupPorts,
)
from navpy.core.flow.nav_oneshot_completion import (
    OneShotCompletion,
    OneShotCompletionPorts,
)
from navpy.core.approach.nav_snap_reporter import NavSnapReporter
from navpy.core.skills.acquire.peer_geo_tick import PeerGeoTick
from navpy.core.flow.nav_application import NavApplication, NavShutdown
from navpy.app.flow.nav_composition_types import (
    ConfirmationWorkflows,
    DecisionWorkflows,
    DetectionReviewOwnership,
    NavigationTaskWorkflows,
    NavCapabilities,
    NavStateOwnership,
    PoiMissionOwnership,
    VehicleApproachOwnership,
)
from navpy.core.flow.nav_state import NavState
from navpy.core.flow.nav_task_availability import is_approaching
from navpy.core.flow.navigation_loop import (
    DetectionSensor,
    NavigationCycle,
    NavigationLoop,
    StateActionDispatcher,
)
from navpy.core.flow.nav_transition import NavTransition
from navpy.core.flow.navigation_transition_handler import (
    NavigationTransitionHandler,
    TransitionPorts,
)
from navpy.core.approach.final_approach_nav_workflow import FinalApproachNavWorkflow
from navpy.core.approach.dock_contact_episode import FinalApproachDockContact
from navpy.core.ports.dock_contact import DockContactSensor
from navpy.core.ports.vehicle_contract import IVehicle
from navpy.core.perception.detection_coordination import DetectionCoordination


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
    dock_contact_sensor: Optional[DockContactSensor] = None,
) -> NavApplication:
    loop: Optional[NavigationLoop] = None
    dock_contact = FinalApproachDockContact(
        dock_contact_sensor,
        lambda: vehicle.target_system,
        navigation.final_approach.begin_dock_episode,
        navigation.final_approach.latch_docked,
        logger,
        source_now_s=lambda: _attitude_boot_s(vehicle),
        end_episode=navigation.final_approach.end_dock_episode,
    )
    nav_transition = NavTransition(
        NavEntry(
            NavEntryPorts(
                request_guided=lambda: approach.commands.request_guided(),
                navigation_init=navigation.init,
                final_approach_active=lambda: navigation.final_approach.is_active,
                arm_dock_contact=lambda: dock_contact.arm(
                    poi.confirmation_manager.active_poi
                ),
                stop_geo_tracking=detection.geo_pointing.stop_geo_tracking,
                is_geo_armed=lambda: detection.geo_pointing.is_geo_armed,
            ),
            state.navigation_task,
            observation.zoom,
            logger,
        ),
        NavExitCleanup(
            NavExitCleanupPorts(
                disarm_dock_contact=dock_contact.disarm,
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
        logger=logger,
    )
    transitions = NavigationTransitionHandler(
        TransitionPorts(
            decision_clock_s=lambda: state.clock.decision_s(),
            restart_auto_mission=decision.recovery.restart_auto_mission,
            enter_recovery=decision.recovery.enter,
            start_task_actor=observation.network.start_task_actor,
            is_oneshot=lambda: args.is_oneshot,
        ),
        state.confirm,
        nav_transition,
        confirmation.reset.reset,
        logger,
        state.post_dock_climb,
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
            lambda: state.phase.is_current(NavState.NAV),
            lambda: vehicle.location(False),
            lambda: vehicle.attitude,
        ),
        lambda: navigation_workflows.mission_navigation.mission_pass.holds_own_search(
            vehicle.get_mode,
            vehicle.mission_items_next,
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
            NavState.DOCKED: navigation.pause,
        },
        lambda committed: observation.network.publish_participation(
            is_approaching(committed),
            committed is NavState.DOCKED,
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


def _attitude_boot_s(vehicle: IVehicle) -> Optional[float]:
    """Latest ATTITUDE boot time (the observation clock), for evidence rows."""
    sample = getattr(vehicle, "attitude_sample", None)
    value = getattr(sample, "time_boot_s", None)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value)


__all__ = ["compose_nav_application"]
