"""Compose terminal admission, event transport, and reset ownership."""

from __future__ import annotations

from dataclasses import dataclass

from navpy.args.nav_args import NavArgs
from navpy.logger.cache_logger import ILogger
from navpy.modules.navigation.approach_strategy import ApproachKind
from navpy.modules.nav.nav_composition_types import (
    ConfirmationAdmissionWorkflows,
    DetectionReviewOwnership,
    NavigationTaskWorkflows,
    NavCapabilities,
    NavStateOwnership,
    ResetWorkflows,
    TargetMissionOwnership,
    VehicleApproachOwnership,
)
from navpy.modules.nav.nav_confirmation_admission_composition import (
    compose_confirmation_admission_workflows,
)
from navpy.modules.nav.nav_network import NavPeerSubmission
from navpy.modules.nav.nav_constants import (
    CONFIRM_FRESH_DETECTION_MAX_AGE_S,
)
from navpy.modules.nav.nav_reset_composition import compose_reset_workflows
from navpy.modules.nav.nav_state import NavState
from navpy.modules.nav.terminal_command_dispatch import (
    TerminalCommandDispatch,
    TerminalCommandPorts,
)
from navpy.modules.nav.terminal_detection_event_pump import (
    TerminalDetectionEventPump,
    TerminalDetectionEventPumpPorts,
    TerminalPumpFailure,
)
from navpy.modules.nav.terminal_nav_workflow import (
    TerminalNavPorts,
    TerminalNavWorkflow,
)
from navpy.modules.nav.terminal_record_commit import (
    TerminalRecordCommit,
    TerminalRecordDeferral,
    TerminalRecordPorts,
)
from navpy.modules.nav.terminal_source_event_handler import (
    TerminalSourceEventHandler,
    TerminalSourceEventHandlerPorts,
)
from navpy.modules.nav.terminal_source_reset_fence import (
    TerminalSourceResetFence,
)
from navpy.modules.vehicle.flight_mode import FlightMode
from navpy.modules.vehicle.vehicle_interface import IVehicle
from navpy.modules.vision.detection_coordination import DetectionCoordination
from navpy.modules.vision.models.detect_request import DetectRequest


@dataclass(frozen=True)
class TerminalWorkflowComposition:
    admission: ConfirmationAdmissionWorkflows
    nav: TerminalNavWorkflow
    reset: ResetWorkflows


def compose_terminal_workflows(
    vehicle: IVehicle,
    detection: DetectionCoordination,
    args: NavArgs,
    logger: ILogger,
    approach_kind: ApproachKind,
    state: NavStateOwnership,
    target: TargetMissionOwnership,
    approach: VehicleApproachOwnership,
    observation: DetectionReviewOwnership,
    navigation_task: NavigationTaskWorkflows,
    navigation: NavCapabilities,
) -> TerminalWorkflowComposition:
    admission = compose_confirmation_admission_workflows(
        vehicle,
        detection,
        args,
        logger,
        approach_kind,
        state,
        target,
        approach,
        observation,
        navigation_task,
        navigation,
    )
    nav = compose_terminal_nav(
        vehicle,
        logger,
        state,
        target,
        approach,
        observation,
        navigation_task,
        navigation,
        detection,
        admission.deadline,
    )
    reset = compose_reset_workflows(
        vehicle,
        detection,
        args,
        state,
        target,
        approach,
        observation,
        navigation,
        nav.close_source_admission,
    )
    return TerminalWorkflowComposition(admission, nav, reset)


def compose_terminal_nav(
    vehicle: IVehicle,
    logger: ILogger,
    state: NavStateOwnership,
    target: TargetMissionOwnership,
    approach: VehicleApproachOwnership,
    observation: DetectionReviewOwnership,
    navigation_task: NavigationTaskWorkflows,
    navigation: NavCapabilities,
    detection: DetectionCoordination,
    deadline: TerminalRecordDeferral,
) -> TerminalNavWorkflow:
    commands = TerminalCommandDispatch(
        TerminalCommandPorts(
            nav=navigation.nav,
            mark_failed=state.navigation_failures.mark_failed,
            logger=logger,
        ),
        observation.source,
    )
    record = TerminalRecordCommit(
        TerminalRecordPorts(
            uses_vision_nav=lambda: navigation.terminal.is_active,
            terminal_recorded=lambda: state.terminal.confirmed_recorded,
            mark_terminal_recorded=state.terminal.mark_recorded,
            record_detection=navigation.terminal.record_confirmed_detection,
            debug=observation.debug.log,
            mark_no_detection=commands.mark_no_detection,
        ),
        observation.source,
        deadline,
    )
    source_reset = TerminalSourceResetFence(
        observation.source.clear_pending_source_events,
        navigation.terminal.invalidate_pending_source_work,
    )
    handler = TerminalSourceEventHandler(
        TerminalSourceEventHandlerPorts(
            active_target=lambda: target.confirmation_manager.active_target,
            session_active=lambda: _source_session_active(
                vehicle,
                state,
            ),
            reset_source=source_reset.reset,
            admission=observation.publication_admission,
            mark_failed=state.navigation_failures.mark_failed,
            logger=logger,
            commands=commands,
        )
    )
    pump = TerminalDetectionEventPump(
        TerminalDetectionEventPumpPorts(
            open_lease=lambda reset_handler: detection.events.open_detection_event_lease(
                DetectRequest(),
                reset_handler,
            ),
            report_failure=lambda failure: _report_pump_failure(
                failure,
                state,
                logger,
            ),
            wall_s=state.clock.wall_s,
            receipt_max_wall_age_s=lambda: state.clock.scheduler_wall_period(
                CONFIRM_FRESH_DETECTION_MAX_AGE_S
            ),
        ),
        handler,
    )
    navigation.bind_source_dispatch(pump.dispatch_available)
    peers = NavPeerSubmission(
        navigation_task.selector,
        state.detections,
        observation.network.submit_nav_peers,
    )
    return TerminalNavWorkflow(
        TerminalNavPorts(
            vehicle_mode=lambda: vehicle.get_mode,
            request_guided=lambda: approach.commands.request_guided(),
            mark_guided_session=lambda: _mark_guided_session(state),
            active_target=lambda: target.confirmation_manager.active_target,
            vision_nav_active=lambda: navigation.terminal.is_active,
            command_liveness_failed=(
                navigation.terminal.consume_command_liveness_failure
            ),
        ),
        observation.source,
        commands,
        record,
        peers,
        pump,
        source_reset.bootstrap,
    )


def _mark_guided_session(state: NavStateOwnership) -> None:
    state.navigation_task.nav_mode_observed = True
    state.navigation_task.terminal_navigation_active = True


def _source_session_active(
    vehicle: IVehicle,
    state: NavStateOwnership,
) -> bool:
    in_nav = state.phase.is_current(NavState.NAV)
    return (
        in_nav
        and vehicle.get_mode == FlightMode.GUIDED
        and vehicle.is_armed is True
    )


def _report_pump_failure(
    failure: TerminalPumpFailure,
    state: NavStateOwnership,
    logger: ILogger,
) -> None:
    state.navigation_failures.mark_failed()
    try:
        logger.error(
            f"NAV_FAIL: source_event_pump {failure.reason}",
            failure.error,
        )
    except Exception:  # noqa: BLE001 - diagnostic sink is non-authoritative
        return


__all__ = [
    "TerminalWorkflowComposition",
    "compose_terminal_nav",
    "compose_terminal_workflows",
]
