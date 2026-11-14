"""Compose guidance admission, event transport, and reset ownership."""

from __future__ import annotations

from dataclasses import dataclass

from navpy.core.kernel.config.nav_args import NavArgs
from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.skills.acquire.approach_strategy import ApproachKind
from navpy.app.flow.nav_composition_types import (
    ConfirmationAdmissionWorkflows,
    DetectionReviewOwnership,
    NavigationTaskWorkflows,
    NavCapabilities,
    NavStateOwnership,
    ResetWorkflows,
    PoiMissionOwnership,
    VehicleApproachOwnership,
)
from navpy.app.flow.nav_confirmation_admission_composition import (
    compose_confirmation_admission_workflows,
)
from navpy.core.swarm.nav_network import NavPeerSubmission
from navpy.core.flow.nav_constants import (
    CONFIRM_FRESH_DETECTION_MAX_AGE_S,
)
from navpy.app.flow.nav_decision_composition import request_guided_owned
from navpy.app.flow.nav_reset_composition import compose_reset_workflows
from navpy.core.flow.nav_state import NavState
from navpy.core.approach.guidance_command_dispatch import (
    GuidanceCommandDispatch,
    GuidanceCommandPorts,
)
from navpy.core.approach.guidance_detection_event_pump import (
    GuidanceDetectionEventPump,
    GuidanceDetectionEventPumpPorts,
    GuidancePumpFailure,
)
from navpy.core.approach.guidance_nav_workflow import (
    GuidanceNavPorts,
    GuidanceNavWorkflow,
)
from navpy.core.approach.guidance_record_commit import (
    GuidanceRecordCommit,
    GuidanceRecordDeferral,
    GuidanceRecordPorts,
)
from navpy.core.approach.guidance_source_event_handler import (
    GuidanceSourceEventHandler,
    GuidanceSourceEventHandlerPorts,
)
from navpy.core.approach.guidance_source_reset_fence import (
    GuidanceSourceResetFence,
)
from navpy.core.ports.flight_mode import FlightMode
from navpy.core.ports.vehicle_contract import IVehicle
from navpy.core.perception.detection_coordination import DetectionCoordination
from navpy.core.ports.detect_request import DetectRequest
from navpy.core.approach.vision_nav import nav_fail_reason as fail_reason


@dataclass(frozen=True)
class GuidanceWorkflowComposition:
    admission: ConfirmationAdmissionWorkflows
    nav: GuidanceNavWorkflow
    reset: ResetWorkflows


def compose_guidance_workflows(
    vehicle: IVehicle,
    detection: DetectionCoordination,
    args: NavArgs,
    logger: ILogger,
    approach_kind: ApproachKind,
    state: NavStateOwnership,
    poi: PoiMissionOwnership,
    approach: VehicleApproachOwnership,
    observation: DetectionReviewOwnership,
    navigation_task: NavigationTaskWorkflows,
    navigation: NavCapabilities,
) -> GuidanceWorkflowComposition:
    admission = compose_confirmation_admission_workflows(
        vehicle,
        detection,
        args,
        logger,
        approach_kind,
        state,
        poi,
        approach,
        observation,
        navigation_task,
        navigation,
    )
    nav = compose_guidance_nav(
        vehicle,
        logger,
        state,
        poi,
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
        poi,
        approach,
        observation,
        navigation,
        nav.close_source_admission,
    )
    return GuidanceWorkflowComposition(admission, nav, reset)


def compose_guidance_nav(
    vehicle: IVehicle,
    logger: ILogger,
    state: NavStateOwnership,
    poi: PoiMissionOwnership,
    approach: VehicleApproachOwnership,
    observation: DetectionReviewOwnership,
    navigation_task: NavigationTaskWorkflows,
    navigation: NavCapabilities,
    detection: DetectionCoordination,
    deadline: GuidanceRecordDeferral,
) -> GuidanceNavWorkflow:
    commands = GuidanceCommandDispatch(
        GuidanceCommandPorts(
            nav=navigation.nav,
            mark_failed=state.navigation_failures.mark_failed,
            logger=logger,
        ),
        observation.source,
    )
    record = GuidanceRecordCommit(
        GuidanceRecordPorts(
            uses_vision_nav=lambda: navigation.guidance.is_active,
            guidance_recorded=lambda: state.guidance.confirmed_recorded,
            mark_guidance_recorded=state.guidance.mark_recorded,
            record_detection=navigation.guidance.record_confirmed_detection,
            debug=observation.debug.log,
            mark_no_detection=commands.mark_no_detection,
        ),
        observation.source,
        deadline,
    )
    source_reset = GuidanceSourceResetFence(
        observation.source.clear_pending_source_events,
        navigation.guidance.invalidate_pending_source_work,
    )
    handler = GuidanceSourceEventHandler(
        GuidanceSourceEventHandlerPorts(
            active_poi=lambda: poi.confirmation_manager.active_poi,
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
    pump = GuidanceDetectionEventPump(
        GuidanceDetectionEventPumpPorts(
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
    return GuidanceNavWorkflow(
        GuidanceNavPorts(
            vehicle_mode=lambda: vehicle.get_mode,
            request_guided=lambda: request_guided_owned(state, approach),
            mark_guided_session=lambda: _mark_guided_session(state),
            active_poi=lambda: poi.confirmation_manager.active_poi,
            vision_nav_active=lambda: navigation.guidance.is_active,
            command_liveness_failed=(
                navigation.guidance.consume_command_liveness_failure
            ),
            dock_missed=navigation.guidance.consume_missed_dock,
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
    state.navigation_task.guidance_navigation_active = True


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
    failure: GuidancePumpFailure,
    state: NavStateOwnership,
    logger: ILogger,
) -> None:
    state.navigation_failures.mark_failed(fail_reason.SOURCE_EVENT_PUMP)
    try:
        logger.error(
            f"NAV_FAIL: source_event_pump {failure.reason}",
            failure.error,
        )
    except Exception:  # noqa: BLE001 - diagnostic sink is non-authoritative
        return


__all__ = [
    "GuidanceWorkflowComposition",
    "compose_guidance_nav",
    "compose_guidance_workflows",
]
