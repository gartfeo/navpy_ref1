"""Compose recovery and ordered navigation decision workflows."""

from __future__ import annotations

from navpy.core.kernel.config.nav_args import NavArgs
from navpy.core.kernel.logging.cache_logger import ILogger
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
from navpy.core.flow.navigation_decision import (
    NavDecisionPorts,
    NavStateDecision,
    InactiveGuardPorts,
    InactiveNavigationGuard,
    NavigationDecision,
    NavigationDecisionPorts,
)
from navpy.core.flow.recovery import RecoveryAction, RecoveryPorts
from navpy.app.flow.nav_delivery_composition import compose_delivery_flow
from navpy.core.ports.vehicle_contract import IVehicle
from navpy.core.perception.detection_coordination import DetectionCoordination


def compose_decision_workflows(
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
    navigation: NavCapabilities,
) -> DecisionWorkflows:
    recovery = RecoveryAction(
        RecoveryPorts(
            final_approach_active=lambda: navigation.final_approach.is_active,
            detector_stop=detection.tracking_commands.stop_tracking,
            current_relative=lambda: vehicle.location(True),
            pause_navigation=navigation.pause,
            set_attitude=vehicle.set_attitude,
            max_pitch_deg=lambda: vehicle.max_pitch,
            min_alt=lambda: args.min_alt,
            is_oneshot=lambda: args.is_oneshot,
        ),
        state.phase,
        observation.status,
        logger,
        state.post_dock_climb,
    )
    inactive = InactiveNavigationGuard(
        InactiveGuardPorts(
            vehicle_armed=lambda: vehicle.is_armed,
            clear_navigation_task=confirmation.reset.reset.clear,
            flow_task_active=lambda: poi.mission.default_delivery_hub_active,
            peer_task_assigned=lambda: observation.network.selected_poi() is not None,
        ),
        state.phase,
        state.navigation_task,
        state.confirm,
        poi.confirmation_manager,
        approach.loiter_radius,
        navigation_workflows.mission_navigation.mission_pass,
        observation.status,
    )
    nav = NavStateDecision(
        NavDecisionPorts(
            clock_s=lambda: state.clock.decision_s(),
            clear_navigation_task=confirmation.reset.reset.clear,
            request_guided=lambda: approach.commands.request_guided(),
            final_approach_active=lambda: navigation.final_approach.is_active,
        ),
        state.phase,
        state.navigation_task,
        state.navigation_failures,
        navigation_workflows.mission_navigation.mission_pass,
        logger,
    )
    flow = compose_delivery_flow(state, poi, observation, nav, confirmation.poi_status)
    decision = NavigationDecision(
        NavigationDecisionPorts(
            vehicle_mode=lambda: vehicle.get_mode,
            next_waypoint=lambda: vehicle.mission_items_next,
            is_oneshot=lambda: args.is_oneshot,
        ),
        state.phase,
        inactive,
        recovery,
        flow.decide,
    )
    return DecisionWorkflows(recovery, decision)


__all__ = ["compose_decision_workflows"]
