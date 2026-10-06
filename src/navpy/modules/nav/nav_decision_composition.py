"""Compose recovery and ordered navigation decision workflows."""

from __future__ import annotations

from navpy.args.nav_args import NavArgs
from navpy.logger.cache_logger import ILogger
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
from navpy.modules.nav.navigation_decision import (
    NavDecisionPorts,
    NavStateDecision,
    InactiveGuardPorts,
    InactiveNavigationGuard,
    NavigationDecision,
    NavigationDecisionPorts,
)
from navpy.modules.nav.recovery import RecoveryAction, RecoveryPorts
from navpy.modules.vehicle.vehicle_interface import IVehicle
from navpy.modules.vision.detection_coordination import DetectionCoordination


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
            set_mode=vehicle.set_mode,
            restart_mission=vehicle.restart_mission,
            set_attitude=vehicle.set_attitude,
            max_pitch_deg=lambda: vehicle.max_pitch,
        ),
        state.phase,
        args,
        observation.status,
        logger,
    )
    inactive = InactiveNavigationGuard(
        InactiveGuardPorts(
            vehicle_armed=lambda: vehicle.is_armed,
            clear_navigation_task=confirmation.reset.reset.clear,
        ),
        state.phase,
        state.navigation_task,
        state.confirm,
        poi.mission,
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
        poi.confirmation_manager,
        navigation_workflows.mission_navigation.mission_pass,
        observation.status,
        logger,
    )
    decision = NavigationDecision(
        NavigationDecisionPorts(
            vehicle_mode=lambda: vehicle.get_mode,
            next_waypoint=lambda: vehicle.mission_items_next,
            clock_s=lambda: state.clock.decision_s(),
        ),
        state.phase,
        inactive,
        recovery,
        nav,
        confirmation.poi_status,
        args,
        poi.mission,
    )
    return DecisionWorkflows(recovery, decision)


__all__ = ["compose_decision_workflows"]
