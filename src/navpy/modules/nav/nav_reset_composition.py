"""Composition of navigation task reset and AUTO-resume workflows."""

from __future__ import annotations

from collections.abc import Callable

from navpy.args.nav_args import NavArgs
from navpy.modules.nav.navigation_task_reset import (
    AutoMissionResume,
    AutoMissionResumePorts,
    NavigationTaskResetTransaction,
    NavigationTaskResourcePorts,
    NavigationTaskResourceReset,
    NavigationTaskStateReset,
)
from navpy.modules.nav.nav_composition_types import (
    DetectionReviewOwnership,
    NavCapabilities,
    NavStateOwnership,
    ResetWorkflows,
    TargetMissionOwnership,
    VehicleApproachOwnership,
)
from navpy.modules.vehicle.vehicle_interface import IVehicle
from navpy.modules.vision.detection_coordination import DetectionCoordination


def compose_reset_workflows(
    vehicle: IVehicle,
    detection: DetectionCoordination,
    args: NavArgs,
    state: NavStateOwnership,
    target: TargetMissionOwnership,
    approach: VehicleApproachOwnership,
    observation: DetectionReviewOwnership,
    navigation: NavCapabilities,
    close_terminal_source: Callable[[], None],
) -> ResetWorkflows:
    state_reset = NavigationTaskStateReset(
        state.navigation_task,
        state.navigation_failures,
        state.geo_hold,
        state.confirm,
        state.detections,
        target.mission,
        target.retry_state,
        target.pass_tracker,
    )
    resource_reset = NavigationTaskResourceReset(
        NavigationTaskResourcePorts(
            reset_peer_dispatch=observation.network.reset_peer_dispatch,
            reset_task_actor=observation.network.reset_task_actor,
            refresh_mission=target.mission.refresh,
        ),
        detection.tracking_commands,
        detection.geo_pointing,
        detection.reset,
        target.confirmation_manager,
        approach.loiter_radius,
        approach.speedup,
        args,
    )
    reset = NavigationTaskResetTransaction(
        state_reset,
        resource_reset,
        close_terminal_source,
    )
    resume_auto = AutoMissionResume(
        AutoMissionResumePorts(
            close_terminal_source=close_terminal_source,
            pause_navigation=navigation.pause,
            clear_selected_target=observation.network.clear_selected_target,
        ),
        vehicle,
        detection.geo_pointing,
        state.navigation_task,
        state.geo_hold,
        state.confirm,
        state.detections,
        target.mission,
        target.pass_tracker,
        approach.loiter_radius,
    )
    return ResetWorkflows(reset, resume_auto)


__all__ = ["compose_reset_workflows"]
