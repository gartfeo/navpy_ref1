"""Composition of navigation task reset and AUTO-resume workflows."""

from __future__ import annotations

from collections.abc import Callable

from navpy.core.kernel.config.nav_args import NavArgs
from navpy.products.delivery.mission_catalog import MissionCatalog
from navpy.core.skills.acquire.approach_planner import ApproachPlanner
from navpy.core.flow.navigation_task_reset import (
    AutoMissionResume,
    AutoMissionResumePorts,
    NavigationTaskResetTransaction,
    NavigationTaskResourcePorts,
    NavigationTaskResourceReset,
    NavigationTaskStateReset,
)
from navpy.app.flow.nav_composition_types import (
    DetectionReviewOwnership,
    NavCapabilities,
    NavStateOwnership,
    ResetWorkflows,
    PoiMissionOwnership,
    VehicleApproachOwnership,
)
from navpy.core.ports.vehicle_contract import IVehicle
from navpy.core.perception.detection_coordination import DetectionCoordination
from navpy.plugins.vehicle.ardupilot.mission_metadata import read_mission_metadata
from navpy.products.delivery.flow import interrupted_leg


def compose_reset_workflows(
    vehicle: IVehicle,
    detection: DetectionCoordination,
    args: NavArgs,
    state: NavStateOwnership,
    poi: PoiMissionOwnership,
    approach: VehicleApproachOwnership,
    observation: DetectionReviewOwnership,
    navigation: NavCapabilities,
    close_final_approach_source: Callable[[], None],
) -> ResetWorkflows:
    state_reset = NavigationTaskStateReset(
        state.navigation_task,
        state.navigation_failures,
        state.geo_hold,
        state.confirm,
        state.detections,
        lambda: poi.mission.clear_navigation_task(),
        poi.retry_state,
        poi.pass_tracker,
    )
    resource_reset = NavigationTaskResourceReset(
        NavigationTaskResourcePorts(
            reset_peer_dispatch=observation.network.reset_peer_dispatch,
            reset_task_actor=observation.network.reset_task_actor,
            refresh_mission=lambda: refresh_mission_metadata(
                vehicle,
                poi.mission,
                approach.planner,
            ),
        ),
        detection.tracking_commands,
        detection.geo_pointing,
        detection.reset,
        poi.confirmation_manager,
        approach.loiter_radius,
        approach.speedup,
        args,
    )
    reset = NavigationTaskResetTransaction(
        state_reset,
        resource_reset,
        close_final_approach_source,
    )
    resume_auto = AutoMissionResume(
        AutoMissionResumePorts(
            close_final_approach_source=close_final_approach_source,
            pause_navigation=navigation.pause,
            clear_selected_poi=observation.network.clear_selected_poi,
            clear_flow_task=lambda: poi.mission.clear_navigation_task(),
            resume_waypoint=interrupted_leg,
        ),
        vehicle,
        detection.geo_pointing,
        state.navigation_task,
        state.geo_hold,
        state.confirm,
        state.detections,
        poi.pass_tracker,
        approach.loiter_radius,
    )
    return ResetWorkflows(reset, resume_auto)


def refresh_mission_metadata(
    vehicle: IVehicle,
    mission: MissionCatalog,
    planner: ApproachPlanner,
) -> None:
    metadata = read_mission_metadata(vehicle)
    mission.update(metadata.default_delivery_hub, metadata.default_delivery_hub_type)
    planner.scan_altitude_rel = metadata.scan_altitude_rel


__all__ = ["compose_reset_workflows", "refresh_mission_metadata"]
