"""Mission, routing, heartbeat, and lifecycle protocol composition."""

from __future__ import annotations

from navpy.exception_groups import BaseExceptionGroup, ExceptionGroup

from navpy.modules.vehicle.heartbeat_runtime import HeartbeatRuntime
from navpy.modules.vehicle.inbound_router import InboundMessageRouter
from navpy.modules.vehicle.mav_mission import MavMission
from navpy.modules.vehicle.parameter_snapshot import ParameterSnapshotClient
from navpy.modules.vehicle.vehicle_build_types import (
    VehicleBuildRequest,
    VehicleFoundation,
    VehicleProtocolParts,
)
from navpy.modules.vehicle.vehicle_lifecycle import VehicleLifecycle


def _raise_protocol_rollback(
    primary: BaseException,
    cleanup: BaseException,
) -> None:
    errors = [primary, cleanup]
    if all(isinstance(error, Exception) for error in errors):
        raise ExceptionGroup("vehicle protocol build and rollback failed", errors)
    raise BaseExceptionGroup("vehicle protocol build and rollback failed", errors)


def build_protocol_parts(
    request: VehicleBuildRequest,
    foundation: VehicleFoundation,
) -> VehicleProtocolParts:
    mission = MavMission(
        request.link.target_system,
        foundation.link.transport,
        foundation.link.logger_ref,
        foundation.state.messages,
        foundation.state.mission_inbox,
    )
    router = InboundMessageRouter(
        foundation.link.identity,
        foundation.state.messages,
        foundation.state.callbacks,
        foundation.state.parameter_repository,
        foundation.state.heartbeat_state,
        foundation.state.packet_loss,
        foundation.state.mission_inbox,
        foundation.link.logger_ref,
    )
    heartbeat = HeartbeatRuntime(
        foundation.link.transport,
        foundation.link.identity,
        foundation.state.heartbeat_state,
        foundation.link.logger_ref,
        request.startup.heartbeat_hz,
    )
    snapshot = ParameterSnapshotClient(
        foundation.link.transport,
        foundation.link.identity,
        foundation.state.callbacks,
    )
    try:
        lifecycle = VehicleLifecycle(
            foundation.link.active_bus,
            foundation.link.bus_lease,
            request.link.target_system,
            heartbeat,
            foundation.state.heartbeat_state,
            foundation.link.logger_ref,
            closers=(snapshot,),
        )
    except BaseException as primary:
        try:
            snapshot.close()
        except BaseException as cleanup:
            _raise_protocol_rollback(primary, cleanup)
        raise
    return VehicleProtocolParts(
        mission=mission,
        router=router,
        snapshot=snapshot,
        heartbeat_runtime=heartbeat,
        lifecycle=lifecycle,
    )


__all__ = ["build_protocol_parts"]
