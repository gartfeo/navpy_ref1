"""Foundation construction for one MAVLink vehicle."""

from __future__ import annotations

from navpy.modules.vehicle.link_state import HeartbeatState, PacketLossTracker
from navpy.modules.vehicle.logger_ref import LoggerRef
from navpy.modules.vehicle.mav_bus import MavBus, MavBusLease
from navpy.modules.vehicle.mav_transport import MavTransport
from navpy.modules.vehicle.message_store import MessageStore
from navpy.modules.vehicle.message_subscriptions import CallbackRegistry
from navpy.modules.vehicle.mission_inbox import MissionInbox
from navpy.modules.vehicle.parameter_repository import ParameterRepository
from navpy.modules.vehicle.pose_telemetry import CommandDiagnostics
from navpy.modules.vehicle.vehicle_build_types import (
    VehicleBuildRequest,
    VehicleFoundation,
    VehicleLinkContext,
    VehicleStateStores,
)
from navpy.modules.vehicle.vehicle_identity import VehicleIdentity


def build_vehicle_foundation(
    request: VehicleBuildRequest,
    active_bus: MavBus,
    bus_lease: MavBusLease,
) -> VehicleFoundation:
    link = request.link
    identity = VehicleIdentity(
        target_system=link.target_system,
        source_system=link.target_system,
        source_component=link.mav_comp_id,
        mav_type=link.mav_type,
    )
    logger_ref = LoggerRef(request.logger)
    return VehicleFoundation(
        link=VehicleLinkContext(
            identity=identity,
            logger_ref=logger_ref,
            active_bus=active_bus,
            bus_lease=bus_lease,
            # From the BUS actually in use, not the request: a supplied bus
            # may have been opened on a different endpoint than the one the
            # request named, and the live-link identity must describe the
            # connection frames really arrive on.
            transport=MavTransport(active_bus.conn, active_bus.send_lock),
            device=active_bus.device,
        ),
        state=VehicleStateStores(
            messages=MessageStore(),
            callbacks=CallbackRegistry(logger_ref),
            parameter_repository=ParameterRepository(),
            heartbeat_state=HeartbeatState(
                link.target_system,
                request.startup.heartbeat_timeout,
            ),
            packet_loss=PacketLossTracker(),
            mission_inbox=MissionInbox(),
            diagnostics=CommandDiagnostics(),
        ),
    )


__all__ = ["build_vehicle_foundation"]
