"""Foundation construction for one MAVLink vehicle."""

from __future__ import annotations

from navpy.plugins.vehicle.ardupilot.link_state import HeartbeatState, PacketLossTracker
from navpy.plugins.vehicle.ardupilot.logger_ref import LoggerRef
from navpy.plugins.vehicle.ardupilot.mav_bus import MavBus, MavBusLease
from navpy.plugins.vehicle.ardupilot.mav_transport import MavTransport
from navpy.plugins.vehicle.ardupilot.message_store import MessageStore
from navpy.plugins.vehicle.ardupilot.message_subscriptions import CallbackRegistry
from navpy.plugins.vehicle.ardupilot.mission_inbox import MissionInbox
from navpy.plugins.vehicle.ardupilot.parameter_repository import ParameterRepository
from navpy.plugins.vehicle.ardupilot.pose_telemetry import CommandDiagnostics
from navpy.plugins.vehicle.ardupilot.vehicle_build_types import (
    VehicleBuildRequest,
    VehicleFoundation,
    VehicleLinkContext,
    VehicleStateStores,
)
from navpy.plugins.vehicle.ardupilot.vehicle_identity import VehicleIdentity


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
