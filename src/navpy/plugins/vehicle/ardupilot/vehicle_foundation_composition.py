"""Foundation construction for one MAVLink vehicle."""

from __future__ import annotations

from collections.abc import Mapping
from types import MappingProxyType

from navpy.core.ports.pose_stream_rate import ATTITUDE_HISTORY_COUNT
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

# Samples kept per response type for `MessageStore.wait_after`: enough that a
# burst of unrelated responses (a full parameter download) cannot evict a
# waiter's answer before it looks; an eviction past a cursor is reported as
# an overrun, never a silent timeout.
RESPONSE_HISTORY_DEPTH = 1024

# The only MAVLink types read from history; every other type keeps only its
# latest sample (docs/design/message_history.md). Consumers:
# PARAM_VALUE: parameter_reader, parameter_writer; MISSION_COUNT and
# MISSION_ITEM(_INT): mission_download; HOME_POSITION and TERRAIN_REPORT:
# position_service; ATTITUDE: pose_telemetry.attitude_history for the frame
# association (real_frame_association).
MESSAGE_HISTORY_POLICY: Mapping[str, int] = MappingProxyType({
    "PARAM_VALUE": RESPONSE_HISTORY_DEPTH,
    "MISSION_COUNT": RESPONSE_HISTORY_DEPTH,
    "MISSION_ITEM": RESPONSE_HISTORY_DEPTH,
    "MISSION_ITEM_INT": RESPONSE_HISTORY_DEPTH,
    "HOME_POSITION": RESPONSE_HISTORY_DEPTH,
    "TERRAIN_REPORT": RESPONSE_HISTORY_DEPTH,
    "ATTITUDE": ATTITUDE_HISTORY_COUNT,
})


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
            messages=MessageStore(MESSAGE_HISTORY_POLICY),
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


__all__ = [
    "MESSAGE_HISTORY_POLICY",
    "RESPONSE_HISTORY_DEPTH",
    "build_vehicle_foundation",
]
