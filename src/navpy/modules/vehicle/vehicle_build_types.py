"""Immutable records exchanged by the vehicle composition stages."""

from __future__ import annotations

from dataclasses import dataclass

from navpy.logger.cache_logger import ILogger
from navpy.modules.vehicle.attitude_command import AttitudeCommand
from navpy.modules.vehicle.flight_telemetry import FlightTelemetry
from navpy.modules.vehicle.guided_navigation import GuidedNavigation
from navpy.modules.vehicle.heartbeat_runtime import HeartbeatRuntime
from navpy.modules.vehicle.inbound_router import InboundMessageRouter
from navpy.modules.vehicle.link_state import HeartbeatState, PacketLossTracker
from navpy.modules.vehicle.logger_ref import LoggerRef
from navpy.modules.vehicle.mav_bus import MavBus, MavBusLease
from navpy.modules.vehicle.mav_mission import MavMission
from navpy.modules.vehicle.mav_transport import MavTransport
from navpy.modules.vehicle.message_store import MessageStore
from navpy.modules.vehicle.message_subscriptions import CallbackRegistry
from navpy.modules.vehicle.mission_inbox import MissionInbox
from navpy.modules.vehicle.mode_control import ModeControl
from navpy.modules.vehicle.parameter_client import (
    ParameterClient,
    SimAutopilotCapability,
)
from navpy.modules.vehicle.parameter_repository import ParameterRepository
from navpy.modules.vehicle.parameter_snapshot import ParameterSnapshotClient
from navpy.modules.vehicle.pose_telemetry import CommandDiagnostics, PoseTelemetry
from navpy.modules.vehicle.position_service import PositionService
from navpy.modules.vehicle.power_gps_telemetry import PowerGpsTelemetry
from navpy.modules.vehicle.preflight_health import PreflightHealth
from navpy.modules.vehicle.transport_identity import TransportIdentity
from navpy.modules.vehicle.vehicle_commands import VehicleCommands
from navpy.modules.vehicle.vehicle_identity import VehicleIdentity
from navpy.modules.vehicle.vehicle_lifecycle import VehicleLifecycle
from navpy.modules.vehicle.vehicle_lifetime import VehicleLifetime
from navpy.modules.vehicle.vehicle_messaging import VehicleMessaging


@dataclass(frozen=True)
class VehicleLinkRequest:
    device: str
    target_system: int
    baud: int
    mav_type: int
    mav_comp_id: int
    bus: MavBus | None


@dataclass(frozen=True)
class VehicleStartupPolicy:
    skip_mission_download: bool
    wait_heartbeat: bool
    send_heartbeat: bool
    heartbeat_hz: float
    heartbeat_timeout: float


@dataclass(frozen=True)
class VehicleBuildRequest:
    link: VehicleLinkRequest
    startup: VehicleStartupPolicy
    logger: ILogger


@dataclass(frozen=True)
class VehicleLinkContext:
    identity: VehicleIdentity
    logger_ref: LoggerRef
    active_bus: MavBus
    bus_lease: MavBusLease
    transport: MavTransport
    # Canonical endpoint the connection was opened on; with the remote system
    # id it forms the LIVE link identity that capture-lookup calibration
    # binds to.
    device: str = ""


@dataclass(frozen=True)
class VehicleStateStores:
    messages: MessageStore
    callbacks: CallbackRegistry
    parameter_repository: ParameterRepository
    heartbeat_state: HeartbeatState
    packet_loss: PacketLossTracker
    mission_inbox: MissionInbox
    diagnostics: CommandDiagnostics


@dataclass(frozen=True)
class VehicleFoundation:
    link: VehicleLinkContext
    state: VehicleStateStores


@dataclass(frozen=True)
class VehicleProtocolParts:
    mission: MavMission
    router: InboundMessageRouter
    snapshot: ParameterSnapshotClient
    heartbeat_runtime: HeartbeatRuntime
    lifecycle: VehicleLifecycle


@dataclass(frozen=True)
class VehicleCapabilityParts:
    power_gps: PowerGpsTelemetry
    flight: FlightTelemetry
    pose: PoseTelemetry
    parameters: ParameterClient
    simulation: SimAutopilotCapability
    mode: ModeControl
    health: PreflightHealth
    position: PositionService
    navigation: GuidedNavigation
    attitude: AttitudeCommand


@dataclass(frozen=True)
class VehicleRuntimeParts:
    identity: VehicleIdentity
    transport_identity: TransportIdentity
    messaging: VehicleMessaging
    commands: VehicleCommands
    lifetime: VehicleLifetime


@dataclass(frozen=True)
class VehicleParts:
    power_gps: PowerGpsTelemetry
    flight: FlightTelemetry
    pose: PoseTelemetry
    parameters: ParameterClient
    simulation: SimAutopilotCapability
    mode: ModeControl
    health: PreflightHealth
    position: PositionService
    mission: MavMission
    navigation: GuidedNavigation
    attitude: AttitudeCommand
    runtime: VehicleRuntimeParts

    @property
    def identity(self) -> VehicleIdentity:
        return self.runtime.identity

    @property
    def transport_identity(self) -> TransportIdentity:
        return self.runtime.transport_identity

    @property
    def messaging(self) -> VehicleMessaging:
        return self.runtime.messaging

    @property
    def commands(self) -> VehicleCommands:
        return self.runtime.commands

    @property
    def lifetime(self) -> VehicleLifetime:
        return self.runtime.lifetime


__all__ = [
    "VehicleBuildRequest",
    "VehicleCapabilityParts",
    "VehicleFoundation",
    "VehicleLinkContext",
    "VehicleLinkRequest",
    "VehicleParts",
    "VehicleProtocolParts",
    "VehicleRuntimeParts",
    "VehicleStartupPolicy",
    "VehicleStateStores",
]
