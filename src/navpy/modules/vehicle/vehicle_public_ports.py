"""Narrow capability bundles consumed by public VehicleMav facets."""

from __future__ import annotations

from typing import Protocol

from navpy.modules.vehicle.attitude_command import AttitudeCommand
from navpy.modules.vehicle.flight_telemetry import FlightTelemetry
from navpy.modules.vehicle.guided_navigation import GuidedNavigation
from navpy.modules.vehicle.mav_mission import MavMission
from navpy.modules.vehicle.mode_control import ModeControl
from navpy.modules.vehicle.parameter_client import (
    ParameterClient,
    SimAutopilotCapability,
)
from navpy.modules.vehicle.pose_telemetry import PoseTelemetry
from navpy.modules.vehicle.position_service import PositionService
from navpy.modules.vehicle.power_gps_telemetry import PowerGpsTelemetry
from navpy.modules.vehicle.preflight_health import PreflightHealth
from navpy.modules.vehicle.transport_identity import TransportIdentity
from navpy.modules.vehicle.vehicle_commands import VehicleCommands
from navpy.modules.vehicle.vehicle_identity import VehicleIdentity
from navpy.modules.vehicle.vehicle_lifetime import VehicleLifetime
from navpy.modules.vehicle.vehicle_messaging import VehicleMessaging


class IdentityParts(Protocol):
    identity: VehicleIdentity
    transport_identity: TransportIdentity


class PowerGpsParts(Protocol):
    power_gps: PowerGpsTelemetry


class MotionParts(Protocol):
    flight: FlightTelemetry


class PoseParts(Protocol):
    pose: PoseTelemetry


class FlightControlParts(Protocol):
    navigation: GuidedNavigation
    attitude: AttitudeCommand
    mode: ModeControl


class CommandParts(Protocol):
    commands: VehicleCommands


class LifetimeParts(Protocol):
    lifetime: VehicleLifetime


class MessagingParts(Protocol):
    messaging: VehicleMessaging
    lifetime: VehicleLifetime


class MissionParts(Protocol):
    mission: MavMission


class SimulationParts(Protocol):
    simulation: SimAutopilotCapability


class ParameterParts(Protocol):
    parameters: ParameterClient


class LimitsModeParts(Protocol):
    parameters: ParameterClient
    mode: ModeControl


class HealthParts(Protocol):
    health: PreflightHealth


class PositionMissionParts(Protocol):
    position: PositionService
    mission: MavMission


__all__ = [
    "CommandParts",
    "FlightControlParts",
    "HealthParts",
    "IdentityParts",
    "LifetimeParts",
    "LimitsModeParts",
    "MessagingParts",
    "MissionParts",
    "MotionParts",
    "ParameterParts",
    "PoseParts",
    "PositionMissionParts",
    "PowerGpsParts",
    "SimulationParts",
]
