"""Narrow capability bundles consumed by public VehicleMav facets."""

from __future__ import annotations

from typing import Protocol

from navpy.plugins.vehicle.ardupilot.attitude_command import AttitudeCommand
from navpy.plugins.vehicle.ardupilot.flight_telemetry import FlightTelemetry
from navpy.plugins.vehicle.ardupilot.guided_navigation import GuidedNavigation
from navpy.plugins.vehicle.ardupilot.mav_mission import MavMission
from navpy.plugins.vehicle.ardupilot.mode_control import ModeControl
from navpy.plugins.vehicle.ardupilot.parameter_client import ParameterClient
from navpy.plugins.vehicle.ardupilot.sim_autopilot import SimAutopilotCapability
from navpy.plugins.vehicle.ardupilot.pose_telemetry import PoseTelemetry
from navpy.plugins.vehicle.ardupilot.position_service import PositionService
from navpy.plugins.vehicle.ardupilot.power_gps_telemetry import PowerGpsTelemetry
from navpy.plugins.vehicle.ardupilot.preflight_health import PreflightHealth
from navpy.plugins.vehicle.ardupilot.transport_identity import TransportIdentity
from navpy.plugins.vehicle.ardupilot.vehicle_commands import VehicleCommands
from navpy.plugins.vehicle.ardupilot.vehicle_identity import VehicleIdentity
from navpy.plugins.vehicle.ardupilot.vehicle_lifetime import VehicleLifetime
from navpy.plugins.vehicle.ardupilot.vehicle_messaging import VehicleMessaging


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
