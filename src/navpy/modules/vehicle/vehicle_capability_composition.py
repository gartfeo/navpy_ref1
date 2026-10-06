"""Compose focused telemetry and command capabilities."""

from __future__ import annotations

from collections.abc import Callable

from navpy.modules.vehicle.attitude_command import AttitudeCommand
from navpy.modules.vehicle.flight_telemetry import FlightTelemetry
from navpy.modules.vehicle.guided_navigation import GuidedNavigation
from navpy.modules.vehicle.mode_control import ModeControl
from navpy.modules.vehicle.parameter_client import (
    ParameterClient,
    ParameterReader,
    ParameterWriter,
    SimAutopilotCapability,
    SimAutopilotState,
)
from navpy.modules.vehicle.pose_telemetry import PoseTelemetry
from navpy.modules.vehicle.position_service import PositionService
from navpy.modules.vehicle.power_gps_telemetry import PowerGpsTelemetry
from navpy.modules.vehicle.preflight_health import PreflightHealth
from navpy.modules.vehicle.vehicle_build_types import (
    VehicleCapabilityParts,
    VehicleFoundation,
    VehicleProtocolParts,
)


def _link_identity_source(
    foundation: VehicleFoundation,
) -> "Callable[[], str | None]":
    """Provider for the live-link identity: endpoint + CONFIRMED remote.

    The bus routes only the leased target system to this vehicle, so the
    first observed heartbeat IS the remote confirming itself on this
    endpoint. Before that: None (nothing admits). The identity also DIES
    with the link: once the heartbeat ages past the timeout (`link_ok`
    False) the provider returns None again, so a dropped link invalidates
    every calibration until the remote is heard again after the
    autoreconnect — the revalidation cycle the decision doc requires. The
    endpoint cannot change within a bus lifetime, and an autopilot
    reboot/replay is additionally caught downstream by the boot-clock seam.
    """
    heartbeat_state = foundation.state.heartbeat_state
    identity = (
        f"{foundation.link.device}"
        f"#{foundation.link.identity.target_system}"
    )

    def read() -> str | None:
        confirmed = heartbeat_state.message is not None
        return identity if confirmed and heartbeat_state.link_ok else None

    return read


def build_capability_parts(
    foundation: VehicleFoundation,
    protocols: VehicleProtocolParts,
) -> VehicleCapabilityParts:
    reader = ParameterReader(
        foundation.link.identity,
        foundation.link.transport,
        foundation.state.parameter_repository,
        foundation.state.messages,
        foundation.link.logger_ref,
    )
    sim_state = SimAutopilotState()
    writer = ParameterWriter(
        foundation.link.identity,
        foundation.link.transport,
        foundation.state.parameter_repository,
        foundation.state.messages,
        foundation.link.logger_ref,
        sim_state,
    )
    parameters = ParameterClient(reader, writer, protocols.snapshot)
    simulation = SimAutopilotCapability(
        foundation.link.identity,
        foundation.link.transport,
        foundation.state.parameter_repository,
        reader,
        sim_state,
    )
    mode = ModeControl(
        foundation.link.identity,
        foundation.link.transport,
        foundation.state.heartbeat_state,
        foundation.link.logger_ref,
    )
    position = PositionService(
        foundation.link.identity,
        foundation.link.transport,
        foundation.state.messages,
        protocols.mission,
        foundation.link.logger_ref,
    )
    return VehicleCapabilityParts(
        power_gps=PowerGpsTelemetry(foundation.state.messages),
        flight=FlightTelemetry(foundation.state.messages),
        pose=PoseTelemetry(
            foundation.state.messages,
            foundation.state.diagnostics,
            # LIVE provider, not a frozen string: the endpoint comes from the
            # bus actually in use, and the identity exists only once the
            # remote's heartbeat has been observed on it -- "identity only
            # after the remote sysid is known". Until then it is None and
            # every capture calibration refuses to admit.
            link_identity_source=_link_identity_source(foundation),
        ),
        parameters=parameters,
        simulation=simulation,
        mode=mode,
        health=PreflightHealth(
            foundation.state.messages,
            foundation.state.heartbeat_state,
            foundation.state.packet_loss,
        ),
        position=position,
        navigation=GuidedNavigation(
            foundation.link.identity,
            foundation.link.transport,
            mode,
            position,
            parameters,
            foundation.link.logger_ref,
        ),
        attitude=AttitudeCommand(
            foundation.link.identity,
            foundation.link.transport,
            foundation.state.diagnostics,
        ),
    )


__all__ = ["build_capability_parts"]
