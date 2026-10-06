"""Assemble public vehicle parts from completed composition stages."""

from __future__ import annotations

from navpy.modules.vehicle.transport_identity import TransportIdentity
from navpy.modules.vehicle.vehicle_build_types import (
    VehicleCapabilityParts,
    VehicleFoundation,
    VehicleParts,
    VehicleProtocolParts,
    VehicleRuntimeParts,
)
from navpy.modules.vehicle.vehicle_commands import VehicleCommands
from navpy.modules.vehicle.vehicle_lifetime import VehicleLifetime
from navpy.modules.vehicle.vehicle_messaging import VehicleMessaging


def assemble_vehicle_parts(
    foundation: VehicleFoundation,
    protocols: VehicleProtocolParts,
    capabilities: VehicleCapabilityParts,
) -> VehicleParts:
    runtime = VehicleRuntimeParts(
        identity=foundation.link.identity,
        transport_identity=TransportIdentity(foundation.link.transport),
        messaging=VehicleMessaging(
            foundation.link.transport,
            foundation.state.callbacks,
            protocols.router,
        ),
        commands=VehicleCommands(
            foundation.link.identity,
            foundation.link.transport,
        ),
        lifetime=VehicleLifetime(
            protocols.lifecycle,
            foundation.link.logger_ref,
        ),
    )
    return VehicleParts(
        power_gps=capabilities.power_gps,
        flight=capabilities.flight,
        pose=capabilities.pose,
        parameters=capabilities.parameters,
        simulation=capabilities.simulation,
        mode=capabilities.mode,
        health=capabilities.health,
        position=capabilities.position,
        mission=protocols.mission,
        navigation=capabilities.navigation,
        attitude=capabilities.attitude,
        runtime=runtime,
    )


__all__ = ["assemble_vehicle_parts"]
