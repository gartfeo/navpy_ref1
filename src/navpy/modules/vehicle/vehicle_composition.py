"""Stable application boundary for staged vehicle construction."""

from __future__ import annotations

from navpy.logger.cache_logger import ILogger
from navpy.modules.vehicle.mav_bus import MavBus
from navpy.modules.vehicle.vehicle_build_transaction import (
    build_requested_vehicle as _build_vehicle_parts,
)
from navpy.modules.vehicle.vehicle_build_types import (
    VehicleBuildRequest,
    VehicleCapabilityParts,
    VehicleFoundation,
    VehicleLinkContext,
    VehicleLinkRequest,
    VehicleParts,
    VehicleProtocolParts,
    VehicleRuntimeParts,
    VehicleStartupPolicy,
    VehicleStateStores,
)


def build_vehicle_parts(
    device: str,
    target_system: int,
    baud: int,
    logger: ILogger,
    skip_mission_download: bool,
    wait_heartbeat: bool,
    send_heartbeat: bool,
    heartbeat_hz: float,
    heartbeat_timeout: float,
    mav_type: int,
    mav_comp_id: int,
    bus: MavBus | None,
) -> VehicleParts:
    """Translate the compatibility signature into one immutable request."""
    return _build_vehicle_parts(
        VehicleBuildRequest(
            link=VehicleLinkRequest(
                device,
                target_system,
                baud,
                mav_type,
                mav_comp_id,
                bus,
            ),
            startup=VehicleStartupPolicy(
                skip_mission_download,
                wait_heartbeat,
                send_heartbeat,
                heartbeat_hz,
                heartbeat_timeout,
            ),
            logger=logger,
        )
    )


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
    "build_vehicle_parts",
]
