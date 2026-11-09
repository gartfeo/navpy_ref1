"""Stable VehicleMav compatibility boundary over focused collaborators."""
from __future__ import annotations

from collections.abc import Callable

from pymavlink import mavutil
from pymavlink.dialects.v20.ardupilotmega import (
    MAV_COMP_ID_ONBOARD_COMPUTER,
    MAV_TYPE_GCS,
    MAV_TYPE_ONBOARD_CONTROLLER,
)

from navpy.core.kernel.logging.cache_logger import ConsoleLogger, ILogger
from navpy.plugins.vehicle.ardupilot.vehicle_composition import build_vehicle_parts
from navpy.core.ports.vehicle_contract import IVehicle
from navpy.plugins.vehicle.ardupilot.vehicle_public_control import (
    VehicleCommandFacet,
    VehicleFlightControlFacet,
    VehicleLifetimeFacet,
)
from navpy.plugins.vehicle.ardupilot.vehicle_public_messaging import VehicleMessagingFacet
from navpy.plugins.vehicle.ardupilot.vehicle_public_mission import (
    VehicleFenceFacet,
    VehicleMissionEditFacet,
    VehicleMissionTransferFacet,
)
from navpy.plugins.vehicle.ardupilot.vehicle_public_parameters import (
    VehicleParameterFacet,
    VehicleSimulationFacet,
)
from navpy.plugins.vehicle.ardupilot.vehicle_public_status import (
    VehicleHealthFacet,
    VehicleLimitsModeFacet,
    VehiclePositionMissionViewFacet,
)
from navpy.plugins.vehicle.ardupilot.vehicle_public_telemetry import (
    VehicleIdentityFacet,
    VehicleMotionFacet,
    VehiclePoseFacet,
    VehiclePowerGpsFacet,
)


STARTUP_HEARTBEAT_TIMEOUT_S = 30.0
#: The companion's heartbeat rate, in aircraft time (heartbeat_pacing.py): 4 Hz (owner ruling 2026-11-08) against
#: the autopilot NavPy watchdog's 1.5 s timeout (scripts/lua/nav_watchdog.lua, AAS_NWD_TMO), so a hold needs about
#: six beats in a row lost, not one (docs/design/navpy_recovery.md, step 7).
HEARTBEAT_HZ = 4.0
#: The GCS backend's own heartbeat (MAV_TYPE_GCS) stays 1 Hz: no watchdog reads it, and it is uplink on every radio.
GCS_HEARTBEAT_HZ = 1.0
NO_HEARTBEAT_EXIT_MARKER = "is not broadcasting heartbeats"


class VehicleMav(
    VehicleIdentityFacet,
    VehiclePowerGpsFacet,
    VehicleMotionFacet,
    VehiclePoseFacet,
    VehicleLimitsModeFacet,
    VehicleHealthFacet,
    VehiclePositionMissionViewFacet,
    VehicleFlightControlFacet,
    VehicleSimulationFacet,
    VehicleParameterFacet,
    VehicleMissionEditFacet,
    VehicleMissionTransferFacet,
    VehicleFenceFacet,
    VehicleMessagingFacet,
    VehicleCommandFacet,
    VehicleLifetimeFacet,
    IVehicle,
):
    """Frozen direct-delegation adapter; behavior belongs to focused owners."""

    def __init__(
        self,
        device: str,
        target_system: int,
        baud: int = 115200,
        logger: ILogger = ConsoleLogger(),
        skip_mission_download: bool = False,
        wait_heartbeat: bool = True,
        send_heartbeat: bool = True,
        heartbeat_timeout: float = 3.0,
        mav_type: int = MAV_TYPE_ONBOARD_CONTROLLER,
        mav_comp_id: int = MAV_COMP_ID_ONBOARD_COMPUTER,
        bus=None,
        stop_requested: Callable[[], bool] | None = None,
    ) -> None:
        self._parts = build_vehicle_parts(
            device, target_system, baud, logger, skip_mission_download,
            wait_heartbeat, send_heartbeat,
            GCS_HEARTBEAT_HZ if mav_type == MAV_TYPE_GCS else HEARTBEAT_HZ, heartbeat_timeout,
            mav_type, mav_comp_id, bus, stop_requested,
        )
