"""Stable VehicleMav compatibility boundary over focused collaborators."""
from __future__ import annotations

from pymavlink import mavutil
from pymavlink.dialects.v20.ardupilotmega import (
    MAV_COMP_ID_ONBOARD_COMPUTER,
    MAV_TYPE_ONBOARD_CONTROLLER,
)

from navpy.logger.cache_logger import ConsoleLogger, ILogger
from navpy.modules.vehicle.parameter_spec import value_in_int_range as _value_in_int_range
from navpy.modules.vehicle.preflight_health import (
    PREARM_STATE_CHECKS_DISABLED,
    PREARM_STATE_FAILED,
    PREARM_STATE_NOT_REPORTED,
    PREARM_STATE_NO_SYS_STATUS,
    PREARM_STATE_OK,
)
from navpy.modules.vehicle.vehicle_composition import build_vehicle_parts
from navpy.modules.vehicle.vehicle_interface import IVehicle
from navpy.modules.vehicle.vehicle_public_control import (
    VehicleCommandFacet,
    VehicleFlightControlFacet,
    VehicleLifetimeFacet,
)
from navpy.modules.vehicle.vehicle_public_messaging import VehicleMessagingFacet
from navpy.modules.vehicle.vehicle_public_mission import (
    VehicleFenceFacet,
    VehicleMissionEditFacet,
    VehicleMissionTransferFacet,
)
from navpy.modules.vehicle.vehicle_public_parameters import (
    VehicleParameterFacet,
    VehicleSimulationFacet,
)
from navpy.modules.vehicle.vehicle_public_status import (
    VehicleHealthFacet,
    VehicleLimitsModeFacet,
    VehiclePositionMissionViewFacet,
)
from navpy.modules.vehicle.vehicle_public_telemetry import (
    VehicleIdentityFacet,
    VehicleMotionFacet,
    VehiclePoseFacet,
    VehiclePowerGpsFacet,
)


STARTUP_HEARTBEAT_TIMEOUT_S = 30.0
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
        heartbeat_hz: float = 1.0,
        heartbeat_timeout: float = 3.0,
        mav_type: int = MAV_TYPE_ONBOARD_CONTROLLER,
        mav_comp_id: int = MAV_COMP_ID_ONBOARD_COMPUTER,
        bus=None,
    ) -> None:
        self._parts = build_vehicle_parts(
            device, target_system, baud, logger, skip_mission_download,
            wait_heartbeat, send_heartbeat, heartbeat_hz, heartbeat_timeout,
            mav_type, mav_comp_id, bus,
        )
