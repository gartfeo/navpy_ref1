"""Fresh home/current position and terrain queries."""
from __future__ import annotations

import math
import time
from collections.abc import Callable
from copy import deepcopy

from pymavlink.dialects.v20.ardupilotmega import (
    MAV_CMD_REQUEST_MESSAGE,
    MAVLINK_MSG_ID_HOME_POSITION,
    MAVLink_home_position_message,
    MAVLink_message,
)

from navpy.modules.common.models.location import Location
from navpy.modules.vehicle.logger_ref import LoggerRef
from navpy.modules.vehicle.mav_mission import MavMission
from navpy.modules.vehicle.mav_transport import MavTransport
from navpy.modules.vehicle.message_store import MessageStore
from navpy.modules.vehicle.vehicle_identity import VehicleIdentity


MODULE_START_MONOTONIC_S = time.monotonic()


def default_elevation_lookup(
    latitude: float,
    longitude: float,
) -> float | None:
    from navpy.utils.zc_util_instance import ZcUtilInstance

    utility = ZcUtilInstance.get()
    return utility.get_elevation((latitude, longitude)) if utility else None


class PositionService:
    def __init__(
        self,
        identity: VehicleIdentity,
        transport: MavTransport,
        messages: MessageStore,
        mission: MavMission,
        logger_ref: LoggerRef,
        elevation_lookup: Callable[[float, float], float | None] = (
            default_elevation_lookup
        ),
    ) -> None:
        self._identity = identity
        self._transport = transport
        self._messages = messages
        self._mission = mission
        self._logger_ref = logger_ref
        self._elevation_lookup = elevation_lookup

    @property
    def home_location(self) -> Location | None:
        home = self._messages.message("HOME_POSITION")
        if home is None:
            cursor = self._messages.cursor()
            self._transport.call(
                lambda connection: connection.mav.command_long_send(
                    self._identity.target_system,
                    0,
                    MAV_CMD_REQUEST_MESSAGE,
                    0,
                    MAVLINK_MSG_ID_HOME_POSITION,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                )
            )
            sample = self._messages.wait_after(
                "HOME_POSITION",
                cursor,
                self._from_poi,
                deadline=time.monotonic() + 1.0,
            )
            home = sample.message if sample else None
        if home is None:
            home = self._fallback_home()
        if home is None or (home.latitude == 0 and home.longitude == 0):
            return None
        return Location(
            home.latitude / 1e7,
            home.longitude / 1e7,
            home.altitude / 1e3,
            is_absolute=True,
        )

    def location(self, is_relative: bool) -> Location | None:
        position = self._messages.message("GLOBAL_POSITION_INT")
        if position is None:
            return None
        altitude = (
            position.relative_alt / 1e3 if is_relative else position.alt / 1e3
        )
        return Location(
            position.lat / 1e7,
            position.lon / 1e7,
            altitude,
            is_absolute=not is_relative,
        )

    def terrain_height_at(
        self,
        latitude: float,
        longitude: float,
        *,
        timeout: float = 1.0,
    ) -> float | None:
        latitude_int = int(float(latitude) * 1e7)
        longitude_int = int(float(longitude) * 1e7)
        cursor = self._messages.cursor()
        try:
            self._transport.call(
                lambda connection: connection.mav.terrain_check_send(
                    latitude_int, longitude_int,
                )
            )
        except Exception:
            return None
        sample = self._messages.wait_after(
            "TERRAIN_REPORT",
            cursor,
            lambda message: self._matches_terrain(
                message, latitude_int, longitude_int,
            ),
            deadline=time.monotonic() + timeout,
        )
        if sample is None:
            return None
        try:
            # Availability applies to this location; pending counts all blocks
            # still loading and can be nonzero for a valid location report.
            if int(sample.message.spacing) <= 0:
                return None
            terrain_height = float(sample.message.terrain_height)
        except (AttributeError, TypeError, ValueError):
            return None
        return terrain_height if math.isfinite(terrain_height) else None

    def _fallback_home(self) -> MAVLink_home_position_message | None:
        if self._mission.count <= 0:
            return None
        waypoint = self._mission.get_item(0)
        if waypoint and waypoint.x and waypoint.y:
            self._logger_ref.value.warning(
                f"Using home from mission: {waypoint.x}, {waypoint.y}, {waypoint.z}"
            )
        else:
            last = self._mission.get_item(self._mission.count - 1)
            if last is None or not last.x or not last.y:
                return None
            elevation = self._elevation_lookup(last.x / 1e7, last.y / 1e7)
            if elevation is None or elevation == 0:
                return None
            waypoint = deepcopy(last)
            waypoint.z = elevation
            self._logger_ref.value.warning(
                f"Using last waypoint {last.seq} as home: "
                f"{waypoint.x}, {waypoint.y}, {waypoint.z}"
            )
        home = MAVLink_home_position_message(
            waypoint.x,
            waypoint.y,
            int(round(waypoint.z * 1000)),
            0.0,
            0.0,
            0.0,
            [1.0, 0.0, 0.0, 0.0],
            0.0,
            0.0,
            0.0,
            int((time.monotonic() - MODULE_START_MONOTONIC_S) * 1000)
            & 0xFFFFFFFF,
        )
        self._messages.publish(
            "HOME_POSITION", home, receipt_time_s=time.time(),
        )
        return home

    def _from_poi(self, message: MAVLink_message) -> bool:
        try:
            return message.get_srcSystem() == self._identity.target_system
        except Exception:
            return False

    def _matches_terrain(
        self,
        message: MAVLink_message,
        latitude: int,
        longitude: int,
    ) -> bool:
        return (
            self._from_poi(message)
            and abs(int(getattr(message, "lat", 0)) - latitude) <= 2
            and abs(int(getattr(message, "lon", 0)) - longitude) <= 2
        )
