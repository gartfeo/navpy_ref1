"""Mission rewrite used to transfer an aircraft to a peer target."""

from __future__ import annotations

from typing import Protocol

from pymavlink.dialects.v20.ardupilotmega import (
    MAV_CMD_NAV_WAYPOINT,
    MAV_FRAME_GLOBAL_RELATIVE_ALT,
    MAVLink_mission_item_message,
)

from navpy.modules.common.models.location import Location


class PeerTargetMissionVehicle(Protocol):
    target_system: int
    home_location: Location

    def location(self, is_relative: bool) -> Location | None: ...
    def clear_mission(self) -> None: ...
    def update_mission_item(
        self,
        sequence: int,
        command: MAVLink_mission_item_message,
    ) -> None: ...
    def restart_mission(self, mission_index: int = 0) -> None: ...
    def upload_mission(self) -> bool: ...


class PeerTargetMissionWriter:
    """Write the historical four-item peer-target mission."""

    def plan(self, vehicle: PeerTargetMissionVehicle, target: Location) -> None:
        current = vehicle.location(True)
        start_waypoint = self._waypoint(
            vehicle,
            current.lat,
            current.lng,
            current.alt,
        )
        target_waypoint = self._waypoint(
            vehicle,
            target.lat,
            target.lng,
            target.alt - vehicle.home_location.alt,
        )
        vehicle.clear_mission()
        vehicle.update_mission_item(0, start_waypoint)
        vehicle.update_mission_item(1, start_waypoint)
        vehicle.update_mission_item(2, start_waypoint)
        vehicle.update_mission_item(3, target_waypoint)

        # Preserve the existing observable order used by this legacy path.
        vehicle.restart_mission(3)
        vehicle.upload_mission()

    @staticmethod
    def _waypoint(
        vehicle: PeerTargetMissionVehicle,
        latitude: float,
        longitude: float,
        altitude: float,
    ) -> MAVLink_mission_item_message:
        return MAVLink_mission_item_message(
            vehicle.target_system,
            0,
            0,
            MAV_FRAME_GLOBAL_RELATIVE_ALT,
            MAV_CMD_NAV_WAYPOINT,
            0,
            0,
            0,
            0,
            0,
            0,
            latitude * 1e7,
            longitude * 1e7,
            altitude,
        )
