"""Stateless mission facets for :class:`VehicleMav`."""

from __future__ import annotations

from collections.abc import Callable, Iterable

from pymavlink.dialects.v20.ardupilotmega import (
    MAV_MISSION_TYPE_FENCE,
    MAVLink_message,
)
from pymavlink.mavwp import MAVWPLoader

from navpy.modules.common.models.location import Location
from navpy.modules.vehicle.vehicle_public_ports import MissionParts


class VehicleMissionEditFacet:
    """Delegate mission collection reads and edits."""

    _parts: MissionParts

    def load_mission_items(self, loader: MAVWPLoader) -> None:
        return self._parts.mission.load_items(loader)

    def get_mission_item(self, sequence: int) -> MAVLink_message | None:
        return self._parts.mission.get_item(sequence)

    def get_mission_item_location(self, sequence: int) -> Location | None:
        return self._parts.mission.get_item_location(sequence)

    def update_mission_item(
        self,
        sequence: int,
        command: MAVLink_message,
    ) -> None:
        return self._parts.mission.update_item(sequence, command)

    def update_mission_item_location(
        self,
        sequence: int,
        location: Location,
        alt: float,
    ) -> None:
        return self._parts.mission.update_item_location(sequence, location, alt)


class VehicleMissionTransferFacet:
    """Delegate mission protocol transfers and sequencing."""

    _parts: MissionParts

    def download_mission(
        self,
        timeout: float = 10.0,
        retries: int = 3,
        on_progress: Callable[[int, int], None] | None = None,
    ) -> int:
        return self._parts.mission.download(timeout, retries, on_progress)

    def upload_mission(
        self,
        timeout: float = 10.0,
        retries: int = 3,
        on_wp_sent: Callable[[int, int], None] | None = None,
    ) -> bool:
        return self._parts.mission.upload(timeout, retries, on_wp_sent)

    def clear_mission(self) -> None:
        return self._parts.mission.clear()

    def restart_mission(self, mission_index: int = 0) -> None:
        return self._parts.mission.restart(mission_index)

    def set_current(self, index: int) -> None:
        return self._parts.mission.set_current(index)


class VehicleFenceFacet:
    """Delegate fence transfers using the fence mission namespace."""

    _parts: MissionParts

    def upload_fence(
        self,
        items: Iterable[MAVLink_message],
        timeout: float = 10.0,
        retries: int = 3,
    ) -> bool:
        return self._parts.mission.upload(
            timeout,
            retries,
            items=items,
            mission_type=MAV_MISSION_TYPE_FENCE,
        )

    def download_fence(
        self,
        timeout: float = 10.0,
        retries: int = 3,
    ) -> list[MAVLink_message]:
        return self._parts.mission.download_items(
            timeout,
            retries,
            MAV_MISSION_TYPE_FENCE,
        )

    def clear_fence(self) -> None:
        return self._parts.mission.clear(MAV_MISSION_TYPE_FENCE)
