"""In-memory mission table ownership."""
from __future__ import annotations

from collections.abc import Callable, Iterable

from pymavlink.dialects.v20.ardupilotmega import (
    MAV_CMD_NAV_TAKEOFF,
    MAV_FRAME_GLOBAL_INT,
    MAV_FRAME_GLOBAL_RELATIVE_ALT,
    MAVLink_message,
)
from pymavlink.mavwp import MAVWPLoader

from navpy.modules.common.models.location import Location
from navpy.modules.vehicle.message_store import MessageStore


class MissionStore:
    def __init__(
        self,
        target_system: int,
        message_store: MessageStore,
        loader: MAVWPLoader | None = None,
        loader_factory: Callable[[], MAVWPLoader] | None = None,
    ) -> None:
        self._loader_factory = loader_factory or (
            lambda: MAVWPLoader(target_system, 0)
        )
        self._loader = loader if loader is not None else self._loader_factory()
        self._messages = message_store

    @property
    def count(self) -> int:
        return self._loader.count()

    @property
    def next_sequence(self) -> int | None:
        current = self._messages.message("MISSION_CURRENT")
        return getattr(current, "seq", None)

    def get(self, sequence: int) -> MAVLink_message | None:
        return self._loader.wp(sequence) if 0 <= sequence < self.count else None

    def get_location(self, sequence: int) -> Location | None:
        waypoint = self.get(sequence)
        if waypoint is None:
            return None
        if waypoint.command == MAV_CMD_NAV_TAKEOFF:
            raise ValueError(
                "Takeoff command is marked for POI. Please use other WP. "
                f"poi_wp_index: {sequence}"
            )
        if any(getattr(waypoint, name, None) is None for name in ("x", "y", "z")):
            raise ValueError(f"Mission item {sequence} does not have valid coordinates.")
        if getattr(waypoint, "frame", None) is None:
            raise ValueError(f"Mission item {sequence} does not have a valid frame.")
        is_absolute = waypoint.frame == MAV_FRAME_GLOBAL_INT
        altitude = waypoint.z / 1e3 if is_absolute else waypoint.z
        return Location(
            waypoint.x / 1e7,
            waypoint.y / 1e7,
            altitude,
            is_absolute,
        )

    def update(self, sequence: int, command: MAVLink_message) -> None:
        self._loader.set(command, sequence)

    def update_location(
        self,
        sequence: int,
        location: Location,
        altitude: float,
    ) -> None:
        waypoint = self.get(sequence)
        if waypoint is None:
            raise ValueError(f"Mission item {sequence} does not exist.")
        waypoint.x = int(location.lat * 1e7)
        waypoint.y = int(location.lng * 1e7)
        waypoint.z = int(altitude)
        waypoint.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT
        self._loader.set(waypoint, sequence)

    def replace(self, waypoint_loader: MAVWPLoader) -> None:
        self.replace_items(
            [waypoint_loader.wp(index) for index in range(waypoint_loader.count())]
        )

    def replace_items(self, waypoints: Iterable[MAVLink_message]) -> None:
        replacement = self._loader_factory()
        for waypoint in waypoints:
            replacement.add(waypoint)
        self._loader = replacement

    def clear(self) -> None:
        self._loader.clear()

    def add(self, waypoint: MAVLink_message) -> None:
        self._loader.add(waypoint)

    def snapshot(self) -> list[MAVLink_message]:
        return [self._loader.wp(index) for index in range(self.count)]

    def restore(self, waypoints: Iterable[MAVLink_message]) -> None:
        self.replace_items(waypoints)
