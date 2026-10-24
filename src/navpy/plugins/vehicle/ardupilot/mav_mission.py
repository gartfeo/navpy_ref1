"""Serialized public mission gateway composed from focused protocol owners."""
from __future__ import annotations

from collections.abc import Callable, Iterable

from pymavlink.dialects.v20.ardupilotmega import (
    MAV_MISSION_TYPE_MISSION,
    MAVLink_message,
)
from pymavlink.mavwp import MAVWPLoader

from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.location import Location
from navpy.core.ports.parameter_exchange import ExchangePolicy
from navpy.plugins.vehicle.ardupilot.link_state import HeartbeatState
from navpy.plugins.vehicle.ardupilot.logger_ref import LoggerRef
from navpy.plugins.vehicle.ardupilot.mav_transport import MavTransport
from navpy.plugins.vehicle.ardupilot.message_store import MessageStore
from navpy.plugins.vehicle.ardupilot.mission_inbox import MissionInbox
from navpy.plugins.vehicle.ardupilot.mission_download import MissionDownloader
from navpy.plugins.vehicle.ardupilot.mission_upload import MissionUploader
from navpy.plugins.vehicle.ardupilot.mission_store import MissionStore

MissionProgress = Callable[[int, int], None]


class MavMission:
    """Owns the mission transaction lock; wire state machines live elsewhere."""

    def __init__(
        self,
        target_system: int,
        transport: MavTransport,
        logger_ref: LoggerRef,
        message_store: MessageStore,
        inbox: MissionInbox,
        link: HeartbeatState | None = None,
        store: MissionStore | None = None,
        downloader: MissionDownloader | None = None,
        uploader: MissionUploader | None = None,
        cancelled: Callable[[], bool] | None = None,
    ) -> None:
        """``cancelled`` ends every transfer once true (a NavPy stop, todo 105)."""
        self._lock = rt.RLock()
        self._store = store or MissionStore(target_system, message_store)
        self._downloader = downloader or MissionDownloader(
            target_system,
            transport,
            self._store,
            message_store,
            logger_ref,
            link,
            cancelled,
        )
        self._uploader = uploader or MissionUploader(
            target_system, transport, self._store, inbox, logger_ref, link, cancelled,
        )

    @property
    def lock(self) -> rt.LockPort:
        return self._lock

    @property
    def count(self) -> int:
        return self._store.count

    @property
    def next_seq(self) -> int | None:
        return self._store.next_sequence

    def get_item(self, sequence: int) -> MAVLink_message | None:
        with self._lock:
            return self._store.get(sequence)

    def get_item_location(self, sequence: int) -> Location | None:
        with self._lock:
            return self._store.get_location(sequence)

    def update_item(
        self,
        sequence: int,
        command: MAVLink_message,
    ) -> None:
        with self._lock:
            self._store.update(sequence, command)

    def update_item_location(
        self,
        sequence: int,
        location: Location,
        altitude: float,
    ) -> None:
        with self._lock:
            self._store.update_location(sequence, location, altitude)

    def load_items(self, waypoint_loader: MAVWPLoader) -> None:
        with self._lock:
            self._store.replace(waypoint_loader)

    def download(
        self,
        on_progress: MissionProgress | None = None,
        mission_type: int = MAV_MISSION_TYPE_MISSION,
        policy: ExchangePolicy | None = None,
    ) -> int:
        with self._lock:
            return self._downloader.download(on_progress, mission_type, policy)

    def download_items(
        self,
        mission_type: int = MAV_MISSION_TYPE_MISSION,
        policy: ExchangePolicy | None = None,
    ) -> list[MAVLink_message]:
        with self._lock:
            return self._downloader.download_items(mission_type, policy)

    def upload(
        self,
        on_wp_sent: MissionProgress | None = None,
        items: Iterable[MAVLink_message] | None = None,
        mission_type: int = MAV_MISSION_TYPE_MISSION,
        policy: ExchangePolicy | None = None,
    ) -> bool:
        with self._lock:
            return self._uploader.upload(on_wp_sent, items, mission_type, policy)

    def clear(self, mission_type: int = MAV_MISSION_TYPE_MISSION) -> None:
        with self._lock:
            self._uploader.clear(mission_type)

    def restart(self, mission_index: int = 0) -> None:
        self.set_current(mission_index)

    def set_current(self, index: int) -> None:
        with self._lock:
            self._uploader.set_current(index)
