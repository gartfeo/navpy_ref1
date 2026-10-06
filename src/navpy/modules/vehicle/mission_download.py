"""Fresh-response MAVLink mission download state machine."""

from __future__ import annotations

import time
from collections.abc import Callable

from pymavlink.dialects.v20.ardupilotmega import (
    MAV_MISSION_TYPE_MISSION,
    MAVLink_message,
)

from navpy.modules.vehicle.logger_ref import LoggerRef
from navpy.modules.vehicle.mav_transport import MavTransport
from navpy.modules.vehicle.message_store import MessageStore
from navpy.modules.vehicle.mission_store import MissionStore

MissionProgress = Callable[[int, int], None]


class MissionDownloader:
    def __init__(
        self,
        target_system: int,
        transport: MavTransport,
        store: MissionStore,
        messages: MessageStore,
        logger_ref: LoggerRef,
    ) -> None:
        self._target = target_system
        self._transport = transport
        self._store = store
        self._messages = messages
        self._logger_ref = logger_ref

    def download(
        self,
        timeout: float = 10.0,
        retries: int = 3,
        on_progress: MissionProgress | None = None,
        mission_type: int = MAV_MISSION_TYPE_MISSION,
    ) -> int:
        logger = self._logger_ref.value
        logger.info("Downloading missionâ€¦")
        count_message = None
        for attempt in range(1, retries + 1):
            cursor = self._messages.cursor()
            self._transport.call(
                lambda connection: connection.mav.mission_request_list_send(
                    self._target,
                    0,
                    mission_type,
                )
            )
            sample = self._messages.wait_after(
                "MISSION_COUNT",
                cursor,
                lambda message: self._from_target_table(
                    message,
                    mission_type,
                ),
                deadline=time.monotonic() + timeout / 2,
            )
            if sample is not None:
                count_message = sample.message
                break
            logger.warning(f"No MISSION_COUNT (attempt {attempt}/{retries})")
        if count_message is None:
            logger.error("Giving up â€“ no MISSION_COUNT.")
            return 0
        total = count_message.count
        if total == 0:
            self._store.replace_items([])
            return 0
        staged_items = []
        for sequence in range(total):
            received = None
            for attempt in range(1, retries + 1):
                cursor = self._messages.cursor()
                self._transport.call(
                    lambda connection, seq=sequence: (
                        connection.mav.mission_request_int_send(
                            self._target,
                            0,
                            seq,
                            mission_type,
                        )
                    )
                )
                sample = self._messages.wait_after(
                    ("MISSION_ITEM_INT", "MISSION_ITEM"),
                    cursor,
                    lambda message, seq=sequence: (
                        self._from_target_table(message, mission_type)
                        and getattr(message, "seq", None) == seq
                    ),
                    deadline=time.monotonic() + timeout,
                )
                if sample is not None:
                    received = sample.message
                    break
                logger.warning(
                    f"MISSION_ITEM {sequence} timeout "
                    f"(attempt {attempt}/{retries})"
                )
            if received is None:
                logger.error(f"Giving up â€“ item {sequence} missing.")
                return 0
            staged_items.append(received)
            if on_progress:
                on_progress(sequence + 1, total)
        self._store.replace_items(staged_items)
        logger.info("Mission download successful.")
        return total

    def download_items(
        self,
        timeout: float = 10.0,
        retries: int = 3,
        mission_type: int = MAV_MISSION_TYPE_MISSION,
    ) -> list[MAVLink_message]:
        snapshot = self._store.snapshot()
        try:
            count = self.download(timeout, retries, mission_type=mission_type)
            return self._store.snapshot() if count > 0 else []
        finally:
            self._store.restore(snapshot)

    def _from_target_table(
        self,
        message: MAVLink_message,
        mission_type: int,
    ) -> bool:
        try:
            if message.get_srcSystem() != self._target:
                return False
        except Exception:
            return False
        return getattr(
            message,
            "mission_type",
            MAV_MISSION_TYPE_MISSION,
        ) == mission_type


__all__ = ["MissionDownloader"]
