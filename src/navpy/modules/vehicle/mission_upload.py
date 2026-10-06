"""Fresh-response MAVLink mission upload state machine."""

from __future__ import annotations

import time
from collections.abc import Callable, Iterable

from pymavlink.dialects.v20.ardupilotmega import (
    MAV_MISSION_TYPE_MISSION,
    MAVLink_message,
)

from navpy.modules.vehicle.logger_ref import LoggerRef
from navpy.modules.vehicle.mav_transport import MavTransport
from navpy.modules.vehicle.mission_inbox import MissionInbox
from navpy.modules.vehicle.mission_store import MissionStore
from navpy.modules.vehicle.mission_upload_matching import MissionUploadMatcher

MissionProgress = Callable[[int, int], None]


class MissionUploader:
    def __init__(
        self,
        target_system: int,
        transport: MavTransport,
        store: MissionStore,
        inbox: MissionInbox,
        logger_ref: LoggerRef,
    ) -> None:
        self._target = target_system
        self._transport = transport
        self._store = store
        self._inbox = inbox
        self._logger_ref = logger_ref
        self._matcher = MissionUploadMatcher(target_system)

    def upload(
        self,
        timeout: float = 10.0,
        retries: int = 3,
        on_wp_sent: MissionProgress | None = None,
        items: Iterable[MAVLink_message] | None = None,
        mission_type: int = MAV_MISSION_TYPE_MISSION,
    ) -> bool:
        logger = self._logger_ref.value
        logger.info("Uploading missionâ€¦")
        waypoints = self._store.snapshot() if items is None else list(items)
        total = len(waypoints)
        if total == 0:
            return True
        first_envelope = None
        for attempt in range(1, retries + 1):
            cursor = self._inbox.cursor()
            self._transport.call(
                lambda connection: connection.mav.mission_count_send(
                    self._target,
                    0,
                    total,
                    mission_type,
                )
            )
            envelope = self._inbox.wait_after(
                cursor,
                lambda message: self._matcher.is_request(
                    message,
                    0,
                    mission_type,
                ),
                deadline=time.monotonic() + timeout / 2,
            )
            if envelope is not None:
                first_envelope = envelope
                break
            logger.warning(
                f"No MISSION_REQUEST for item 0 (attempt {attempt}/{retries})"
            )
        if first_envelope is None:
            logger.error("Giving up â€“ autopilot ignores MISSION_COUNT.")
            return False
        envelope = first_envelope
        next_sequence = 0
        while next_sequence < total:
            request = envelope.message
            sequence = request.seq
            if sequence < 0 or sequence >= total:
                logger.warning(f"Ignoring out-of-range request seq={sequence}")
                response_cursor = envelope.generation
            else:
                response_cursor = self._inbox.cursor()
                waypoint = waypoints[sequence]
                self._send_item(waypoint, sequence, mission_type)
                if sequence >= next_sequence:
                    next_sequence = sequence + 1
                    if on_wp_sent:
                        on_wp_sent(next_sequence, total)
                if next_sequence >= total:
                    ack = self._inbox.wait_after(
                        response_cursor,
                        lambda message: self._matcher.is_accepted_ack(
                            message,
                            mission_type,
                        ),
                        deadline=time.monotonic() + 3.0,
                    )
                    ok = ack is not None
                    logger.info(
                        "Mission upload "
                        f"{'successful' if ok else 'failed / timed out'}."
                    )
                    return ok
            envelope = self._inbox.wait_after(
                response_cursor,
                lambda message: self._matcher.is_request(
                    message,
                    mission_type=mission_type,
                ),
                deadline=time.monotonic() + timeout,
            )
            if envelope is None:
                logger.warning(
                    "Timeout waiting for MISSION_REQUEST "
                    f"(expecting {next_sequence})"
                )
                return False
            request = envelope.message
        return False

    def clear(self, mission_type: int = MAV_MISSION_TYPE_MISSION) -> None:
        self._transport.call(
            lambda connection: connection.mav.mission_clear_all_send(
                self._target,
                0,
                mission_type,
            )
        )
        if mission_type == MAV_MISSION_TYPE_MISSION:
            self._store.clear()

    def set_current(self, index: int) -> None:
        self._transport.call(
            lambda connection: connection.mav.mission_set_current_send(
                self._target,
                0,
                index,
            )
        )

    def _send_item(
        self,
        waypoint: MAVLink_message,
        sequence: int,
        mission_type: int,
    ) -> None:
        self._transport.call(
            lambda connection: connection.mav.mission_item_int_send(
                self._target,
                0,
                sequence,
                waypoint.frame,
                waypoint.command,
                waypoint.current,
                waypoint.autocontinue,
                waypoint.param1,
                waypoint.param2,
                waypoint.param3,
                waypoint.param4,
                waypoint.x,
                waypoint.y,
                waypoint.z,
                getattr(waypoint, "mission_type", mission_type),
            )
        )

__all__ = ["MissionUploader"]
