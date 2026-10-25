"""Fresh-response MAVLink mission download state machine.

Each step is one bounded exchange (``docs/design/radio_budget.md``, todo
105): ``MISSION_REQUEST_LIST`` → ``MISSION_COUNT``, then each
``MISSION_REQUEST_INT(seq)`` → that item, under one :class:`ExchangePolicy`
for the whole transfer. Resending a request is safe: the autopilot answers
download requests without state.
"""

from __future__ import annotations

from collections.abc import Callable

from pymavlink.dialects.v20.ardupilotmega import (
    MAV_MISSION_TYPE_MISSION,
    MAVLink_message,
)

from navpy.core.ports.mission_download import MissionDownload, MissionDownloadOutcome
from navpy.core.ports.parameter_exchange import ExchangePolicy, with_cancel
from navpy.plugins.vehicle.ardupilot.link_state import HeartbeatState
from navpy.plugins.vehicle.ardupilot.logger_ref import LoggerRef
from navpy.plugins.vehicle.ardupilot.mav_transport import MavTransport
from navpy.plugins.vehicle.ardupilot.message_store import MessageStore
from navpy.plugins.vehicle.ardupilot.mission_store import MissionStore
from navpy.plugins.vehicle.ardupilot.mission_timing import (
    MISSION_ACK_TIMEOUT_S,
    MISSION_ITEM_TIMEOUT_S,
    default_mission_policy,
)
from navpy.plugins.vehicle.ardupilot.parameter_exchange_loop import ExchangeIo, run_exchange

MissionProgress = Callable[[int, int], None]


class MissionDownloader:
    def __init__(
        self,
        target_system: int,
        transport: MavTransport,
        store: MissionStore,
        messages: MessageStore,
        logger_ref: LoggerRef,
        link: HeartbeatState | None = None,
        cancelled: Callable[[], bool] | None = None,
    ) -> None:
        self._target = target_system
        self._transport = transport
        self._store = store
        self._messages = messages
        self._logger_ref = logger_ref
        self._link = link
        self._cancelled = cancelled

    def download(
        self,
        on_progress: MissionProgress | None = None,
        mission_type: int = MAV_MISSION_TYPE_MISSION,
        policy: ExchangePolicy | None = None,
        *,
        ack_timeout: float = MISSION_ACK_TIMEOUT_S,
        item_timeout: float = MISSION_ITEM_TIMEOUT_S,
    ) -> int:
        """The item count, or 0 when empty or failed (the store is then unchanged)."""
        return self.download_outcome(
            on_progress, mission_type, policy,
            ack_timeout=ack_timeout, item_timeout=item_timeout,
        ).count

    def download_outcome(
        self,
        on_progress: MissionProgress | None = None,
        mission_type: int = MAV_MISSION_TYPE_MISSION,
        policy: ExchangePolicy | None = None,
        *,
        ack_timeout: float = MISSION_ACK_TIMEOUT_S,
        item_timeout: float = MISSION_ITEM_TIMEOUT_S,
    ) -> MissionDownload:
        """Empty, complete, or failed with the store unchanged (:class:`MissionDownloadOutcome`)."""
        logger = self._logger_ref.value
        logger.info("Downloading mission…")
        policy = with_cancel(policy or default_mission_policy(), self._cancelled)
        log = self._log
        exchange, sample = run_exchange(
            policy, self._link, self._messages,
            ExchangeIo(
                "MISSION_REQUEST_LIST",
                lambda: self._transport.call(
                    lambda connection: connection.mav.mission_request_list_send(
                        self._target, 0, mission_type,
                    )
                ),
                lambda cursor, deadline: self._messages.wait_after(
                    "MISSION_COUNT",
                    cursor,
                    lambda message: self._from_target_table(message, mission_type),
                    deadline=deadline,
                ),
            ),
            ack_timeout,
            log=log,
        )
        if sample is None:
            logger.error(f"Giving up – no MISSION_COUNT ({exchange.reason.value}).")
            return MissionDownload(MissionDownloadOutcome.FAILED)
        total = sample.message.count
        if total == 0:
            self._store.replace_items([])
            return MissionDownload(MissionDownloadOutcome.EMPTY)
        staged_items = []
        for sequence in range(total):
            exchange, sample = run_exchange(
                policy, self._link, self._messages,
                ExchangeIo(
                    f"MISSION_REQUEST_INT {sequence}",
                    lambda seq=sequence: self._transport.call(
                        lambda connection: connection.mav.mission_request_int_send(
                            self._target, 0, seq, mission_type,
                        )
                    ),
                    lambda cursor, deadline, seq=sequence: self._messages.wait_after(
                        ("MISSION_ITEM_INT", "MISSION_ITEM"),
                        cursor,
                        lambda message: (
                            self._from_target_table(message, mission_type)
                            and getattr(message, "seq", None) == seq
                        ),
                        deadline=deadline,
                    ),
                ),
                item_timeout,
                log=log,
            )
            if sample is None:
                logger.error(f"Giving up – item {sequence} missing ({exchange.reason.value}).")
                return MissionDownload(MissionDownloadOutcome.FAILED)
            staged_items.append(sample.message)
            if on_progress:
                on_progress(sequence + 1, total)
        self._store.replace_items(staged_items)
        logger.info("Mission download successful.")
        return MissionDownload(MissionDownloadOutcome.COMPLETE, total)

    def download_items(
        self,
        mission_type: int = MAV_MISSION_TYPE_MISSION,
        policy: ExchangePolicy | None = None,
    ) -> list[MAVLink_message]:
        snapshot = self._store.snapshot()
        try:
            count = self.download(mission_type=mission_type, policy=policy)
            return self._store.snapshot() if count > 0 else []
        finally:
            self._store.restore(snapshot)

    def _log(self, line: str) -> None:
        self._logger_ref.value.info(line, "mission")

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
