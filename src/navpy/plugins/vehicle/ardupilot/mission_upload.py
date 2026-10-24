"""Fresh-response MAVLink mission upload state machine.

The autopilot drives an upload: it requests each item and re-requests a
missing one (``docs/design/radio_budget.md``, todo 105). ``MISSION_COUNT``
→ the request for item 0 is a bounded exchange; after it every request, of
any seq, is answered, the last item's too while the final ACK is awaited.
Each wait is cut by the policy's deadline, checks the link and
``cancelled`` every ``ack_timeout``, and ends after ``abandon_s`` without a
protocol reply, when ArduPilot has dropped the upload. An item the
autopilot no longer expects is answered ``INVALID_SEQUENCE`` (a late
duplicate), which is ignored; any other non-accepted ACK ends the upload,
also one answering ``MISSION_COUNT``. With ``stop_on_silence`` (a GCS
transfer) every send is logged; otherwise only a missed count and a resent item.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable

from pymavlink.dialects.v20.ardupilotmega import (
    MAV_MISSION_ACCEPTED,
    MAV_MISSION_INVALID_SEQUENCE,
    MAV_MISSION_TYPE_MISSION,
    MAVLink_message,
)

from navpy.core.kernel import runtime_time as rt
from navpy.core.ports.parameter_exchange import ExchangePolicy, ExchangeReason, with_cancel
from navpy.plugins.vehicle.ardupilot.link_state import HeartbeatState
from navpy.plugins.vehicle.ardupilot.logger_ref import LoggerRef
from navpy.plugins.vehicle.ardupilot.mav_transport import MavTransport
from navpy.plugins.vehicle.ardupilot.mission_inbox import MissionEnvelope, MissionInbox
from navpy.plugins.vehicle.ardupilot.mission_store import MissionStore
from navpy.plugins.vehicle.ardupilot.mission_timing import (
    MISSION_ACK_TIMEOUT_S,
    UPLOAD_ABANDON_S,
    default_mission_policy,
)
from navpy.plugins.vehicle.ardupilot.mission_upload_matching import MissionUploadMatcher
from navpy.plugins.vehicle.ardupilot.parameter_exchange_loop import stop_reason

MissionProgress = Callable[[int, int], None]


class MissionUploader:
    def __init__(
        self,
        target_system: int,
        transport: MavTransport,
        store: MissionStore,
        inbox: MissionInbox,
        logger_ref: LoggerRef,
        link: HeartbeatState | None = None,
        cancelled: Callable[[], bool] | None = None,
    ) -> None:
        self._target = target_system
        self._transport = transport
        self._store = store
        self._inbox = inbox
        self._logger_ref = logger_ref
        self._link = link
        self._cancelled = cancelled
        self._matcher = MissionUploadMatcher(target_system)

    def upload(
        self,
        on_wp_sent: MissionProgress | None = None,
        items: Iterable[MAVLink_message] | None = None,
        mission_type: int = MAV_MISSION_TYPE_MISSION,
        policy: ExchangePolicy | None = None,
        *,
        ack_timeout: float = MISSION_ACK_TIMEOUT_S,
        abandon_s: float = UPLOAD_ABANDON_S,
    ) -> bool:
        """True once the autopilot accepted the whole mission."""
        logger = self._logger_ref.value
        logger.info("Uploading mission…")
        policy = with_cancel(policy or default_mission_policy(), self._cancelled)
        waypoints = self._store.snapshot() if items is None else list(items)
        total = len(waypoints)
        if total == 0:
            return True
        envelope, reason = self._first_request(total, mission_type, policy, ack_timeout)
        if envelope is None:
            assert reason is not None
            logger.error(f"Giving up – no MISSION_REQUEST for item 0 ({reason.value}).")
            return False
        next_sequence = 0
        while True:
            message = envelope.message
            if self._matcher.is_ack(message, mission_type):
                kind = getattr(message, "type", None)
                if kind == MAV_MISSION_ACCEPTED and next_sequence >= total:
                    logger.info("Mission upload successful.")
                    return True
                if kind != MAV_MISSION_INVALID_SEQUENCE:
                    logger.warning(f"Mission upload refused (MISSION_ACK type {kind}).")
                    return False
            else:
                sequence = message.seq
                if 0 <= sequence < total:
                    reason = stop_reason(policy, self._link)  # one that came during the wait
                    if reason is not None:
                        logger.warning(f"Mission upload ended before item {sequence} ({reason.value}).")
                        return False
                    self._send_item(waypoints[sequence], sequence, mission_type)
                    if policy.stop_on_silence or sequence < next_sequence:
                        self._log(f"MISSION_ITEM_INT {sequence}/{total} "
                                  f"{'resent' if sequence < next_sequence else 'sent'}")
                    if sequence >= next_sequence:
                        next_sequence = sequence + 1
                        if on_wp_sent:
                            on_wp_sent(next_sequence, total)
                else:
                    logger.warning(f"Ignoring out-of-range request seq={sequence}")
            envelope, reason = self._next_reply(
                envelope.generation, mission_type, policy, ack_timeout, abandon_s,
            )
            if envelope is None:
                assert reason is not None
                waiting = "the final ACK" if next_sequence >= total else f"request {next_sequence}"
                logger.warning(f"Mission upload failed waiting for {waiting} ({reason.value}).")
                return False

    def _first_request(
        self,
        total: int,
        mission_type: int,
        policy: ExchangePolicy,
        ack_timeout: float,
    ) -> tuple[MissionEnvelope | None, ExchangeReason | None]:
        # The cursor precedes the first send, so a late request still counts.
        cursor = self._inbox.cursor()
        sent = 0
        reason = stop_reason(policy, self._link)
        while reason is None and sent < policy.attempts:
            self._transport.call(
                lambda connection: connection.mav.mission_count_send(
                    self._target, 0, total, mission_type,
                )
            )
            sent += 1
            envelope = self._inbox.wait_after(
                cursor,
                lambda message: (
                    self._matcher.is_request(message, 0, mission_type)
                    or self._is_refusal(message, mission_type)
                ),
                deadline=_capped(rt.monotonic() + ack_timeout, policy),
            )
            if policy.stop_on_silence or envelope is None:
                self._log(f"MISSION_COUNT send {sent}/{policy.attempts}: "
                          f"{'no reply' if envelope is None else envelope.message.get_type()}")
            if envelope is not None:
                return envelope, None
            reason = stop_reason(policy, self._link)
        return None, reason or ExchangeReason.NO_ECHO

    def _next_reply(
        self,
        cursor: int,
        mission_type: int,
        policy: ExchangePolicy,
        ack_timeout: float,
        abandon_s: float,
    ) -> tuple[MissionEnvelope | None, ExchangeReason | None]:
        idle_until = rt.monotonic() + abandon_s
        while True:
            reason = stop_reason(policy, self._link)
            if reason is not None:
                return None, reason
            now = rt.monotonic()
            if now >= idle_until:
                return None, ExchangeReason.NO_ECHO
            envelope = self._inbox.wait_after(
                cursor,
                lambda message: (
                    self._matcher.is_request(message, mission_type=mission_type)
                    or self._matcher.is_ack(message, mission_type)
                ),
                deadline=_capped(min(now + ack_timeout, idle_until), policy),
            )
            if envelope is not None:
                return envelope, None

    def _is_refusal(self, message: MAVLink_message, mission_type: int) -> bool:
        """An ACK that ends the upload: neither accepted nor a late duplicate's."""
        return self._matcher.is_ack(message, mission_type) and getattr(message, "type", None) not in (
            MAV_MISSION_ACCEPTED, MAV_MISSION_INVALID_SEQUENCE,
        )

    def _log(self, line: str) -> None:
        self._logger_ref.value.info(line, "mission")

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


def _capped(deadline: float, policy: ExchangePolicy) -> float:
    return deadline if policy.deadline_s is None else min(deadline, policy.deadline_s)


__all__ = ["MissionUploader"]
