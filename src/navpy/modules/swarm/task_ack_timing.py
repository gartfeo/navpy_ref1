"""Timers of the acknowledged task-assignment handshake.

Every value derives from the message TTLs in ``ttl_defaults.py`` and the
swarm resend interval, so a TTL change moves the timers with it. The timers
bound liveness only; no safety property depends on them (see
docs/design/swarm-task-assignment-ack.md, "Timing").
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from navpy.modules.comm.messages.ttl_defaults import (
    SWARM_RESEND_INTERVAL_MS,
    get_ttl_ms,
)
from navpy.modules.comm.messages.types import MsgType


@dataclass(frozen=True)
class RepeatSchedule:
    """``copies`` sends ``interval_s`` apart, then one deadline tick.

    ``deadline_s`` counts from the first send.
    """

    interval_s: float
    copies: int
    deadline_s: float

    def delay_after(self, sent: int) -> float:
        """Delay from send number ``sent`` (1-based) to the next tick."""
        if sent < self.copies:
            return self.interval_s
        return self.deadline_s - (self.copies - 1) * self.interval_s


@dataclass(frozen=True)
class AssignAckTiming:
    """Owner step-3 copies then release; helper step-4 copies then expiry."""

    request: RepeatSchedule
    response: RepeatSchedule


def ack_ttl_ms(acked_type: MsgType) -> int:
    """An ack lives as long as the message it acknowledges.

    A shorter-lived ack could be dropped while its message is still admitted.
    """
    return get_ttl_ms(acked_type)


def assign_ack_timing(
    resend_interval_ms: int = SWARM_RESEND_INTERVAL_MS,
) -> AssignAckTiming:
    """Derive the handshake timers from the current TTL table."""
    request_ttl_ms = get_ttl_ms(MsgType.TASK_ASSIGN_REQUEST)
    # The owner's copies spread over one request TTL; it releases once the
    # last copy can no longer be admitted by the helper.
    request_copies = math.ceil(request_ttl_ms / resend_interval_ms)
    release_ms = (request_copies - 1) * resend_interval_ms + request_ttl_ms
    # The helper answers while the owner may still confirm: every copy goes
    # out before the release.
    response_copies = math.ceil(release_ms / resend_interval_ms)
    # It gives up once its last copy and the owner's ack of it expired.
    expiry_ms = (
        (response_copies - 1) * resend_interval_ms
        + get_ttl_ms(MsgType.TASK_ASSIGN_RESPONSE)
        + ack_ttl_ms(MsgType.TASK_ASSIGN_RESPONSE)
    )
    interval_s = resend_interval_ms / 1000.0
    return AssignAckTiming(
        request=RepeatSchedule(interval_s, request_copies, release_ms / 1000.0),
        response=RepeatSchedule(interval_s, response_copies, expiry_ms / 1000.0),
    )


ASSIGN_ACK_TIMING = assign_ack_timing()


__all__ = [
    "ASSIGN_ACK_TIMING",
    "AssignAckTiming",
    "RepeatSchedule",
    "ack_ttl_ms",
    "assign_ack_timing",
]
