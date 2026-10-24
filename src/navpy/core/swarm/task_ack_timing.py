"""Timers of the acknowledged task-assignment handshake.

Every value derives from the message TTLs in ``ttl_defaults.py`` and the
swarm resend interval, so a TTL change moves the timers with it. Each side
repeats until it has its answer and gives up only after a silence with no
progress from the peer boot it talks to (see
docs/design/swarm-task-assignment-ack.md, "Lossy link"). The timers bound
liveness only; no safety property depends on them.
"""

from __future__ import annotations

from dataclasses import dataclass

from navpy.core.link.ttl_defaults import (
    SWARM_RESEND_INTERVAL_MS,
    get_ttl_ms,
)
from navpy.core.link.types import MsgType


@dataclass(frozen=True)
class HandshakeTiming:
    """Repeat every ``interval_s``; give up after ``silence_s`` without
    progress; remember sent UIDs for ``retention_s`` (their TTL plus their
    ack's), so a late ack still names a known copy."""

    interval_s: float
    silence_s: float
    retention_s: float


@dataclass(frozen=True)
class AssignAckTiming:
    """Owner step-3 repeats and helper step-4 repeats."""

    request: HandshakeTiming
    response: HandshakeTiming


def ack_ttl_ms(acked_type: MsgType) -> int:
    """An ack lives as long as the message it acknowledges.

    A shorter-lived ack could be dropped while its message is still admitted.
    """
    return get_ttl_ms(acked_type)


def assign_ack_timing(
    resend_interval_ms: int = SWARM_RESEND_INTERVAL_MS,
) -> AssignAckTiming:
    """Derive the handshake timers from the current TTL table."""
    interval_s = resend_interval_ms / 1000.0
    # Silence means what it means for a peer: one heartbeat TTL.
    silence_s = get_ttl_ms(MsgType.SWARM_HEARTBEAT) / 1000.0

    def timing(sent: MsgType) -> HandshakeTiming:
        retention_ms = get_ttl_ms(sent) + ack_ttl_ms(sent)
        return HandshakeTiming(interval_s, silence_s, retention_ms / 1000.0)

    return AssignAckTiming(
        request=timing(MsgType.TASK_ASSIGN_REQUEST),
        response=timing(MsgType.TASK_ASSIGN_RESPONSE),
    )


ASSIGN_ACK_TIMING = assign_ack_timing()


__all__ = [
    "ASSIGN_ACK_TIMING",
    "AssignAckTiming",
    "HandshakeTiming",
    "ack_ttl_ms",
    "assign_ack_timing",
]
