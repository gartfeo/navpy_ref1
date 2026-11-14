"""Exact-round types and the dedicated outbound transaction fence."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
from enum import Enum, unique
from typing import Optional, TypeVar

from navpy.core.kernel import runtime_time as rt
from navpy.core.link.msg_meta import MsgMeta
from navpy.core.ports.detect_data import DetectedObject


_ResultT = TypeVar("_ResultT")


class ConfirmationOutboundTransaction:
    """Serialize one round's sends with every exact-round resolution."""

    def __init__(self) -> None:
        # A transport can synchronously deliver a response from broadcast().
        # Resolution re-enters this exact transaction on the sending thread.
        self._lock = rt.RLock()

    def run(self, action: Callable[[], _ResultT]) -> _ResultT:
        with self._lock:
            return action()


class ConfirmationDeadline:
    """One round's response deadline: fixed when built, never extended.

    The round's worker and the response resolver ask this one object, so a
    late worker, a slow image send, a resend or a changed wait parameter
    cannot keep the round open for an approval. Expiry is one-way: a clock
    that steps back does not reopen the round.
    """

    def __init__(self, monotonic: Callable[[], float], wait_s: float) -> None:
        self._monotonic = monotonic
        self._wait_s = wait_s
        start_s = monotonic()
        self._deadline_s = start_s + wait_s
        # "not (a < b)", here and below: a NaN clock or wait counts as expired.
        self._expired = not start_s < self._deadline_s
        # Innermost lock: held only around the clock read and the latch.
        self._lock = rt.Lock()

    @property
    def wait_s(self) -> float:
        return self._wait_s

    def remaining_s(self) -> float:
        """Seconds left to answer; 0.0 from the deadline on."""
        with self._lock:
            if not self._expired:
                now_s = self._monotonic()
                if now_s < self._deadline_s:
                    return self._deadline_s - now_s
                self._expired = True
            return 0.0

    def expired(self) -> bool:
        return not self.remaining_s() > 0.0


@dataclass(frozen=True)
class ConfirmationWorkerLease:
    poi_id: Optional[int]
    generation: int
    cancel_event: rt.Event


@dataclass(frozen=True)
class ConfirmationRequestRef:
    """Wire identity of one confirmation request round."""

    boot_id: int
    msg_seq: int

    @classmethod
    def from_meta(cls, meta: Optional[MsgMeta]) -> Optional["ConfirmationRequestRef"]:
        if meta is None:
            return None
        boot_id = meta.boot_id
        msg_seq = meta.msg_seq
        if type(boot_id) is not int or type(msg_seq) is not int:
            raise ValueError("invalid confirmation request metadata")
        if boot_id == 0 and msg_seq == 0:
            return None
        if not (0 <= boot_id <= 0xFFFFFFFF and 0 <= msg_seq <= 0xFFFFFFFF):
            raise ValueError("invalid confirmation request metadata")
        return cls(boot_id, msg_seq)


@dataclass(frozen=True, eq=False)
class ConfirmationRound:
    poi_id: int
    generation: int
    response_event: rt.Event
    poi: DetectedObject
    worker: ConfirmationWorkerLease
    deadline: ConfirmationDeadline
    request_ref: Optional[ConfirmationRequestRef] = None
    accepts_legacy_response: bool = True
    outbound: ConfirmationOutboundTransaction = field(
        default_factory=ConfirmationOutboundTransaction,
        repr=False,
    )


@unique
class ConfirmationResponseKind(Enum):
    CONFIRMED = "confirmed"
    REJECTED = "rejected"
    CANCELLATION_REQUESTED = "cancellation_requested"
    LATE_OR_DUPLICATE = "late_or_duplicate"
    ALREADY_REJECTED = "already_rejected"
    WAKE_ONLY = "wake_only"
    # An approval when the round's response window is closed (at or after its
    # deadline, or a round with no window): refused, round left registered.
    DEADLINE_PASSED = "deadline_passed"


__all__ = [
    "ConfirmationDeadline",
    "ConfirmationOutboundTransaction",
    "ConfirmationRequestRef",
    "ConfirmationResponseKind",
    "ConfirmationRound",
    "ConfirmationWorkerLease",
]
