"""Exact-round types and the dedicated outbound transaction fence."""

from __future__ import annotations

import threading
from collections.abc import Callable
from dataclasses import dataclass, field
from enum import Enum, unique
from typing import Optional, TypeVar

from navpy.modules.comm.messages.msg_meta import MsgMeta
from navpy.modules.vision.models.detect_data import DetectedObject


_ResultT = TypeVar("_ResultT")


class ConfirmationOutboundTransaction:
    """Serialize one round's sends with every exact-round resolution."""

    def __init__(self) -> None:
        # A transport can synchronously deliver a response from broadcast().
        # Resolution re-enters this exact transaction on the sending thread.
        self._lock = threading.RLock()

    def run(self, action: Callable[[], _ResultT]) -> _ResultT:
        with self._lock:
            return action()


@dataclass(frozen=True)
class ConfirmationWorkerLease:
    poi_id: Optional[int]
    generation: int
    cancel_event: threading.Event


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
    response_event: threading.Event
    poi: DetectedObject
    worker: ConfirmationWorkerLease
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


__all__ = [
    "ConfirmationOutboundTransaction",
    "ConfirmationRequestRef",
    "ConfirmationResponseKind",
    "ConfirmationRound",
    "ConfirmationWorkerLease",
]
