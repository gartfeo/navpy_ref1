"""Immutable auction plans plus the shared synchronized store."""

from __future__ import annotations

import math
import threading
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Protocol, Sequence

from navpy.modules.comm.messages.task_message_data import TaskMsgData
from navpy.modules.swarm.task_dispatch import TaskDispatch
from navpy.modules.swarm.task_msg_refs import MsgRef
from navpy.modules.swarm.task_peer_roster import PeerRoster


UNAVAILABLE_ASSIGNMENT_COST = 1e9

# The ETA a helper sends for a task it cannot fly ("can't do", e.g. fuel).
DECLINED_ETA_MIN = -1.0


def is_declined_eta(value: object) -> bool:
    """A finite negative ETA is an explicit decline of that one task."""
    return (
        type(value) in {int, float}
        and math.isfinite(value)
        and value < 0
    )


@dataclass(frozen=True)
class TaskOffer:
    task_id: int
    task: TaskMsgData
    eta_by_peer: Mapping[int, Optional[float]]


class TaskAssignmentPlanner(Protocol):
    def plan(
        self,
        offers: Sequence[TaskOffer],
        busy_peers: set[int],
    ) -> list[tuple[int, int]]: ...


@dataclass(frozen=True)
class TaskReservation:
    task_id: int
    peer_id: int
    task: TaskMsgData
    generation: int
    attempt: int


@dataclass(frozen=True)
class TaskRejectOutcome:
    kind: str
    generation: int
    task: Optional[TaskMsgData] = None
    retry_count: int = 0
    remaining_peers: int = 0


class ResponseVerdict(Enum):
    """The owner's verdict on one step-4 copy (design verdict table)."""

    NOT_YOURS = "not_yours"
    UNFENCED = "unfenced"
    CONFIRMED = "confirmed"
    REPEAT = "repeat"
    REJECTED = "rejected"
    KEPT = "kept"


@dataclass(frozen=True)
class AssignVerdict:
    """The verdict, the ack status it answers with, and its side effects."""

    verdict: ResponseVerdict
    ack_status: int
    reject: Optional[TaskRejectOutcome] = None
    busy_changed: bool = False


@dataclass(frozen=True)
class AssignConfirmationPorts:
    """Owner actions a due assign confirmation may take under the lock."""

    send_request: Callable[[TaskReservation], Optional[MsgRef]]
    advertise: Callable[[list[TaskMsgData]], Optional[MsgRef]]
    on_due: Callable[[TaskReservation], None]


@dataclass(frozen=True)
class TaskConfirmationOutcome:
    """``stale``: not current; ``resent``; ``acked`` (no copy needed);
    ``released``."""

    kind: str
    generation: int
    sends: int = 0
    retry_count: int = 0


@dataclass(frozen=True)
class TaskRebroadcastPlan:
    task_id: int
    task: TaskMsgData
    missing_count: int
    generation: int


@dataclass
class _TaskAuctionStore:
    lock: threading.RLock
    peers: PeerRoster
    dispatches: dict[int, TaskDispatch]
    generation: int = 0
    closed: bool = False


__all__ = [
    "AssignConfirmationPorts",
    "AssignVerdict",
    "DECLINED_ETA_MIN",
    "ResponseVerdict",
    "TaskAssignmentPlanner",
    "TaskConfirmationOutcome",
    "TaskOffer",
    "TaskRebroadcastPlan",
    "TaskRejectOutcome",
    "TaskReservation",
    "UNAVAILABLE_ASSIGNMENT_COST",
    "is_declined_eta",
]
