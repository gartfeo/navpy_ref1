"""Immutable auction plans plus the shared synchronized store."""

from __future__ import annotations

import threading
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from typing import Optional, Protocol, Sequence

from navpy.modules.comm.messages.task_message_data import TaskMsgData
from navpy.modules.swarm.task_actor_slots import PeerRoster
from navpy.modules.swarm.task_dispatch import TaskDispatch


UNAVAILABLE_ASSIGNMENT_COST = 1e9


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


@dataclass(frozen=True)
class TaskRejectOutcome:
    kind: str
    generation: int
    task: Optional[TaskMsgData] = None
    retry_count: int = 0
    remaining_peers: int = 0


@dataclass(frozen=True)
class AssignConfirmationPolicy:
    """How long a CONFIRMING reservation waits for its assign response.

    The request is sent up to ``max_sends`` times, ``resend_interval_s``
    apart, so one lost request or response does not strand the task. After
    the last send the owner waits ``release_delay_s`` before releasing the
    reservation; set it to the request TTL so no copy of the request is
    still accepted by the peer once the task is released.
    """

    resend_interval_s: float
    max_sends: int
    release_delay_s: float

    def delay_after(self, sends: int) -> float:
        if sends < self.max_sends:
            return self.resend_interval_s
        return self.release_delay_s


@dataclass(frozen=True)
class AssignConfirmationPorts:
    """Owner actions a due assign confirmation may take under the lock."""

    send_request: Callable[[TaskReservation], bool]
    advertise: Callable[[list[TaskMsgData]], bool]
    on_due: Callable[[TaskReservation], None]


@dataclass(frozen=True)
class TaskConfirmationOutcome:
    """``stale``: reservation no longer current; ``resent``; ``released``."""

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
    "AssignConfirmationPolicy",
    "AssignConfirmationPorts",
    "TaskAssignmentPlanner",
    "TaskConfirmationOutcome",
    "TaskOffer",
    "TaskRebroadcastPlan",
    "TaskRejectOutcome",
    "TaskReservation",
    "UNAVAILABLE_ASSIGNMENT_COST",
]
