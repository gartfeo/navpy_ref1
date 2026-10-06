"""Immutable auction plans plus the shared synchronized store."""

from __future__ import annotations

import threading
from collections.abc import Mapping
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
    "TaskAssignmentPlanner",
    "TaskOffer",
    "TaskRebroadcastPlan",
    "TaskRejectOutcome",
    "TaskReservation",
    "UNAVAILABLE_ASSIGNMENT_COST",
]
