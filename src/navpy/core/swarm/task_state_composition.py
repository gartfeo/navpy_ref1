"""Composition of task participation state around one shared lock."""

from __future__ import annotations

from collections.abc import Callable
from typing import NamedTuple

from navpy.core.kernel import runtime_time as rt
from navpy.core.swarm.task_actor_slots import SelectedTaskSlot
from navpy.core.swarm.task_auction_confirmation import (
    TaskAssignConfirmation,
)
from navpy.core.swarm.task_auction_lifecycle import TaskAuctionLifecycle
from navpy.core.swarm.task_auction_models import _TaskAuctionStore
from navpy.core.swarm.task_auction_state import TaskAuctionState
from navpy.core.swarm.task_peer_roster import PeerRoster
from navpy.core.swarm.task_rebroadcast_state import TaskRebroadcastState


class SwarmTaskState(NamedTuple):
    selection: SelectedTaskSlot
    auction: TaskAuctionState
    rebroadcast: TaskRebroadcastState
    confirmation: TaskAssignConfirmation


def create_task_state(
    lock: rt.RLock,
    monotonic_s: Callable[[], float] = rt.monotonic,
) -> SwarmTaskState:
    peers = PeerRoster(lock, monotonic_s)
    store = _TaskAuctionStore(lock=lock, peers=peers, dispatches={})
    return SwarmTaskState(
        SelectedTaskSlot(lock, monotonic_s),
        TaskAuctionState(store, TaskAuctionLifecycle(store)),
        TaskRebroadcastState(store),
        TaskAssignConfirmation(store, monotonic_s),
    )


__all__ = ["SwarmTaskState", "create_task_state"]
