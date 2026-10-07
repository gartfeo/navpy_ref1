"""Composition of task participation state around one shared lock."""

from __future__ import annotations

import threading
from typing import NamedTuple

from navpy.modules.swarm.task_actor_slots import PeerRoster, SelectedTaskSlot
from navpy.modules.swarm.task_auction_confirmation import (
    TaskAssignConfirmation,
)
from navpy.modules.swarm.task_auction_lifecycle import TaskAuctionLifecycle
from navpy.modules.swarm.task_auction_models import _TaskAuctionStore
from navpy.modules.swarm.task_auction_state import TaskAuctionState
from navpy.modules.swarm.task_rebroadcast_state import TaskRebroadcastState


class SwarmTaskState(NamedTuple):
    selection: SelectedTaskSlot
    auction: TaskAuctionState
    rebroadcast: TaskRebroadcastState
    confirmation: TaskAssignConfirmation


def create_swarm_task_state(lock: threading.RLock) -> SwarmTaskState:
    peers = PeerRoster(lock)
    store = _TaskAuctionStore(lock=lock, peers=peers, dispatches={})
    return SwarmTaskState(
        SelectedTaskSlot(lock),
        TaskAuctionState(store, TaskAuctionLifecycle(store)),
        TaskRebroadcastState(store),
        TaskAssignConfirmation(store),
    )


def create_task_state(
    lock: threading.RLock,
) -> tuple[SelectedTaskSlot, TaskAuctionState, TaskRebroadcastState]:
    state = create_swarm_task_state(lock)
    return state.selection, state.auction, state.rebroadcast


__all__ = ["SwarmTaskState", "create_swarm_task_state", "create_task_state"]
