"""Composition of task participation state around one shared lock."""

from __future__ import annotations

import threading

from navpy.modules.swarm.task_actor_slots import PeerRoster, SelectedTaskSlot
from navpy.modules.swarm.task_auction_lifecycle import TaskAuctionLifecycle
from navpy.modules.swarm.task_auction_models import _TaskAuctionStore
from navpy.modules.swarm.task_auction_state import TaskAuctionState
from navpy.modules.swarm.task_rebroadcast_state import TaskRebroadcastState


def create_task_state(
    lock: threading.RLock,
) -> tuple[SelectedTaskSlot, TaskAuctionState, TaskRebroadcastState]:
    peers = PeerRoster(lock)
    store = _TaskAuctionStore(lock=lock, peers=peers, dispatches={})
    return (
        SelectedTaskSlot(lock),
        TaskAuctionState(store, TaskAuctionLifecycle(store)),
        TaskRebroadcastState(store),
    )


__all__ = ["create_task_state"]
