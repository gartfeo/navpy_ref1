"""Read-only queries over the synchronized task-auction store."""

from __future__ import annotations

from typing import Optional

from navpy.modules.comm.messages.types import TaskDispatchStatus
from navpy.modules.swarm.task_auction_models import _TaskAuctionStore
from navpy.modules.swarm.task_dispatch import TaskDispatch


# Per free peer: its ETA, or None when it declined the task.
BidMatrixSignature = tuple[
    frozenset[int],
    tuple[tuple[int, tuple[tuple[int, Optional[float]], ...]], ...],
]


def busy_peers(store: _TaskAuctionStore) -> set[int]:
    """Peers no plan may use: reserved by this owner or reported BUSY.

    A CONFIRMED peer stays busy only through its own BUSY reports.
    """
    reserved = {
        dispatch.assigned_peer
        for dispatch in store.dispatches.values()
        if dispatch.assigned_peer is not None
        and dispatch.status is TaskDispatchStatus.CONFIRMING
    }
    return reserved | store.peers.busy()


def free_peers(store: _TaskAuctionStore) -> set[int]:
    return store.peers.snapshot() - busy_peers(store)


def answered_peers(dispatch: TaskDispatch) -> set[int]:
    """Peers that bid on or declined the task."""
    return set(dispatch.task_handle_by_peer) | dispatch.answers.declined


def advert_targets(
    store: _TaskAuctionStore,
    dispatch: TaskDispatch,
    busy: set[int],
) -> set[int]:
    """Peers still owed the task's advert.

    A free peer that has not answered, plus a released peer that has not
    answered since its release: its release advert may have been lost
    while it waits for this very task.
    """
    peers = store.peers.snapshot()
    missing = peers - busy - answered_peers(dispatch)
    released = dispatch.answers.released_to
    if released is not None and released in peers:
        missing.add(released)
    return missing


def reserved_dispatch(
    store: _TaskAuctionStore,
    task_id: int,
    peer_id: int,
) -> Optional[TaskDispatch]:
    dispatch = store.dispatches.get(task_id)
    if (
        dispatch is None
        or dispatch.status is not TaskDispatchStatus.CONFIRMING
        or dispatch.assigned_peer != peer_id
        or not store.peers.contains(peer_id)
    ):
        return None
    return dispatch


def complete_bid_matrix(
    store: _TaskAuctionStore,
) -> Optional[BidMatrixSignature]:
    """Return a stable signature once every free peer answered every task."""
    with store.lock:
        if not store.peers.is_full():
            return None
        free = free_peers(store)
        available = [
            (task_id, dispatch)
            for task_id, dispatch in sorted(store.dispatches.items())
            if dispatch.status == TaskDispatchStatus.AVAILABLE
        ]
        if not free or not available:
            return None
        if any(
            not free.issubset(answered_peers(dispatch))
            for _, dispatch in available
        ):
            return None
        return (
            frozenset(free),
            tuple(
                (
                    task_id,
                    tuple(
                        (peer_id, _answer(dispatch, peer_id))
                        for peer_id in sorted(free)
                    ),
                )
                for task_id, dispatch in available
            ),
        )


def _answer(dispatch: TaskDispatch, peer_id: int) -> Optional[float]:
    handle = dispatch.task_handle_by_peer.get(peer_id)
    return None if handle is None else handle.time_in_min


__all__ = [
    "BidMatrixSignature",
    "advert_targets",
    "answered_peers",
    "busy_peers",
    "complete_bid_matrix",
    "free_peers",
    "reserved_dispatch",
]
