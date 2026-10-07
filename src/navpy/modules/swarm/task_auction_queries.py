"""Read-only queries over the synchronized task-auction store."""

from __future__ import annotations

from typing import Optional

from navpy.modules.comm.messages.types import TaskDispatchStatus
from navpy.modules.swarm.task_auction_models import _TaskAuctionStore
from navpy.modules.swarm.task_dispatch import TaskDispatch


BidMatrixSignature = tuple[
    frozenset[int],
    tuple[tuple[int, tuple[tuple[int, float], ...]], ...],
]


def busy_peers(dispatches: dict[int, TaskDispatch]) -> set[int]:
    return {
        dispatch.assigned_peer
        for dispatch in dispatches.values()
        if dispatch.assigned_peer is not None
        and dispatch.status in (
            TaskDispatchStatus.CONFIRMING,
            TaskDispatchStatus.CONFIRMED,
        )
    }


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
    """Return a stable signature once every demo peer bid on every free task."""
    with store.lock:
        if not store.peers.is_full():
            return None
        peers = store.peers.snapshot()
        free_peers = peers - busy_peers(store.dispatches)
        available = [
            (task_id, dispatch)
            for task_id, dispatch in sorted(store.dispatches.items())
            if dispatch.status == TaskDispatchStatus.AVAILABLE
        ]
        if not free_peers or not available:
            return None
        if any(
            not free_peers.issubset(dispatch.task_handle_by_peer)
            for _, dispatch in available
        ):
            return None
        return (
            frozenset(free_peers),
            tuple(
                (
                    task_id,
                    tuple(
                        (
                            peer_id,
                            dispatch.task_handle_by_peer[peer_id].time_in_min,
                        )
                        for peer_id in sorted(free_peers)
                    ),
                )
                for task_id, dispatch in available
            ),
        )


__all__ = [
    "BidMatrixSignature",
    "busy_peers",
    "complete_bid_matrix",
    "reserved_dispatch",
]
