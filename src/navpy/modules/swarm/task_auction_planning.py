"""Snapshot, plan, and atomically reserve owner-side task assignments."""

from __future__ import annotations

from collections.abc import Callable
from types import MappingProxyType
from typing import Optional

from navpy.modules.comm.messages.types import TaskDispatchStatus
from navpy.modules.swarm.task_auction_models import (
    TaskAssignmentPlanner,
    TaskOffer,
    TaskReservation,
    _TaskAuctionStore,
)
from navpy.modules.swarm.task_auction_queries import (
    BidMatrixSignature,
    busy_peers,
    complete_bid_matrix,
    free_peers,
)


def plan_and_reserve(
    store: _TaskAuctionStore,
    planner: TaskAssignmentPlanner,
    generation: int,
    *,
    require_complete_matrix: bool,
) -> Optional[list[TaskReservation]]:
    """Plan outside the lock, then reserve only an unchanged valid snapshot."""
    with store.lock:
        if store.closed or generation != store.generation:
            return None if require_complete_matrix else []
        matrix = _required_matrix(store, require_complete_matrix)
        if require_complete_matrix and matrix is None:
            return None
        busy = busy_peers(store)
        free = store.peers.snapshot() - busy
        offers = _available_offers(store)

    offered_eta = {
        (offer.task_id, peer_id): eta
        for offer in offers
        for peer_id, eta in offer.eta_by_peer.items()
    }
    planned = planner.plan(tuple(offers), set(busy))

    with store.lock:
        if store.closed or generation != store.generation:
            return None if require_complete_matrix else []
        if (
            require_complete_matrix and complete_bid_matrix(store) != matrix
        ) or free_peers(store) != free:
            return None if require_complete_matrix else []
        return _reserve_valid_plan(store, planned, offered_eta, generation)


def plan_reserve_and_send_if_complete(
    store: _TaskAuctionStore,
    planner: TaskAssignmentPlanner,
    generation: int,
    send: Callable[[TaskReservation], bool],
) -> Optional[bool]:
    """Send a complete auction before reset can invalidate its reservations."""
    return _plan_reserve_and_send(
        store,
        planner,
        generation,
        send,
        require_complete_matrix=True,
    )


def plan_retry_reserve_and_send(
    store: _TaskAuctionStore,
    planner: TaskAssignmentPlanner,
    generation: int,
    task_id: int,
    send: Callable[[TaskReservation], bool],
) -> Optional[bool]:
    """Retry one rejected task from its remaining valid bids."""
    return _plan_reserve_and_send(
        store,
        planner,
        generation,
        send,
        require_complete_matrix=False,
        task_ids=frozenset({task_id}),
    )


def _plan_reserve_and_send(
    store: _TaskAuctionStore,
    planner: TaskAssignmentPlanner,
    generation: int,
    send: Callable[[TaskReservation], bool],
    *,
    require_complete_matrix: bool,
    task_ids: frozenset[int] | None = None,
) -> Optional[bool]:
    with store.lock:
        if store.closed or generation != store.generation:
            return None
        matrix = _required_matrix(store, require_complete_matrix)
        if require_complete_matrix and matrix is None:
            return None
        busy = busy_peers(store)
        free = store.peers.snapshot() - busy
        offers = _available_offers(store, task_ids)

    offered_eta = {
        (offer.task_id, peer_id): eta
        for offer in offers
        for peer_id, eta in offer.eta_by_peer.items()
    }
    planned = planner.plan(tuple(offers), set(busy))

    with store.lock:
        # A plan whose free set changed while planning is aborted.
        if (
            store.closed
            or generation != store.generation
            or (
                require_complete_matrix
                and complete_bid_matrix(store) != matrix
            )
            or free_peers(store) != free
        ):
            return None
        reservations = _reserve_valid_plan(
            store,
            planned,
            offered_eta,
            generation,
        )
        if not reservations:
            return False
        all_sent = True
        for index, reservation in enumerate(reservations):
            dispatch = store.dispatches.get(reservation.task_id)
            try:
                sent = send(reservation)
            except BaseException:
                _release_reservations(store, reservations[index:])
                raise
            if sent:
                continue
            all_sent = False
            if dispatch is not None:
                _release_reservation(dispatch, reservation.peer_id)
        return all_sent


def _required_matrix(
    store: _TaskAuctionStore,
    required: bool,
) -> Optional[BidMatrixSignature]:
    return complete_bid_matrix(store) if required else None


def _available_offers(
    store: _TaskAuctionStore,
    task_ids: frozenset[int] | None = None,
) -> list[TaskOffer]:
    return [
        TaskOffer(
            task_id=task_id,
            task=dispatch.task,
            eta_by_peer=MappingProxyType({
                peer_id: handle.time_in_min
                for peer_id, handle in dispatch.task_handle_by_peer.items()
            }),
        )
        for task_id, dispatch in store.dispatches.items()
        if task_ids is None or task_id in task_ids
        if dispatch.status == TaskDispatchStatus.AVAILABLE
        and dispatch.task_handle_by_peer
    ]


def _reserve_valid_plan(
    store: _TaskAuctionStore,
    planned: list[tuple[int, int]],
    offered_eta: dict[tuple[int, int], Optional[float]],
    generation: int,
) -> list[TaskReservation]:
    reservations: list[TaskReservation] = []
    for task_id, peer_id in planned:
        dispatch = store.dispatches.get(task_id)
        handle = (
            dispatch.task_handle_by_peer.get(peer_id)
            if dispatch is not None
            else None
        )
        if (
            dispatch is None
            or dispatch.status != TaskDispatchStatus.AVAILABLE
            or handle is None
            or not store.peers.contains(peer_id)
            or (task_id, peer_id) not in offered_eta
            or offered_eta[(task_id, peer_id)] != handle.time_in_min
            or peer_id in busy_peers(store)
        ):
            continue
        dispatch.cancel_peer_select_timer()
        dispatch.assigned_peer = peer_id
        dispatch.set_status(TaskDispatchStatus.CONFIRMING)
        reservations.append(TaskReservation(
            task_id=task_id,
            peer_id=peer_id,
            task=dispatch.task,
            generation=generation,
            attempt=dispatch.begin_attempt(),
        ))
    return reservations


def _release_reservations(
    store: _TaskAuctionStore,
    reservations: list[TaskReservation],
) -> None:
    for reservation in reservations:
        dispatch = store.dispatches.get(reservation.task_id)
        if dispatch is not None:
            _release_reservation(dispatch, reservation.peer_id)


def _release_reservation(dispatch: TaskDispatch, peer_id: int) -> None:
    if (
        dispatch.status is TaskDispatchStatus.CONFIRMING
        and dispatch.assigned_peer == peer_id
    ):
        dispatch.assigned_peer = None
        dispatch.set_status(TaskDispatchStatus.AVAILABLE)


__all__ = [
    "plan_and_reserve",
    "plan_reserve_and_send_if_complete",
    "plan_retry_reserve_and_send",
]
