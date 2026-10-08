"""Generation-fenced task auction transitions."""

from __future__ import annotations

import math

from typing import Callable, Optional

from navpy.logger.cache_logger import ILogger
from navpy.modules.comm.messages.swarm_heartbeat_msg import SwarmNodeState
from navpy.modules.comm.messages.task_message_data import (
    TaskHandleMsgData,
    TaskMsgData,
)
from navpy.modules.comm.messages.types import TaskDispatchStatus
from navpy.modules.swarm.task_auction_evidence import report_peer_state
from navpy.modules.swarm.task_auction_models import (
    TaskAssignmentPlanner,
    TaskReservation,
    UNAVAILABLE_ASSIGNMENT_COST,
    _TaskAuctionStore,
)
from navpy.modules.swarm.task_auction_lifecycle import TaskAuctionLifecycle
from navpy.modules.swarm.task_auction_planning import (
    plan_and_reserve as reserve_auction_plan,
    plan_reserve_and_send_if_complete,
    plan_retry_reserve_and_send,
)
from navpy.modules.swarm.task_dispatch import TaskDispatch
from navpy.modules.swarm.task_msg_refs import MsgRef


class TaskAuctionState:
    """Own dispatches and perform auction transitions under one lock."""

    def __init__(
        self,
        store: _TaskAuctionStore,
        lifecycle: TaskAuctionLifecycle,
        dispatch_factory: Callable[[TaskMsgData], TaskDispatch] = TaskDispatch,
    ) -> None:
        self._store = store
        self._lifecycle = lifecycle
        self._dispatch_factory = dispatch_factory

    def register(self, task: TaskMsgData) -> Optional[tuple[TaskDispatch, int]]:
        return self.register_dispatch(self._dispatch_factory(task))

    def register_dispatch(
        self,
        dispatch: TaskDispatch,
    ) -> Optional[tuple[TaskDispatch, int]]:
        with self._store.lock:
            if self._store.closed:
                return None
            task_id = dispatch.task.task_id
            if task_id in self._store.dispatches:
                return None
            self._store.dispatches[task_id] = dispatch
            return dispatch, self._store.generation

    def lookup(self, task_id: int) -> Optional[TaskDispatch]:
        with self._store.lock:
            return self._store.dispatches.get(task_id)

    def current_generation(self) -> int:
        with self._store.lock:
            return self._store.generation

    def task_ids(self) -> set[int]:
        with self._store.lock:
            return set(self._store.dispatches)

    def admits_peer(self, peer_id: int) -> bool:
        return self._store.peers.contains(peer_id)

    def observe_peer(
        self,
        peer_id: int,
        state: SwarmNodeState,
        order: Optional[MsgRef],
    ) -> bool:
        """Apply a peer's report; True when the busy set changed."""
        return report_peer_state(self._store, peer_id, state, order)

    def record_offer(
        self,
        task_id: int,
        peer_id: int,
        handle: TaskHandleMsgData,
        select: Callable[[int, int], None],
        delay_s: float,
        *,
        order: Optional[MsgRef] = None,
    ) -> bool:
        """Record a bid; a bid sent before the peer's BUSY report is stale."""
        with self._store.lock:
            dispatch = _answerable(self._store, task_id, peer_id, order)
            eta = _normalized_eta(handle.time_in_min)
            if dispatch is None or eta is None:
                return False
            dispatch.on_peer_available(
                peer_id,
                TaskHandleMsgData(task_id=task_id, time_in_min=eta),
                order,
            )
            generation = self._store.generation
            dispatch.start_peer_select_timer(
                lambda selected_id: select(selected_id, generation),
                delay_s,
            )
            return True

    def record_decline(
        self,
        task_id: int,
        peer_id: int,
        *,
        order: Optional[MsgRef] = None,
    ) -> bool:
        """Leave a peer that cannot fly this task out of its matrix."""
        with self._store.lock:
            dispatch = _answerable(self._store, task_id, peer_id, order)
            if dispatch is None:
                return False
            dispatch.on_peer_declined(peer_id)
            return True

    def plan_and_reserve(
        self,
        planner: TaskAssignmentPlanner,
        generation: int,
    ) -> list[TaskReservation]:
        reservations = reserve_auction_plan(
            self._store,
            planner,
            generation,
            require_complete_matrix=False,
        )
        return reservations or []

    def plan_reserve_and_send_if_complete(
        self,
        planner: TaskAssignmentPlanner,
        generation: int,
        send: Callable[[TaskReservation], bool],
    ) -> Optional[bool]:
        return plan_reserve_and_send_if_complete(
            self._store,
            planner,
            generation,
            send,
        )

    def plan_retry_reserve_and_send(
        self,
        planner: TaskAssignmentPlanner,
        generation: int,
        task_id: int,
        send: Callable[[TaskReservation], bool],
    ) -> Optional[bool]:
        return plan_retry_reserve_and_send(
            self._store,
            planner,
            generation,
            task_id,
            send,
        )

    def reset(self, logger: ILogger) -> None:
        self._lifecycle.reset(logger)

    def shutdown(self, logger: ILogger) -> None:
        self._lifecycle.shutdown(logger)


def _answerable(
    store: _TaskAuctionStore,
    task_id: int,
    peer_id: int,
    order: Optional[MsgRef],
) -> Optional[TaskDispatch]:
    """The AVAILABLE task an admitted peer's current answer may update."""
    if store.closed or not store.peers.contains(peer_id):
        return None
    dispatch = store.dispatches.get(task_id)
    if (
        dispatch is None
        or dispatch.status is not TaskDispatchStatus.AVAILABLE
        or not store.peers.bid_is_current(peer_id, order)
    ):
        return None
    return dispatch


def _normalized_eta(value: object) -> Optional[float]:
    if type(value) not in {int, float}:
        return None
    try:
        normalized = float(value)
    except (OverflowError, ValueError):
        return None
    if (
        not math.isfinite(normalized)
        or normalized < 0.0
        or normalized >= UNAVAILABLE_ASSIGNMENT_COST
    ):
        return None
    return normalized


__all__ = ["TaskAuctionState"]
