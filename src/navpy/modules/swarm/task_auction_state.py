"""Generation-fenced task auction transitions."""

from __future__ import annotations

import math

from typing import Callable, Optional

from navpy.logger.cache_logger import ILogger
from navpy.modules.comm.messages.task_message_data import (
    TaskHandleMsgData,
    TaskMsgData,
)
from navpy.modules.comm.messages.types import TaskDispatchStatus
from navpy.modules.swarm.task_auction_models import (
    TaskAssignmentPlanner,
    TaskRejectOutcome,
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
from navpy.modules.swarm.task_auction_queries import busy_peers as _busy_peers
from navpy.modules.swarm.task_dispatch import TaskDispatch


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

    def record_offer(
        self,
        task_id: int,
        peer_id: int,
        handle: TaskHandleMsgData,
        select: Callable[[int, int], None],
        delay_s: float,
    ) -> bool:
        with self._store.lock:
            if self._store.closed:
                return False
            if not self._store.peers.contains(peer_id):
                return False
            dispatch = self._store.dispatches.get(task_id)
            eta = _normalized_eta(handle.time_in_min)
            if (
                dispatch is None
                or dispatch.status is not TaskDispatchStatus.AVAILABLE
                or eta is None
            ):
                return False
            dispatch.on_peer_available(
                peer_id,
                TaskHandleMsgData(task_id=task_id, time_in_min=eta),
            )
            generation = self._store.generation
            dispatch.start_peer_select_timer(
                lambda selected_id: select(selected_id, generation),
                delay_s,
            )
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

    def accept(self, task_id: int, peer_id: int) -> bool:
        with self._store.lock:
            if self._store.closed:
                return False
            dispatch = _reserved_dispatch(self._store, task_id, peer_id)
            if dispatch is None:
                return False
            dispatch.peer_accept(peer_id)
            return True

    def reject(self, task_id: int, peer_id: int) -> TaskRejectOutcome:
        with self._store.lock:
            if self._store.closed:
                return TaskRejectOutcome("missing", self._store.generation)
            dispatch = _reserved_dispatch(self._store, task_id, peer_id)
            if dispatch is None:
                return TaskRejectOutcome("missing", self._store.generation)
            remaining = dispatch.peer_reject(peer_id)
            dispatch.retry_count += 1
            if remaining > 0 and dispatch.retry_count < dispatch.max_retry_attempts:
                return TaskRejectOutcome(
                    "retry",
                    self._store.generation,
                    retry_count=dispatch.retry_count,
                    remaining_peers=remaining,
                )
            retries = dispatch.retry_count
            dispatch.retry_count = 0
            dispatch.set_status(TaskDispatchStatus.AVAILABLE)
            return TaskRejectOutcome(
                "exhausted",
                self._store.generation,
                task=dispatch.task,
                retry_count=retries,
                remaining_peers=remaining,
            )

    def reset(self, logger: ILogger) -> None:
        self._lifecycle.reset(logger)

    def shutdown(self, logger: ILogger) -> None:
        self._lifecycle.shutdown(logger)


def _reserved_dispatch(
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
