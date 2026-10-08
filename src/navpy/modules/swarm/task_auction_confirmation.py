"""Owner side of the acknowledged assign handshake, under the store lock.

Request copies, the fence floor, release on silence, and the verdict on
each response copy follow docs/design/swarm-task-assignment-ack.md.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Optional

from navpy.modules.comm.messages.swarm_ack_msg import (
    ACK_STATUS_APPLIED,
    ACK_STATUS_PROCESSING,
    ACK_STATUS_RECEIVED,
    SwarmAckMsg,
)
from navpy.modules.comm.messages.swarm_heartbeat_msg import SwarmNodeState
from navpy.modules.comm.messages.task_assignment_msg import TaskAssignResponseMsg
from navpy.modules.comm.messages.types import TaskDispatchStatus
from navpy.modules.swarm.task_ack_routing import acked_ref
from navpy.modules.swarm.task_ack_timing import RepeatSchedule
from navpy.modules.swarm.task_auction_evidence import report_peer_state
from navpy.modules.swarm.task_auction_models import (
    AssignConfirmationPorts,
    AssignVerdict,
    ResponseVerdict,
    TaskConfirmationOutcome,
    TaskRejectOutcome,
    TaskReservation,
    _TaskAuctionStore,
)
from navpy.modules.swarm.task_auction_queries import reserved_dispatch
from navpy.modules.swarm.task_dispatch import TaskDispatch
from navpy.modules.swarm.task_msg_refs import MsgRef, msg_ref


_HELD = (TaskDispatchStatus.CONFIRMING, TaskDispatchStatus.CONFIRMED)


class TaskAssignConfirmation:
    """Confirm a reservation only through its helper's fenced response."""

    def __init__(self, store: _TaskAuctionStore) -> None:
        self._store = store

    def arm(
        self,
        reservation: TaskReservation,
        request: MsgRef,
        on_due: Callable[[TaskReservation], None],
        schedule: RepeatSchedule,
    ) -> None:
        """Record the first request copy and wait for the next slot."""
        with self._store.lock:
            dispatch = self._current(reservation)
            if dispatch is None:
                return
            dispatch.attempt.sends = 1
            dispatch.attempt.request_refs.add(request)
            dispatch.start_confirm_timer(
                lambda _task_id: on_due(reservation),
                schedule.delay_after(1),
            )

    def due(
        self,
        reservation: TaskReservation,
        ports: AssignConfirmationPorts,
        schedule: RepeatSchedule,
    ) -> TaskConfirmationOutcome:
        """Send the next request copy, or release an unconfirmed reservation.

        Only the reservation's own generation, peer and attempt are touched,
        so a stale timer cannot affect a reset or newer reservation. Once
        the helper acked a copy, a slot sends nothing. Release drops the
        silent peer's bid like a rejection, counts as a retry, and
        re-advertises the task in the same lock section.
        """
        with self._store.lock:
            generation = self._store.generation
            dispatch = self._current(reservation)
            if dispatch is None:
                return TaskConfirmationOutcome("stale", generation)
            attempt = dispatch.attempt
            if attempt.sends < schedule.copies:
                attempt.sends += 1
                # Arm before sending so a raising send cannot end the chain.
                dispatch.start_confirm_timer(
                    lambda _task_id: ports.on_due(reservation),
                    schedule.delay_after(attempt.sends),
                )
                if attempt.floor is not None:
                    return TaskConfirmationOutcome(
                        "acked", generation, sends=attempt.sends,
                    )
                request = ports.send_request(reservation)
                if request is not None:
                    attempt.request_refs.add(request)
                return TaskConfirmationOutcome(
                    "resent", generation, sends=attempt.sends,
                )
            sends = len(attempt.request_refs)
            dispatch.peer_reject(reservation.peer_id)
            dispatch.answers.released_to = reservation.peer_id
            dispatch.retry_count += 1
            retries = dispatch.retry_count
            if dispatch.retry_count >= dispatch.max_retry_attempts:
                dispatch.retry_count = 0
            ports.advertise([dispatch.task])
            return TaskConfirmationOutcome(
                "released",
                generation,
                sends=sends,
                retry_count=retries,
            )

    def on_request_ack(self, ack: SwarmAckMsg) -> Optional[int]:
        """Set the fence floor from the reserved helper's first ack.

        Returns the task id whose floor was set.
        """
        floor = msg_ref(ack)
        acked = acked_ref(ack)
        if floor is None:
            return None
        with self._store.lock:
            if self._store.closed:
                return None
            for task_id, dispatch in self._store.dispatches.items():
                attempt = dispatch.attempt
                if acked not in attempt.request_refs:
                    continue
                if (
                    dispatch.status is not TaskDispatchStatus.CONFIRMING
                    or dispatch.assigned_peer != ack.sender_id
                    or attempt.floor is not None
                ):
                    return None
                attempt.floor = floor
                return task_id
        return None

    def answer_response(self, message: TaskAssignResponseMsg) -> AssignVerdict:
        """Apply one step-4 copy by the design's verdict table."""
        peer_id = message.sender_id
        response = msg_ref(message)
        with self._store.lock:
            busy_changed = message.is_accepted and report_peer_state(
                self._store, peer_id, SwarmNodeState.BUSY, response,
            )
            dispatch = self._store.dispatches.get(message.task_id)
            if (
                self._store.closed
                or dispatch is None
                or dispatch.status not in _HELD
                or dispatch.assigned_peer != peer_id
                or not self._store.peers.contains(peer_id)
            ):
                return AssignVerdict(
                    ResponseVerdict.NOT_YOURS, ACK_STATUS_RECEIVED,
                    busy_changed=busy_changed,
                )
            if response is None or not response.follows(dispatch.attempt.floor):
                return AssignVerdict(
                    ResponseVerdict.UNFENCED, ACK_STATUS_PROCESSING,
                    busy_changed=busy_changed,
                )
            confirming = dispatch.status is TaskDispatchStatus.CONFIRMING
            if message.is_accepted:
                if confirming:
                    dispatch.peer_accept(peer_id)
                return AssignVerdict(
                    ResponseVerdict.CONFIRMED if confirming
                    else ResponseVerdict.REPEAT,
                    ACK_STATUS_APPLIED,
                    busy_changed=busy_changed,
                )
            if not confirming:
                return AssignVerdict(
                    ResponseVerdict.KEPT, ACK_STATUS_RECEIVED,
                    busy_changed=busy_changed,
                )
            return AssignVerdict(
                ResponseVerdict.REJECTED,
                ACK_STATUS_RECEIVED,
                reject=_reject(dispatch, peer_id, self._store.generation),
                busy_changed=busy_changed,
            )

    def _current(self, reservation: TaskReservation) -> Optional[TaskDispatch]:
        if self._store.closed or reservation.generation != self._store.generation:
            return None
        dispatch = reserved_dispatch(
            self._store,
            reservation.task_id,
            reservation.peer_id,
        )
        if dispatch is None or dispatch.attempt.number != reservation.attempt:
            return None
        return dispatch


def _reject(
    dispatch: TaskDispatch,
    peer_id: int,
    generation: int,
) -> TaskRejectOutcome:
    remaining = dispatch.peer_reject(peer_id)
    dispatch.retry_count += 1
    if remaining > 0 and dispatch.retry_count < dispatch.max_retry_attempts:
        return TaskRejectOutcome(
            "retry",
            generation,
            retry_count=dispatch.retry_count,
            remaining_peers=remaining,
        )
    retries = dispatch.retry_count
    dispatch.retry_count = 0
    return TaskRejectOutcome(
        "exhausted",
        generation,
        task=dispatch.task,
        retry_count=retries,
        remaining_peers=remaining,
    )


__all__ = ["TaskAssignConfirmation"]
