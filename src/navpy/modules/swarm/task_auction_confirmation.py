"""Generation-fenced resend and release of unanswered assign requests."""

from __future__ import annotations

from collections.abc import Callable
from typing import Optional

from navpy.modules.swarm.task_auction_models import (
    AssignConfirmationPolicy,
    AssignConfirmationPorts,
    TaskConfirmationOutcome,
    TaskReservation,
    _TaskAuctionStore,
)
from navpy.modules.swarm.task_auction_queries import reserved_dispatch
from navpy.modules.swarm.task_dispatch import TaskDispatch


class TaskAssignConfirmation:
    """Keep a CONFIRMING reservation from waiting forever on a lost message."""

    def __init__(self, store: _TaskAuctionStore) -> None:
        self._store = store

    def arm(
        self,
        reservation: TaskReservation,
        on_due: Callable[[TaskReservation], None],
        policy: AssignConfirmationPolicy,
    ) -> None:
        """Start waiting for the response to the first assign request."""
        with self._store.lock:
            dispatch = self._current(reservation)
            if dispatch is None:
                return
            dispatch.assign_request_sends = 1
            dispatch.start_confirm_timer(
                lambda _task_id: on_due(reservation),
                policy.delay_after(1),
            )

    def due(
        self,
        reservation: TaskReservation,
        ports: AssignConfirmationPorts,
        policy: AssignConfirmationPolicy,
    ) -> TaskConfirmationOutcome:
        """Resend an unanswered assign request, or release the reservation.

        Only the reservation's own generation and peer are touched, so a
        stale timer cannot affect a reset or reassigned task. Release drops
        the silent peer's bid like a rejection and counts as a retry. The
        task returns to AVAILABLE and is re-advertised in the same lock
        section, before any later assign request can be sent; it is
        reassigned only once every free peer has bid again, and a peer
        still holding it bids only after that re-advertisement released it.
        """
        with self._store.lock:
            generation = self._store.generation
            dispatch = self._current(reservation)
            if dispatch is None:
                return TaskConfirmationOutcome("stale", generation)
            if dispatch.assign_request_sends < policy.max_sends:
                dispatch.assign_request_sends += 1
                sends = dispatch.assign_request_sends
                # Arm before sending so a raising send cannot end the chain.
                dispatch.start_confirm_timer(
                    lambda _task_id: ports.on_due(reservation),
                    policy.delay_after(sends),
                )
                ports.send_request(reservation)
                return TaskConfirmationOutcome("resent", generation, sends=sends)
            sends = dispatch.assign_request_sends
            dispatch.peer_reject(reservation.peer_id)
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

    def _current(self, reservation: TaskReservation) -> Optional[TaskDispatch]:
        if self._store.closed or reservation.generation != self._store.generation:
            return None
        return reserved_dispatch(
            self._store,
            reservation.task_id,
            reservation.peer_id,
        )


__all__ = ["TaskAssignConfirmation"]
