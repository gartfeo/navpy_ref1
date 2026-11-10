"""Owner side: what happens to a CONFIRMED task whose helper is lost.

The helper that holds a task is one boot of the peer: the boot of its first
ack of the request (the attempt's floor). That boot is lost when it checks
out after its newest accepted step-4 copy (after that ack when it accepted
none), or when no newer message of it arrived for one
heartbeat TTL (PEER_SILENCE_EXPIRY_S): the peer is silent, or it restarted
and only a new boot (which holds no task) talks. Late copies and other
boots never keep it alive, and no boot order is needed (boot ids are
random). The owner's policy decides (owner ruling 2026-11-09, todo 115):

- ``keep`` (default): design decision 4, a confirmed task is never taken
  back; the task stays with the lost helper and the loss is logged once.
- ``auto``: the task returns to the auction at once, and the free peers bid
  on it again. A helper that is only cut off, not gone, may still fly it.
  On a check-out, a reservation still CONFIRMING to that boot (acked, not
  answered) is released too; a silent one is left to the confirm timers.

A helper whose boot reported FREE after its newest accepted step-4 copy no
longer holds the task (flown or dropped), so its task is never taken back
(AssignAttempt.helper_freed).
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional

from navpy.core.link.types import TaskDispatchStatus
from navpy.core.swarm.task_auction_models import _TaskAuctionStore
from navpy.core.swarm.task_dispatch import AssignAttempt, PeerAnswers, TaskDispatch
from navpy.core.swarm.task_msg_refs import MsgRef
from navpy.core.swarm.task_peer_roster import PEER_SILENCE_EXPIRY_S


class LostHelperPolicy(Enum):
    KEEP = "keep"
    AUTO = "auto"


#: Logged per task when its helper is lost, by policy.
LOST_HELPER_RELEASED_LOG = (
    "Task {task}: helper {peer} lost ({reason}); taken back and "
    "re-auctioned (lost-helper policy auto)."
)
LOST_HELPER_KEPT_LOG = (
    "Task {task}: helper {peer} lost ({reason}); it stays with that helper "
    "(lost-helper policy keep)."
)


@dataclass(frozen=True)
class LostTask:
    """One task whose helper boot was lost, and whether it was released."""

    task_id: int
    peer_id: int
    reason: str
    released: bool


class LostHelpers:
    """Apply the lost-helper policy to the owner's dispatches."""

    def __init__(self, store: _TaskAuctionStore) -> None:
        self._store = store

    def checked_out(
        self,
        peer_id: int,
        policy: LostHelperPolicy,
        checkout: MsgRef,
    ) -> list[LostTask]:
        """The peer's boot that sent ``checkout`` left: its tasks acked and
        accepted before it are lost (a late check-out of an older boot, or
        of an earlier actor of this boot, spares a newer acceptance)."""
        auto = policy is LostHelperPolicy.AUTO
        with self._store.lock:
            return self._apply(policy, [
                (dispatch, "checked out")
                for dispatch in self._held(confirming=auto)
                if dispatch.assigned_peer == peer_id
                and checkout.follows(
                    dispatch.attempt.holder_accept or dispatch.attempt.floor
                )
            ])

    def expired(self, policy: LostHelperPolicy) -> list[LostTask]:
        """Tasks whose holder boot was not heard for one heartbeat TTL."""
        with self._store.lock:
            now_s = self._store.monotonic_s()
            return self._apply(policy, [
                (dispatch, self._silence_reason(dispatch))
                for dispatch in self._held(confirming=False)
                if now_s - dispatch.attempt.holder_heard_s
                > PEER_SILENCE_EXPIRY_S
            ])

    def _held(self, *, confirming: bool) -> list[TaskDispatch]:
        held = (TaskDispatchStatus.CONFIRMED, TaskDispatchStatus.CONFIRMING)
        if self._store.closed:
            return []
        return [
            dispatch
            for dispatch in self._store.dispatches.values()
            if dispatch.status in (held if confirming else held[:1])
            and dispatch.assigned_peer is not None
            # No floor: no boot acked the reservation, so none holds it.
            and dispatch.attempt.floor is not None
            and not dispatch.attempt.helper_freed()
        ]

    def _silence_reason(self, dispatch: TaskDispatch) -> str:
        status = self._store.peers.status(dispatch.assigned_peer)
        current: Optional[MsgRef] = None if status is None else status.order
        floor = dispatch.attempt.floor
        if current is not None and floor is not None and current.owner != floor.owner:
            return "restarted"
        return "silent"

    def _apply(
        self,
        policy: LostHelperPolicy,
        lost: list[tuple[TaskDispatch, str]],
    ) -> list[LostTask]:
        tasks = []
        for dispatch, reason in lost:
            peer_id = dispatch.assigned_peer
            if policy is LostHelperPolicy.AUTO:
                _release(dispatch)
            elif dispatch.attempt.loss_logged:
                continue
            else:
                dispatch.attempt.loss_logged = True
            tasks.append(LostTask(
                dispatch.task.task_id, peer_id, reason,
                policy is LostHelperPolicy.AUTO,
            ))
        return tasks


def _release(dispatch: TaskDispatch) -> None:
    # A fresh auction: every bid and answer is dropped, so the free peers
    # are advertised to and bid again with current ETAs, and a new attempt
    # fences the lost helper's late copies off. Under the store lock.
    dispatch.cancel_peer_select_timer()
    dispatch.cancel_confirm_timer()
    dispatch.task_handle_by_peer.clear()
    dispatch.answers = PeerAnswers()
    dispatch.assigned_peer = None
    dispatch.retry_count = 0
    dispatch.attempt = AssignAttempt(dispatch.attempt.number + 1)
    dispatch.status = TaskDispatchStatus.AVAILABLE


__all__ = [
    "LOST_HELPER_KEPT_LOG",
    "LOST_HELPER_RELEASED_LOG",
    "LostHelperPolicy",
    "LostHelpers",
    "LostTask",
]
