"""Owner-side evidence of peer availability, applied under the store lock.

Heartbeats report a peer's state, a bid or decline reports it FREE and an
accepted step-4 copy reports it BUSY; each applies only when newer than the
peer's last evidence (docs/design/swarm-task-assignment-ack.md).
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Optional

from navpy.core.link.swarm_heartbeat_msg import SwarmNodeState
from navpy.core.link.types import TaskDispatchStatus
from navpy.core.swarm.task_auction_models import _TaskAuctionStore
from navpy.core.swarm.task_auction_queries import busy_peers
from navpy.core.swarm.task_dispatch import TaskDispatch
from navpy.core.swarm.task_msg_refs import MsgRef
from navpy.core.swarm.task_peer_roster import sent_after


def report_peer_state(
    store: _TaskAuctionStore,
    peer_id: int,
    state: SwarmNodeState,
    order: Optional[MsgRef],
) -> bool:
    """Apply one report; True when the peer entered or left the busy set.

    A BUSY report withdraws the peer's older bids on AVAILABLE tasks; a FREE
    report ends the extra adverts to a peer released while it waited.
    """
    with store.lock:
        if store.closed or not store.peers.contains(peer_id):
            return False
        was_busy = peer_id in busy_peers(store)
        # Whether or not the roster applies it (newer evidence may already
        # be there): the boot that sent it is alive, and a FREE proves the
        # helper let its task go all the same.
        now_s = store.monotonic_s()
        for dispatch in store.dispatches.values():
            _note_holder_heard(dispatch, peer_id, order, now_s)
            if state is SwarmNodeState.FREE:
                _note_helper_freed(dispatch, peer_id, order)
        if store.peers.report(peer_id, state, order):
            if state is SwarmNodeState.BUSY:
                _withdraw_bids(store, peer_id, order)
            else:
                for dispatch in store.dispatches.values():
                    dispatch.answers.heard_from(peer_id)
        return was_busy != (peer_id in busy_peers(store))


def apply_presence(
    store: _TaskAuctionStore,
    change: Callable[[], None],
) -> bool:
    """Apply a presence change (heard, silent); True if busy set changed."""
    with store.lock:
        if store.closed:
            return False
        busy = busy_peers(store)
        change()
        return busy != busy_peers(store)


def _note_holder_heard(
    dispatch: TaskDispatch,
    peer_id: int,
    report: Optional[MsgRef],
    now_s: float,
) -> None:
    # Only a newer message of the boot that acked the reservation: a late
    # copy, or another boot of the peer, does not keep that boot alive.
    attempt = dispatch.attempt
    if (
        dispatch.assigned_peer == peer_id
        and report is not None
        and attempt.holder_heard is not None
        and report.follows(attempt.holder_heard)
    ):
        attempt.holder_heard = report
        attempt.holder_heard_s = now_s


def _note_helper_freed(
    dispatch: TaskDispatch,
    peer_id: int,
    report: Optional[MsgRef],
) -> None:
    # A helper reports BUSY from its first ack of the request (its slot is
    # no longer EMPTY) and never bids while it holds a task, so a FREE its
    # boot sent after its newest accepted copy means it let the task go
    # (AssignAttempt.helper_freed). Only once CONFIRMED: before, a FREE is
    # a dropped WAITING slot, which a later request copy can undo.
    attempt = dispatch.attempt
    if (
        dispatch.status is TaskDispatchStatus.CONFIRMED
        and dispatch.assigned_peer == peer_id
        and report is not None
        and report.follows(attempt.floor)
        and (attempt.holder_free is None or report.follows(attempt.holder_free))
    ):
        attempt.holder_free = report


def _withdraw_bids(
    store: _TaskAuctionStore,
    peer_id: int,
    report: Optional[MsgRef],
) -> None:
    for dispatch in store.dispatches.values():
        if (
            dispatch.status is not TaskDispatchStatus.AVAILABLE
            or peer_id not in dispatch.task_handle_by_peer
            or sent_after(dispatch.answers.bid_orders.get(peer_id), report)
        ):
            continue
        dispatch.task_handle_by_peer.pop(peer_id)
        dispatch.answers.forget(peer_id)


__all__ = ["apply_presence", "report_peer_state"]
