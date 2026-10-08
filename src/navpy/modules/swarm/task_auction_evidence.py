"""Owner-side evidence of peer availability, applied under the store lock.

Heartbeats report a peer's state, a bid or decline reports it FREE and an
accepted step-4 copy reports it BUSY; each applies only when newer than the
peer's last evidence (docs/design/swarm-task-assignment-ack.md).
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Optional

from navpy.modules.comm.messages.swarm_heartbeat_msg import SwarmNodeState
from navpy.modules.comm.messages.types import TaskDispatchStatus
from navpy.modules.swarm.task_auction_models import _TaskAuctionStore
from navpy.modules.swarm.task_auction_queries import busy_peers
from navpy.modules.swarm.task_msg_refs import MsgRef
from navpy.modules.swarm.task_peer_roster import sent_after


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
