"""Admitted swarm peers and the newest evidence of their availability."""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Optional

from navpy.modules.comm.messages.swarm_heartbeat_msg import SwarmNodeState
from navpy.modules.swarm.task_msg_refs import MsgRef


MAX_REMOTE_PEERS = 2


@dataclass
class PeerStatus:
    """What one peer last reported about itself, as the owner sees it.

    ``order`` is the UID of the evidence applied last and ``busy_order``
    that of the newest BUSY evidence, which makes the peer's older bids
    stale.
    """

    state: SwarmNodeState = SwarmNodeState.FREE
    order: Optional[MsgRef] = None
    busy_order: Optional[MsgRef] = None


def sent_after(message: Optional[MsgRef], report: Optional[MsgRef]) -> bool:
    """True when ``message`` provably followed ``report`` (or no report)."""
    if report is None:
        return True
    return message is not None and message.follows(report)


def _supersedes(evidence: Optional[MsgRef], current: Optional[MsgRef]) -> bool:
    # Another boot replaces the record; evidence without a UID never
    # overrides ordered evidence.
    if current is None:
        return True
    if evidence is None:
        return False
    return evidence.owner != current.owner or evidence.msg_seq > current.msg_seq


class PeerRoster:
    """Own the swarm peers discovered from heartbeats and their status."""

    def __init__(self, lock: threading.RLock) -> None:
        self._lock = lock
        self._peers: dict[int, PeerStatus] = {}

    def add(self, peer_id: int) -> bool:
        with self._lock:
            if peer_id in self._peers or len(self._peers) >= MAX_REMOTE_PEERS:
                return False
            self._peers[peer_id] = PeerStatus()
            return True

    def contains(self, peer_id: int) -> bool:
        with self._lock:
            return peer_id in self._peers

    def snapshot(self) -> set[int]:
        with self._lock:
            return set(self._peers)

    def is_full(self) -> bool:
        with self._lock:
            return len(self._peers) >= MAX_REMOTE_PEERS

    def clear(self) -> None:
        with self._lock:
            self._peers.clear()

    def report(
        self,
        peer_id: int,
        state: SwarmNodeState,
        order: Optional[MsgRef],
    ) -> bool:
        """Apply evidence newer than the peer's last; True when applied."""
        with self._lock:
            status = self._peers.get(peer_id)
            if status is None or not _supersedes(order, status.order):
                return False
            if (
                order is not None
                and status.order is not None
                and order.owner != status.order.owner
            ):
                status.busy_order = None  # the peer restarted
            status.state = state
            status.order = order
            if state is SwarmNodeState.BUSY:
                status.busy_order = order
            return True

    def status(self, peer_id: int) -> Optional[PeerStatus]:
        with self._lock:
            status = self._peers.get(peer_id)
            return None if status is None else PeerStatus(
                status.state, status.order, status.busy_order,
            )

    def busy(self) -> set[int]:
        with self._lock:
            return {
                peer_id
                for peer_id, status in self._peers.items()
                if status.state is SwarmNodeState.BUSY
            }

    def bid_is_current(self, peer_id: int, bid: Optional[MsgRef]) -> bool:
        """False when the peer reported BUSY after sending this bid."""
        with self._lock:
            status = self._peers.get(peer_id)
            busy_order = status.busy_order if status is not None else None
        return sent_after(bid, busy_order)


__all__ = [
    "MAX_REMOTE_PEERS",
    "PeerRoster",
    "PeerStatus",
    "sent_after",
]
