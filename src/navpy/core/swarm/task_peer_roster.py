"""Admitted swarm peers and the newest evidence of their availability."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Optional

from navpy.core.kernel import runtime_time as rt
from navpy.core.link.swarm_heartbeat_msg import SwarmNodeState
from navpy.core.link.ttl_defaults import get_ttl_ms
from navpy.core.link.types import MsgType
from navpy.core.swarm.task_msg_refs import MsgRef


MAX_REMOTE_PEERS = 2

# A peer unheard (no heartbeat or check-in) for one heartbeat TTL is silent.
PEER_SILENCE_EXPIRY_S = get_ttl_ms(MsgType.SWARM_HEARTBEAT) / 1000.0
#: What the roster logs when a peer's silence begins by expiry, and when it ends.
PEER_SILENT_LOG = "Peer {} silent"
PEER_HEARD_AGAIN_LOG = "Peer {} heard again"
#: What is logged at debug when the roster first takes a peer: at info it would be in every run's journal.
PEER_ADMITTED_LOG = "Peer {} admitted"


@dataclass
class PeerStatus:
    """What one peer last reported about itself, as the owner sees it.

    ``order`` is the UID of the evidence applied last and ``busy_order``
    that of the newest BUSY evidence, which makes the peer's older bids
    stale. A silent peer counts as busy until it is heard again.
    """

    state: SwarmNodeState = SwarmNodeState.FREE
    order: Optional[MsgRef] = None
    busy_order: Optional[MsgRef] = None
    last_heard_s: float = 0.0
    silent: bool = False


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

    def __init__(
        self,
        lock: rt.RLock,
        monotonic_s: Callable[[], float] = rt.monotonic,
        note: Callable[[str], None] = lambda _line: None,
    ) -> None:
        self._lock = lock
        self._monotonic_s = monotonic_s
        self._note = note
        self._peers: dict[int, PeerStatus] = {}

    def add(self, peer_id: int) -> bool:
        with self._lock:
            if peer_id in self._peers or len(self._peers) >= MAX_REMOTE_PEERS:
                return False
            self._peers[peer_id] = PeerStatus(last_heard_s=self._monotonic_s())
            return True

    def heard(self, peer_id: int) -> None:
        """A heartbeat or check-in: the peer is present (revives it)."""
        with self._lock:
            status = self._peers.get(peer_id)
            if status is not None:
                status.last_heard_s = self._monotonic_s()
                if status.silent:
                    self._note(PEER_HEARD_AGAIN_LOG.format(peer_id))
                status.silent = False

    def silence(self, peer_id: int) -> None:
        """The peer checked out: silent at once."""
        with self._lock:
            status = self._peers.get(peer_id)
            if status is not None:
                status.silent = True

    def expire_silent(self) -> None:
        """Mark peers unheard for longer than PEER_SILENCE_EXPIRY_S."""
        with self._lock:
            now_s = self._monotonic_s()
            for peer_id, status in self._peers.items():
                if not status.silent and now_s - status.last_heard_s > PEER_SILENCE_EXPIRY_S:
                    status.silent = True
                    self._note(PEER_SILENT_LOG.format(peer_id))

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
                status.state,
                status.order,
                status.busy_order,
                status.last_heard_s,
                status.silent,
            )

    def busy(self) -> set[int]:
        """Peers reported BUSY or silent."""
        with self._lock:
            return {
                peer_id
                for peer_id, status in self._peers.items()
                if status.state is SwarmNodeState.BUSY or status.silent
            }

    def bid_is_current(self, peer_id: int, bid: Optional[MsgRef]) -> bool:
        """False when the peer reported BUSY after sending this bid."""
        with self._lock:
            status = self._peers.get(peer_id)
            busy_order = status.busy_order if status is not None else None
        return sent_after(bid, busy_order)


__all__ = [
    "MAX_REMOTE_PEERS",
    "PEER_ADMITTED_LOG",
    "PEER_HEARD_AGAIN_LOG",
    "PEER_SILENCE_EXPIRY_S",
    "PEER_SILENT_LOG",
    "PeerRoster",
    "PeerStatus",
    "sent_after",
]
