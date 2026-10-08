"""Owner side: apply what presence hears about peers, replan on a change."""

from __future__ import annotations

from collections.abc import Callable
from typing import Optional

from navpy.modules.comm.messages.swarm_heartbeat_msg import SwarmNodeState
from navpy.modules.swarm.task_auction_state import TaskAuctionState
from navpy.modules.swarm.task_msg_refs import MsgRef
from navpy.modules.swarm.task_rebroadcast import TaskRebroadcastCoordinator


class PeerStatusCoordinator:
    """Heartbeats, check-ins, check-outs and silence move peers between the
    free and busy sets; any move re-advertises and plans again."""

    def __init__(
        self,
        state: TaskAuctionState,
        rebroadcast: TaskRebroadcastCoordinator,
        replan: Callable[[], None],
    ) -> None:
        self._state = state
        self._rebroadcast = rebroadcast
        self._replan = replan

    def observe_peer(
        self,
        peer_id: int,
        state: Optional[SwarmNodeState],
        order: Optional[MsgRef],
    ) -> None:
        """A heartbeat (with its state) or a check-in (None): heard."""
        revived = self._rebroadcast.peer_heard(peer_id)
        reported = state is not None and self._state.observe_peer(
            peer_id, state, order,
        )
        if revived or reported:
            self._replan()

    def peer_checked_out(self, peer_id: int) -> None:
        if self._rebroadcast.peer_checked_out(peer_id):
            self._replan()

    def check_silence(self) -> None:
        """At this node's heartbeat tick: unheard peers fall silent."""
        if self._rebroadcast.expire_silent_peers():
            self._replan()


__all__ = ["PeerStatusCoordinator"]
