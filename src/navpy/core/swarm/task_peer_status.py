"""Owner side: apply what presence hears about peers, replan on a change."""

from __future__ import annotations

from collections.abc import Callable
from typing import Optional

from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.link.swarm_heartbeat_msg import SwarmNodeState
from navpy.core.swarm.task_auction_state import TaskAuctionState
from navpy.core.swarm.task_lost_helper import (
    LOST_HELPER_KEPT_LOG,
    LOST_HELPER_RELEASED_LOG,
    LostHelperPolicy,
    LostTask,
)
from navpy.core.swarm.task_msg_refs import MsgRef
from navpy.core.swarm.task_rebroadcast import TaskRebroadcastCoordinator


class PeerStatusCoordinator:
    """Heartbeats, check-ins, check-outs and silence move peers between the
    free and busy sets; any move re-advertises and plans again.

    A confirmed task's helper boot may be lost (checked out, or unheard for
    one heartbeat TTL): the lost-helper policy keeps the task or takes it
    back (task_lost_helper.py).
    """

    def __init__(
        self,
        state: TaskAuctionState,
        rebroadcast: TaskRebroadcastCoordinator,
        replan: Callable[[], None],
        logger: ILogger,
        lost_helper_policy: Callable[[], LostHelperPolicy] = (
            lambda: LostHelperPolicy.KEEP
        ),
    ) -> None:
        self._state = state
        self._rebroadcast = rebroadcast
        self._replan = replan
        self._logger = logger
        self._lost_helper_policy = lost_helper_policy

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

    def peer_checked_out(self, peer_id: int, order: Optional[MsgRef]) -> None:
        """A check-out silences the peer; the tasks its boot acked before
        it are lost (one without a UID names no boot: silence decides)."""
        changed = self._rebroadcast.peer_checked_out(peer_id)
        released = order is not None and self._report(
            self._state.lost_helpers.checked_out(
                peer_id, self._lost_helper_policy(), order,
            )
        )
        if changed or released:
            self._replan()

    def check_silence(self) -> None:
        """At this node's heartbeat tick: unheard peers fall silent, and
        unheard helper boots are lost."""
        changed = self._rebroadcast.expire_silent_peers()
        released = self._report(
            self._state.lost_helpers.expired(self._lost_helper_policy())
        )
        if changed or released:
            self._replan()

    def _report(self, tasks: list[LostTask]) -> bool:
        """Log each lost task; True if any was released."""
        for task in tasks:
            template = (
                LOST_HELPER_RELEASED_LOG if task.released
                else LOST_HELPER_KEPT_LOG
            )
            self._logger.warning(template.format(
                task=task.task_id, peer=task.peer_id, reason=task.reason,
            ))
        return any(task.released for task in tasks)


__all__ = ["PeerStatusCoordinator"]
