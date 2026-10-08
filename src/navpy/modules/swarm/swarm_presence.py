"""Swarm check-in, check-out, and heartbeat presence messages."""

from __future__ import annotations

import threading
from collections.abc import Callable
from typing import Optional

from navpy.logger.cache_logger import ILogger
from navpy.modules.comm.messages.check_msg import CheckInMsg, CheckOutMsg
from navpy.modules.comm.messages.location_msg import LocationMsgData
from navpy.modules.comm.messages.swarm_heartbeat_msg import (
    SwarmHeartbeatMsg,
    SwarmNodeState,
)
from navpy.modules.swarm.swarm_heartbeat_runtime import SwarmHeartbeatRuntime
from navpy.modules.swarm.task_messaging import TaskMessageSender
from navpy.modules.swarm.task_msg_refs import MsgRef, msg_ref
from navpy.modules.swarm.task_ports import TaskLocationReader


PeerReport = Callable[[int, SwarmNodeState, Optional[MsgRef]], None]


class SwarmPresence:
    """Own swarm presence messages and delegate heartbeat lifecycle."""

    def __init__(
        self,
        actor_id: int,
        vehicle: TaskLocationReader,
        sender: TaskMessageSender,
        lock: threading.RLock,
        node_state: Callable[[], SwarmNodeState],
        peer_discovered: Callable[[int], None],
        peer_reported: PeerReport,
        logger: ILogger,
    ) -> None:
        self._actor_id = actor_id
        self._vehicle = vehicle
        self._sender = sender
        self._lock = lock
        self._node_state = node_state
        self._peer_discovered = peer_discovered
        self._peer_reported = peer_reported
        self._logger = logger
        self._heartbeat = SwarmHeartbeatRuntime(self)

    def start(self) -> None:
        if not self._heartbeat.start():
            return
        try:
            self._sender.checkin()
        except BaseException:
            self._heartbeat.stop()
            raise

    def is_started(self) -> bool:
        return self._heartbeat.is_started()

    def stop(self) -> None:
        # Never under the actor lock: the heartbeat thread takes it.
        self._heartbeat.stop()

    def heartbeat(self) -> Optional[MsgRef]:
        """Report this node's state.

        The state is read and stamped under the actor lock, so the report
        is ordered against step-4 copies; it is sent outside the lock.
        """
        with self._lock:
            message = self._sender.heartbeat_message(self._node_state())
        return self._sender.send_heartbeat(message)

    def on_heartbeat(self, message: SwarmHeartbeatMsg) -> None:
        if self._discover_foreign_peer(message.sender_id):
            self._peer_reported(
                message.sender_id,
                SwarmNodeState.from_code(message.state),
                msg_ref(message),
            )

    def on_checkin(self, message: CheckInMsg) -> None:
        if self._discover_foreign_peer(message.sender_id):
            # Reply immediately so a later starter learns incumbents without
            # waiting for the next one-second wall heartbeat.
            self.heartbeat()
        self._logger.info(f"Actor {message.sender_id} checked in.")

    def on_checkout(self, message: CheckOutMsg) -> None:
        self._logger.info(
            f"Actor {message.sender_id} checked out at location "
            f"{message.location}."
        )

    def checkin(self) -> None:
        self._sender.checkin()

    def checkout(self) -> None:
        location = self._vehicle.location(False)
        if location is None:
            self._logger.warning(
                f"{self._actor_id}: Location not set. Cannot check out."
            )
            return
        self._sender.checkout(LocationMsgData(
            location.lat,
            location.lng,
            location.alt,
        ))

    def heartbeat_thread(self) -> threading.Thread | None:
        return self._heartbeat.heartbeat_thread()

    def raise_if_failed(self) -> None:
        self._heartbeat.raise_if_failed()

    def _discover_foreign_peer(self, sender_id: int) -> bool:
        if sender_id == self._actor_id:
            return False
        self._peer_discovered(sender_id)
        return True


__all__ = ["SwarmPresence"]
