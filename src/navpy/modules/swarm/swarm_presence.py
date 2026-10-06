"""Swarm check-in, check-out, and heartbeat presence messages."""

from __future__ import annotations

from collections.abc import Callable
from threading import Thread

from navpy.logger.cache_logger import ILogger
from navpy.modules.comm.messages.check_msg import CheckInMsg, CheckOutMsg
from navpy.modules.comm.messages.location_msg import LocationMsgData
from navpy.modules.comm.messages.swarm_heartbeat_msg import SwarmHeartbeatMsg
from navpy.modules.swarm.swarm_heartbeat_runtime import SwarmHeartbeatRuntime
from navpy.modules.swarm.task_messaging import TaskMessageSender
from navpy.modules.swarm.task_ports import TaskLocationReader


class SwarmPresence:
    """Own swarm presence messages and delegate heartbeat lifecycle."""

    def __init__(
        self,
        actor_id: int,
        vehicle: TaskLocationReader,
        sender: TaskMessageSender,
        peer_discovered: Callable[[int], None],
        logger: ILogger,
    ) -> None:
        self._actor_id = actor_id
        self._vehicle = vehicle
        self._sender = sender
        self._peer_discovered = peer_discovered
        self._logger = logger
        self._heartbeat = SwarmHeartbeatRuntime(sender)

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
        self._heartbeat.stop()

    def on_heartbeat(self, message: SwarmHeartbeatMsg) -> None:
        self._discover_foreign_peer(message.sender_id)

    def on_checkin(self, message: CheckInMsg) -> None:
        if self._discover_foreign_peer(message.sender_id):
            # Reply immediately so a later starter learns incumbents without
            # waiting for the next one-second wall heartbeat.
            self._sender.heartbeat()
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

    def heartbeat_thread(self) -> Thread | None:
        return self._heartbeat.heartbeat_thread()

    def raise_if_failed(self) -> None:
        self._heartbeat.raise_if_failed()

    def _discover_foreign_peer(self, sender_id: int) -> bool:
        if sender_id == self._actor_id:
            return False
        self._peer_discovered(sender_id)
        return True


__all__ = ["SwarmPresence"]
