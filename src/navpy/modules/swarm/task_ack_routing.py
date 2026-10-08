"""Route inbound SWARM_ACKs to the owner of the message type they ack."""

from __future__ import annotations

from collections.abc import Callable, Mapping
from typing import Optional

from navpy.logger.cache_logger import ILogger
from navpy.modules.comm.messages.swarm_ack_msg import SwarmAckMsg
from navpy.modules.comm.messages.types import MsgType
from navpy.modules.swarm.task_msg_refs import MsgRef


def acked_ref(ack: SwarmAckMsg) -> MsgRef:
    """The acknowledged message: one this ack's receiver sent."""
    return MsgRef(ack.receiver_id, ack.ref_boot_id, ack.ref_msg_seq)


class TaskAckRouter:
    """Dispatch each ack by the type of the message it acknowledges."""

    def __init__(
        self,
        handlers: Mapping[MsgType, Callable[[SwarmAckMsg], None]],
        logger: ILogger,
    ) -> None:
        self._handlers = dict(handlers)
        self._logger = logger

    def route(self, ack: SwarmAckMsg) -> None:
        handler = self._handlers.get(_acked_type(ack.ref_msg_type))
        if handler is None:
            self._logger.debug(
                f"Ignoring SWARM_ACK of message type {ack.ref_msg_type} "
                f"from {ack.sender_id}."
            )
            return
        handler(ack)


def _acked_type(value: int) -> Optional[MsgType]:
    try:
        return MsgType(value)
    except ValueError:
        return None


__all__ = ["TaskAckRouter", "acked_ref"]
