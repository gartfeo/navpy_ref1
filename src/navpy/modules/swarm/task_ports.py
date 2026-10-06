"""Narrow structural ports and adapters for swarm task leaves."""

from __future__ import annotations

from collections.abc import Callable
from typing import Optional, Protocol

from navpy.modules.comm.network_abc import NetworkAbc
from navpy.modules.comm.messages.msg_abc import MsgABC
from navpy.modules.common.models.location import Location


class TaskMessageBroadcaster(Protocol):
    def broadcast(self, message: MsgABC) -> None: ...


class TaskLocationReader(Protocol):
    def location(self, relative: bool = False) -> Optional[Location]: ...


class ClockOffsetResetPort(Protocol):
    def reset(self) -> None: ...


class MessageClockReset:
    """Adapt the compatibility network surface to one clock-reset command."""

    def __init__(self, clear_offsets: Callable[[], None]) -> None:
        self._clear_offsets = clear_offsets

    @classmethod
    def from_network(cls, network: NetworkAbc) -> "MessageClockReset":
        return cls(network.message_filter.offset_estimator.clear)

    def reset(self) -> None:
        self._clear_offsets()


__all__ = [
    "ClockOffsetResetPort",
    "MessageClockReset",
    "TaskLocationReader",
    "TaskMessageBroadcaster",
]
