"""Narrow structural ports and adapters for swarm task leaves."""

from __future__ import annotations

from typing import Optional, Protocol

from navpy.core.link.msg_abc import MsgABC
from navpy.core.kernel.location import Location


class TaskMessageBroadcaster(Protocol):
    def broadcast(self, message: MsgABC) -> None: ...


class TaskLocationReader(Protocol):
    def location(self, relative: bool = False) -> Optional[Location]: ...


__all__ = [
    "TaskLocationReader",
    "TaskMessageBroadcaster",
]
