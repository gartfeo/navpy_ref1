"""References to sent swarm messages, as SWARM_ACK names them.

A message is identified by its dedup UID ``(sender, boot_id, msg_seq)``.
``msg_seq`` is monotonic per sender process, so two references from one
sender boot are ordered by their seq; references from different boots or
senders are not ordered at all.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Optional

from navpy.core.kernel import runtime_time as rt
from navpy.core.link.msg_abc import MsgABC


@dataclass(frozen=True)
class OwnerRef:
    """One task-owner process; a restarted owner is a new boot."""

    owner_id: int
    boot_id: int


@dataclass(frozen=True)
class MsgRef:
    """The UID of one sent message."""

    sender_id: int
    boot_id: int
    msg_seq: int

    @property
    def owner(self) -> OwnerRef:
        """The sending process, as the owner of what this message offers."""
        return OwnerRef(self.sender_id, self.boot_id)

    def follows(self, floor: Optional[MsgRef]) -> bool:
        """True only when the same sender boot sent this after ``floor``."""
        return (
            floor is not None
            and self.owner == floor.owner
            and self.msg_seq > floor.msg_seq
        )


class RecentRefs:
    """UIDs of sent copies, each kept ``retention_s`` after its send.

    The retention covers a copy's TTL and its ack's, so an ordinary ack
    names a known copy; a later one names nothing.
    """

    def __init__(
        self,
        retention_s: float,
        monotonic_s: Callable[[], float] = rt.monotonic,
    ) -> None:
        self._retention_s = retention_s
        self._monotonic_s = monotonic_s
        self._sent: dict[MsgRef, float] = {}

    def add(self, ref: MsgRef) -> None:
        self._prune()
        self._sent.pop(ref, None)
        self._sent[ref] = self._monotonic_s()

    def __contains__(self, ref: object) -> bool:
        self._prune()
        return ref in self._sent

    def __len__(self) -> int:
        self._prune()
        return len(self._sent)

    def _prune(self) -> None:
        cutoff = self._monotonic_s() - self._retention_s
        # Sends are kept in time order, so the stale ones lead.
        for ref, sent_s in list(self._sent.items()):
            if sent_s > cutoff:
                break
            del self._sent[ref]


def msg_ref(message: MsgABC) -> Optional[MsgRef]:
    """Return the message's UID, or None when it has no valid meta.

    A zero ``(boot_id, msg_seq)`` pair is legacy meta (see MessageFilter),
    which names no message.
    """
    meta = message.meta
    if meta is None or (meta.boot_id == 0 and meta.msg_seq == 0):
        return None
    return MsgRef(message.sender_id, meta.boot_id, meta.msg_seq)


__all__ = ["MsgRef", "OwnerRef", "RecentRefs", "msg_ref"]
