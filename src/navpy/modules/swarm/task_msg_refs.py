"""References to sent swarm messages, as SWARM_ACK names them.

A message is identified by its dedup UID ``(sender, boot_id, msg_seq)``.
``msg_seq`` is monotonic per sender process, so two references from one
sender boot are ordered by their seq; references from different boots or
senders are not ordered at all.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from navpy.modules.comm.messages.msg_abc import MsgABC


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


def msg_ref(message: MsgABC) -> Optional[MsgRef]:
    """Return the message's UID, or None when it has no valid meta.

    A zero ``(boot_id, msg_seq)`` pair is legacy meta (see MessageFilter),
    which names no message.
    """
    meta = message.meta
    if meta is None or (meta.boot_id == 0 and meta.msg_seq == 0):
        return None
    return MsgRef(message.sender_id, meta.boot_id, meta.msg_seq)


__all__ = ["MsgRef", "OwnerRef", "msg_ref"]
