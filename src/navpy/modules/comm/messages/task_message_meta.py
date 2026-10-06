"""Shared MAVLink metadata conversion for task protocol messages."""

from __future__ import annotations

from typing import Optional, Protocol

from navpy.modules.comm.messages.msg_meta import MsgMeta


class MavlinkMetaFields(Protocol):
    boot_id: int
    msg_seq: int
    time_ms: int
    ttl_ms: int


def get_meta_fields(meta: Optional[MsgMeta]) -> tuple[int, int, int, int]:
    if meta is None:
        return 0, 0, 0, 0
    return meta.boot_id, meta.msg_seq, meta.time_ms, meta.ttl_ms


def meta_from_mavlink(message: MavlinkMetaFields) -> MsgMeta:
    return MsgMeta(
        boot_id=message.boot_id,
        msg_seq=message.msg_seq,
        time_ms=message.time_ms,
        ttl_ms=message.ttl_ms,
    )


__all__ = ["MavlinkMetaFields", "get_meta_fields", "meta_from_mavlink"]
