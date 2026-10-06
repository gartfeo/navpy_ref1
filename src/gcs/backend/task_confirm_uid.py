"""Canonical confirmation-round identity shared by GCS boundaries."""

from __future__ import annotations


LEGACY_ROUND_UID = "legacy"


def parse_round_uid(uid: str) -> tuple[int, int]:
    if uid == LEGACY_ROUND_UID:
        return 0, 0
    if type(uid) is not str:
        raise ValueError("invalid confirmation round uid")
    parts = uid.split(":")
    if (
        len(parts) != 2
        or not all(part.isascii() and part.isdecimal() for part in parts)
    ):
        raise ValueError("invalid confirmation round uid")
    boot_id, msg_seq = (int(part) for part in parts)
    if (
        (boot_id == 0 and msg_seq == 0)
        or boot_id > 0xFFFFFFFF
        or msg_seq > 0xFFFFFFFF
        or uid != f"{boot_id}:{msg_seq}"
    ):
        raise ValueError("invalid confirmation round uid")
    return boot_id, msg_seq


def canonical_round_uid(uid: str) -> str:
    boot_id, msg_seq = parse_round_uid(uid)
    if boot_id == 0 and msg_seq == 0:
        return LEGACY_ROUND_UID
    return f"{boot_id}:{msg_seq}"


__all__ = ["LEGACY_ROUND_UID", "canonical_round_uid", "parse_round_uid"]
