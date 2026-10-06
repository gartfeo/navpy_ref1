"""Confirmation popup-round bookkeeping independent of transport and images."""

from __future__ import annotations

import threading
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional

from gcs.backend.task_confirm_uid import (
    LEGACY_ROUND_UID,
    canonical_round_uid,
    parse_round_uid,
)
from navpy.modules.comm.messages.available_task_msg import TaskConfirmRequestMsg


class RequestAction(Enum):
    POPUP = auto()
    REPEAT = auto()
    ANSWER = auto()


@dataclass(frozen=True)
class OpenRound:
    first_seen: float
    round_uid: str


@dataclass(frozen=True)
class DecidedRound:
    is_confirmed: bool
    decided_at: float
    round_uid: str


def round_uid(request: TaskConfirmRequestMsg) -> str:
    if not request.has_meta():
        return LEGACY_ROUND_UID
    meta = request.meta
    if meta.boot_id == 0 and meta.msg_seq == 0:
        return LEGACY_ROUND_UID
    return f"{meta.boot_id}:{meta.msg_seq}"


class ConfirmationRoundRegistry:
    """Own round classification, retained decisions, and image request flags."""

    def __init__(
        self,
    ) -> None:
        self._lock = threading.Lock()
        self._handled: dict[tuple[int, int], OpenRound] = {}
        self._image_received: set[tuple[int, int]] = set()
        self._thumbnail_requested: set[tuple[int, int]] = set()
        self._decided: dict[tuple[int, int], dict[str, DecidedRound]] = {}

    def classify(
        self,
        sys_id: int,
        task_id: int,
        uid: str,
        now: float,
        retention_s: float,
    ) -> tuple[RequestAction, bool]:
        uid = canonical_round_uid(uid)
        key = (sys_id, task_id)
        with self._lock:
            decided_rounds = self._decided.get(key)
            if decided_rounds is not None:
                stale_uids = [
                    round_uid
                    for round_uid, decided in decided_rounds.items()
                    if (now - decided.decided_at) >= retention_s
                ]
                for stale_uid in stale_uids:
                    decided_rounds.pop(stale_uid, None)
                decided = decided_rounds.get(uid)
                if decided is not None:
                    return RequestAction.ANSWER, decided.is_confirmed
                if not decided_rounds:
                    self._decided.pop(key, None)

            open_round = self._handled.get(key)
            if (
                open_round is not None
                and (now - open_round.first_seen) < retention_s
                and open_round.round_uid == uid
            ):
                return RequestAction.REPEAT, False

            self._forget_open(key)
            self._handled[key] = OpenRound(now, uid)
            return RequestAction.POPUP, False

    def clear(self, sys_id: int, task_id: int) -> None:
        with self._lock:
            key = (sys_id, task_id)
            self._forget_open(key)
            self._decided.pop(key, None)

    def mark_decided(
        self,
        sys_id: int,
        task_id: int,
        is_confirmed: bool,
        uid: Optional[str],
        now: float,
    ) -> None:
        key = (sys_id, task_id)
        if uid is None:
            return
        uid = canonical_round_uid(uid)
        with self._lock:
            open_round = self._handled.get(key)
            if open_round is not None and open_round.round_uid == uid:
                self._forget_open(key)
            decided_rounds = self._decided.setdefault(key, {})
            decided_rounds[uid] = DecidedRound(is_confirmed, now, uid)

    def open_uid(self, sys_id: int, task_id: int) -> Optional[str]:
        with self._lock:
            open_round = self._handled.get((sys_id, task_id))
            return None if open_round is None else open_round.round_uid

    def thumbnail_request_due(
        self,
        sys_id: int,
        task_id: int,
        now: float,
        grace_s: float,
    ) -> Optional[str]:
        key = (sys_id, task_id)
        with self._lock:
            open_round = self._handled.get(key)
            if open_round is None:
                return None
            if key in self._image_received or key in self._thumbnail_requested:
                return None
            if (now - open_round.first_seen) < grace_s:
                return None
            self._thumbnail_requested.add(key)
            return open_round.round_uid

    def accept_image(
        self,
        sys_id: int,
        task_id: int,
        uid: Optional[str],
    ) -> bool:
        if uid is None:
            return False
        uid = canonical_round_uid(uid)
        with self._lock:
            key = (sys_id, task_id)
            open_round = self._handled.get(key)
            if open_round is None or open_round.round_uid != uid:
                return False
            self._image_received.add(key)
            return True

    def _forget_open(self, key: tuple[int, int]) -> None:
        self._handled.pop(key, None)
        self._image_received.discard(key)
        self._thumbnail_requested.discard(key)


__all__ = [
    "ConfirmationRoundRegistry",
    "canonical_round_uid",
    "DecidedRound",
    "LEGACY_ROUND_UID",
    "OpenRound",
    "RequestAction",
    "parse_round_uid",
    "round_uid",
]
