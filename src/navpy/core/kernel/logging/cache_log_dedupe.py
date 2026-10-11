"""Caller-side duplicate suppression policies for CacheLogger."""

from __future__ import annotations

import threading


class CacheLogDeduplicator:
    """Own raw-message dedupe independently from formatting and output."""

    def __init__(self) -> None:
        self._last_info_by_key: dict[str, object] = {}
        self._last_warning: object = None
        self._single_warning_keys: set[str] = set()
        self._lock = threading.Lock()

    def accept_info(self, key: str, msg: object) -> bool:
        with self._lock:
            if self._last_info_by_key.get(key) == msg:
                return False
            self._last_info_by_key[key] = msg
            return True

    def accept_warning(self, msg: object) -> bool:
        with self._lock:
            if self._last_warning == msg:
                return False
            self._last_warning = msg
            return True

    def accept_single_warning(self, key: str) -> bool:
        with self._lock:
            if key in self._single_warning_keys:
                return False
            self._single_warning_keys.add(key)
            return True


__all__ = ["CacheLogDeduplicator"]
