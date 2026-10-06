"""Thread-safe positive and temporary-negative parameter cache."""
from __future__ import annotations

import threading
import time


class ParameterRepository:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._values: dict[str, float] = {}
        self._missing_until: dict[str, float] = {}

    def update(self, name: str, value: float) -> None:
        normalized = name.rstrip("\x00").upper()
        with self._lock:
            self._values[normalized] = value
            self._missing_until.pop(normalized, None)

    def get(self, name: str) -> float | None:
        with self._lock:
            return self._values.get(name.upper())

    def contains(self, name: str) -> bool:
        with self._lock:
            return name.upper() in self._values

    def invalidate(self, name: str) -> None:
        normalized = name.upper()
        with self._lock:
            self._values.pop(normalized, None)
            self._missing_until.pop(normalized, None)

    def mark_missing(self, name: str, retry_after_s: float) -> None:
        with self._lock:
            self._missing_until[name.upper()] = time.time() + retry_after_s

    def missing_is_fresh(self, name: str) -> bool:
        normalized = name.upper()
        with self._lock:
            deadline = self._missing_until.get(normalized)
            if deadline is None:
                return False
            if time.time() < deadline:
                return True
            self._missing_until.pop(normalized, None)
            return False
