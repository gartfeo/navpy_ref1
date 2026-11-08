"""Thread-safe positive and temporary-negative parameter cache."""
from __future__ import annotations

from collections.abc import Callable

from navpy.core.kernel import runtime_time as rt


class ParameterRepository:
    def __init__(self) -> None:
        self._lock = rt.Lock()
        self._values: dict[str, float] = {}
        self._missing_until: dict[str, float] = {}
        self._watchers: dict[str, list[Callable[[], None]]] = {}

    def watch(self, name: str, changed: Callable[[], None]) -> None:
        """Call ``changed`` (on the updating thread, outside the lock) after each update of ``name``."""
        with self._lock:
            self._watchers.setdefault(name.upper(), []).append(changed)

    def update(self, name: str, value: float) -> None:
        normalized = name.rstrip("\x00").upper()
        with self._lock:
            self._values[normalized] = value
            self._missing_until.pop(normalized, None)
            watchers = tuple(self._watchers.get(normalized, ()))
        for changed in watchers:
            changed()

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
            self._missing_until[name.upper()] = rt.time() + retry_after_s

    def missing_is_fresh(self, name: str) -> bool:
        normalized = name.upper()
        with self._lock:
            deadline = self._missing_until.get(normalized)
            if deadline is None:
                return False
            if rt.time() < deadline:
                return True
            self._missing_until.pop(normalized, None)
            return False
