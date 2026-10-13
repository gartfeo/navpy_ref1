"""Latched command-stream failure shared by navigation and navigation."""

from __future__ import annotations

from typing import Protocol

from navpy.core.kernel import runtime_time as rt


class FinalApproachCommandFailureSink(Protocol):
    def mark_failed(self) -> None: ...


class FinalApproachCommandLiveness:
    """Publish one failure until navigation consumes or reset clears it."""

    def __init__(self) -> None:
        self._lock = rt.Lock()
        self._failed = False

    def mark_failed(self) -> None:
        with self._lock:
            self._failed = True

    def consume_failure(self) -> bool:
        with self._lock:
            failed = self._failed
            self._failed = False
            return failed

    def reset(self) -> None:
        with self._lock:
            self._failed = False


__all__ = ["FinalApproachCommandFailureSink", "FinalApproachCommandLiveness"]
