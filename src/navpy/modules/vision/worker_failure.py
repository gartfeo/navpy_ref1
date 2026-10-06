"""Persistent exact failure state for daemon-owned vision workers."""

from __future__ import annotations

import threading
from collections.abc import Callable
from typing import Protocol, runtime_checkable


@runtime_checkable
class FailureHealthPort(Protocol):
    def raise_if_failed(self) -> None: ...


class StaticFailureHealth:
    """Default health contract for components with no background execution."""

    def raise_if_failed(self) -> None:
        pass


def require_failure_health(value: object) -> FailureHealthPort:
    """Fail closed when a lifecycle component omits worker-health reporting."""
    if not isinstance(value, FailureHealthPort):
        raise TypeError(
            f"{type(value).__name__} does not provide raise_if_failed()"
        )
    return value


class WorkerFailureLatch:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._failure: BaseException | None = None

    def record(self, failure: BaseException) -> None:
        with self._lock:
            if self._failure is None:
                self._failure = failure

    def run(
        self,
        action: Callable[[], None],
        on_failure: Callable[[], None] | None = None,
    ) -> None:
        try:
            action()
        except BaseException as failure:
            self.record(failure)
            if on_failure is not None:
                try:
                    on_failure()
                except BaseException:
                    pass

    def raise_if_failed(self) -> None:
        with self._lock:
            failure = self._failure
        if failure is not None:
            raise failure


__all__ = [
    "FailureHealthPort",
    "StaticFailureHealth",
    "WorkerFailureLatch",
    "require_failure_health",
]
