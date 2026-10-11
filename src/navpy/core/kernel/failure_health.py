"""Persistent health boundary for detachable runtime components."""

from __future__ import annotations

import threading
from typing import Protocol, Sequence


class FailureHealthPort(Protocol):
    def raise_if_failed(self) -> None: ...


class ComponentFailureLatch:
    """Remember the first fatal even after its component is detached."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._failure: BaseException | None = None

    def raise_if_failed(
        self,
        components: Sequence[FailureHealthPort | None],
    ) -> None:
        with self._lock:
            failure = self._failure
        if failure is not None:
            raise failure
        for component in components:
            if component is None:
                continue
            try:
                component.raise_if_failed()
            except BaseException as component_failure:
                raise self._remember(component_failure)

    def harvest(self, component: FailureHealthPort) -> None:
        try:
            component.raise_if_failed()
        except BaseException as failure:
            self._remember(failure)

    def _remember(self, failure: BaseException) -> BaseException:
        with self._lock:
            if self._failure is None:
                self._failure = failure
            return self._failure


__all__ = ["ComponentFailureLatch", "FailureHealthPort"]
