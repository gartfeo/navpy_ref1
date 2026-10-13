"""Serialize final-approach bootstrap with source reset invalidation."""

from __future__ import annotations

from collections.abc import Callable, Iterator
from contextlib import contextmanager

from navpy.core.kernel import runtime_time as rt


class FinalApproachSourceResetFence:
    """Make nav-inbox discard and command invalidation one reset fence."""

    def __init__(
        self,
        discard_events: Callable[[], None],
        invalidate_pending: Callable[[], None],
    ) -> None:
        self._discard_events = discard_events
        self._invalidate_pending = invalidate_pending
        self._lock = rt.RLock()

    @contextmanager
    def bootstrap(self) -> Iterator[None]:
        with self._lock:
            yield

    def reset(self) -> None:
        with self._lock:
            try:
                self._discard_events()
            finally:
                self._invalidate_pending()


__all__ = ["FinalApproachSourceResetFence"]
