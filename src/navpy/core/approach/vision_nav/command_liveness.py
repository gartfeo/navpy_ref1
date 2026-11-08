"""Latched command-stream failure shared by navigation and navigation."""

from __future__ import annotations

from typing import Protocol

from navpy.core.kernel import runtime_time as rt


class GuidanceCommandFailureSink(Protocol):
    def mark_failed(self) -> None: ...


class GuidanceCommandLiveness:
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


class GuidanceEpisodeFailureGate(Protocol):
    def latch_failure(self) -> bool: ...


class GuidanceEpisodeFailures:
    """Worker failure sink: the episode latch first, then the liveness signal.

    A worker failure (stale frame, expired held command, law failure) ends the
    episode at the moment it happens, under the runtime lock, so whichever of
    it and a dock contact comes first wins (physical_docking.md). After a
    contact the failure is dropped.
    """

    def __init__(
        self,
        lock: rt.RLock,
        episode: GuidanceEpisodeFailureGate,
        liveness: GuidanceCommandLiveness,
    ) -> None:
        self._lock = lock
        self._episode = episode
        self._liveness = liveness

    def mark_failed(self) -> None:
        with self._lock:
            latched = True
            try:
                latched = self._episode.latch_failure()
            finally:
                # A raising terminal observer must not hide the failure from
                # the NAV tick: the approach would neither command nor end.
                if latched:
                    self._liveness.mark_failed()


__all__ = [
    "GuidanceCommandFailureSink",
    "GuidanceCommandLiveness",
    "GuidanceEpisodeFailures",
]
