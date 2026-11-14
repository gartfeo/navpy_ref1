"""How an armed guidance episode ended, for NAV exit (missed_dock.md).

NAV cleanup reads the episode's first result before the reset clears the
latch; after a successful exit it records the DOCK_FAILED row. Neither is a
command input.
"""

from __future__ import annotations

from collections.abc import Callable

from navpy.core.approach.vision_nav.episode_latch import (
    GuidanceEpisodeLatch,
    GuidanceEpisodeResult,
)
from navpy.core.kernel import runtime_time as rt
from navpy.core.ports.dock_contact import DockContactIdentity


class GuidanceDockOutcome:
    def __init__(
        self,
        lock: rt.RLock,
        episode: GuidanceEpisodeLatch,
        record_failed: Callable[[DockContactIdentity, str], None],
    ) -> None:
        self._lock = lock
        self._episode = episode
        self._record_failed = record_failed

    def result(self) -> GuidanceEpisodeResult | None:
        """The armed episode's first result; None when none was armed."""
        with self._lock:
            return self._episode.result_snapshot()

    def record_failed(self, result: GuidanceEpisodeResult, reason: str) -> None:
        """The DOCK_FAILED row: after the reset drained the command evidence."""
        self._record_failed(result.identity, reason)


__all__ = ["GuidanceDockOutcome"]
