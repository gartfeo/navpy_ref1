"""How an armed final-approach episode ended, for NAV exit (missed_dock.md).

NAV cleanup reads the episode's first terminal before the reset clears the
latch; after a successful exit it records the DOCK_FAILED row. Neither is a
command input.
"""

from __future__ import annotations

from collections.abc import Callable

from navpy.core.approach.vision_nav.episode_latch import (
    FinalApproachEpisodeLatch,
    FinalApproachEpisodeTerminal,
)
from navpy.core.kernel import runtime_time as rt
from navpy.core.ports.dock_contact import DockContactIdentity


class FinalApproachDockOutcome:
    def __init__(
        self,
        lock: rt.RLock,
        episode: FinalApproachEpisodeLatch,
        record_failed: Callable[[DockContactIdentity, str], None],
    ) -> None:
        self._lock = lock
        self._episode = episode
        self._record_failed = record_failed

    def terminal(self) -> FinalApproachEpisodeTerminal | None:
        """The armed episode's first terminal; None when none was armed."""
        with self._lock:
            return self._episode.terminal_snapshot()

    def record_failed(self, terminal: FinalApproachEpisodeTerminal, reason: str) -> None:
        """The DOCK_FAILED row: after the reset drained the command evidence."""
        self._record_failed(terminal.identity, reason)


__all__ = ["FinalApproachDockOutcome"]
