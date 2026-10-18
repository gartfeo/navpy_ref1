"""NAV exit's view of how the final-approach episode ended (missed_dock.md)."""

from __future__ import annotations

from typing import Optional

from navpy.core.approach.navigation_mode import NavigationModeState
from navpy.core.approach.vision_nav.episode_latch import (
    FinalApproachEpisodeTerminal,
)


class FinalApproachDockOutcomeService:
    """Route the episode's terminal and its DOCK_FAILED row through the mode session."""

    def __init__(self, mode_state: NavigationModeState) -> None:
        self._mode_state = mode_state

    def terminal(self) -> Optional[FinalApproachEpisodeTerminal]:
        """NAV exit, before reset: the armed episode's first terminal."""
        with self._mode_state.session() as active:
            final_approach = active.final_approach
            outcome = None if final_approach is None else final_approach.dock_outcome
            return None if outcome is None else outcome.terminal()

    def record_failed(self, terminal: FinalApproachEpisodeTerminal, reason: str) -> None:
        """NAV exit of a failed dock, after cleanup (evidence row only)."""
        with self._mode_state.session() as active:
            final_approach = active.final_approach
            outcome = None if final_approach is None else final_approach.dock_outcome
            if outcome is not None:
                outcome.record_failed(terminal, reason)


__all__ = ["FinalApproachDockOutcomeService"]
