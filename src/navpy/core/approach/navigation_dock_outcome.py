"""NAV exit's view of how the guidance episode ended (missed_dock.md)."""

from __future__ import annotations

from typing import Optional

from navpy.core.approach.navigation_mode import NavigationModeState
from navpy.core.approach.vision_nav.episode_latch import (
    GuidanceEpisodeResult,
)


class GuidanceDockOutcomeService:
    """Route the episode's result and its DOCK_FAILED row through the mode session."""

    def __init__(self, mode_state: NavigationModeState) -> None:
        self._mode_state = mode_state

    def result(self) -> Optional[GuidanceEpisodeResult]:
        """NAV exit, before reset: the armed episode's first result."""
        with self._mode_state.session() as active:
            guidance = active.guidance
            outcome = None if guidance is None else guidance.dock_outcome
            return None if outcome is None else outcome.result()

    def record_failed(self, result: GuidanceEpisodeResult, reason: str) -> None:
        """NAV exit of a failed dock, after cleanup (evidence row only)."""
        with self._mode_state.session() as active:
            guidance = active.guidance
            outcome = None if guidance is None else guidance.dock_outcome
            if outcome is not None:
                outcome.record_failed(result, reason)


__all__ = ["GuidanceDockOutcomeService"]
