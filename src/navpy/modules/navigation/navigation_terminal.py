"""Narrow terminal capabilities consumed by the navigation state machine."""

from __future__ import annotations

from typing import Iterable, Optional

from navpy.modules.navigation.navigation_mode import NavigationModeState
from navpy.modules.vision.models.detect_data import DetectedObject


class TerminalNavigationService:
    """Route terminal operations without exposing runtime mutable state."""

    def __init__(self, mode_state: NavigationModeState) -> None:
        self._mode_state = mode_state

    @property
    def is_active(self) -> bool:
        with self._mode_state.session() as active:
            return active.terminal is not None

    def clear_source_discontinuity(
        self,
        source_names: str | Iterable[str],
    ) -> None:
        with self._mode_state.session() as active:
            active.runtime.clear_source_discontinuity_state(source_names)

    def invalidate_pending_source_work(self) -> None:
        """Fence work queued before a detector source reset completes."""
        with self._mode_state.session() as active:
            active.runtime.invalidate_commands()

    def consume_command_liveness_failure(self) -> bool:
        with self._mode_state.session() as active:
            return active.runtime.consume_command_liveness_failure()

    def can_confirm_detection(self, detect_data: DetectedObject) -> bool:
        with self._mode_state.session() as active:
            terminal = active.terminal
            return (
                False
                if terminal is None
                else terminal.confirmation.can_confirm_detection(detect_data)
            )

    def record_confirmed_detection(self, detect_data: DetectedObject) -> bool:
        with self._mode_state.session() as active:
            terminal = active.terminal
            return (
                False
                if terminal is None
                else terminal.confirmation.record_terminal_confirmed_detection(
                    detect_data
                )
            )

    def target_passed_override(self) -> Optional[bool]:
        with self._mode_state.session() as active:
            terminal = active.terminal
            return (
                None
                if terminal is None
                else terminal.status.target_passed_override()
            )

    def last_measured_lateral_bearing_deg(self) -> Optional[float]:
        with self._mode_state.session() as active:
            terminal = active.terminal
            return (
                None
                if terminal is None
                else terminal.status.last_measured_lateral_bearing_deg()
            )


__all__ = ["TerminalNavigationService"]
