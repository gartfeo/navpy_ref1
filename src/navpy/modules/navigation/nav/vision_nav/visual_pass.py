"""Visual-only terminal pass state machine."""

from __future__ import annotations

from dataclasses import dataclass

from navpy.modules.navigation.nav.vision_nav.frame import TerminalVisionFrame


@dataclass(frozen=True)
class VisualPassState:
    identity: tuple[str, int, int, int] | None = None
    armed: bool = False
    aft_pending: bool = False
    passed: bool = False


@dataclass(frozen=True)
class VisualPassPlan:
    suppress_command: bool
    next_state: VisualPassState


class VisualPassDetector:
    """Require a commanded forward frame and two fresh consecutive aft frames."""

    def __init__(self) -> None:
        self._state = VisualPassState()

    @property
    def passed(self) -> bool:
        return self._state.passed

    def reset(self) -> None:
        self._state = VisualPassState()

    def plan(self, frame: TerminalVisionFrame) -> VisualPassPlan:
        state = self._state
        if state.identity != frame.continuity_key:
            state = VisualPassState(identity=frame.continuity_key)
        if state.passed:
            return VisualPassPlan(True, state)
        if frame.body_x > 0.0:
            return VisualPassPlan(
                False,
                VisualPassState(frame.continuity_key, True, False, False),
            )
        if not state.armed:
            return VisualPassPlan(False, state)
        if not state.aft_pending:
            return VisualPassPlan(
                True,
                VisualPassState(frame.continuity_key, True, True, False),
            )
        return VisualPassPlan(
            True,
            VisualPassState(frame.continuity_key, True, True, True),
        )

    def commit(self, plan: VisualPassPlan) -> None:
        self._state = plan.next_state


__all__ = ["VisualPassDetector", "VisualPassPlan", "VisualPassState"]
