"""Visual-only aft gating: an armed dock seen behind is a missed dock.

The picture never declares success (physical_docking.md): while the dock is
ahead the approach keeps closing, and only dock contact completes it. This
gate keeps its arming and aft-frame suppression, which matters for
`ideal_360`, where an aft dock stays visible; two aft frames after arming end
the approach as a miss (NAV_FAIL: missed_dock), never as a pass.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

from navpy.core.ports.frame import FinalApproachVisionFrame

# Arm only once the dock is clearly ahead: the frame-local LOS within this
# angle of the nose. From the waiting circle the dock starts about 90 deg
# abeam, so arming on any body_x > 0 let one small turn read as "ahead, then
# behind" 630 m out (.logs/a1-measure-b, uav_1). 60 deg keeps the abeam start
# 30 deg outside the cone, while a closing approach (LOS a few deg off the
# nose in the a1 terminal phase) arms well inside it.
PASS_ARM_MAX_OFF_NOSE_DEG = 60.0
PASS_ARM_MIN_BODY_X = math.cos(math.radians(PASS_ARM_MAX_OFF_NOSE_DEG))


@dataclass(frozen=True)
class VisualPassState:
    identity: tuple[str, int, int, int] | None = None
    armed: bool = False
    aft_pending: bool = False
    missed: bool = False


@dataclass(frozen=True)
class VisualPassPlan:
    suppress_command: bool
    next_state: VisualPassState


class VisualPassDetector:
    """Arm on a clearly-ahead frame; two aft frames after arming are a miss."""

    def __init__(self) -> None:
        self._state = VisualPassState()

    @property
    def missed(self) -> bool:
        return self._state.missed

    def reset(self) -> None:
        self._state = VisualPassState()

    def plan(self, frame: FinalApproachVisionFrame) -> VisualPassPlan:
        state = self._state
        key = frame.continuity_key
        if state.identity != key:
            state = VisualPassState(identity=key)
        if state.missed:
            return VisualPassPlan(True, state)
        if frame.body_x > 0.0:
            armed = state.armed or frame.body_x >= PASS_ARM_MIN_BODY_X
            return VisualPassPlan(False, VisualPassState(key, armed))
        if not state.armed:
            return VisualPassPlan(False, state)
        return VisualPassPlan(
            True,
            replace(state, aft_pending=True, missed=state.aft_pending),
        )

    def commit(self, plan: VisualPassPlan) -> None:
        self._state = plan.next_state


__all__ = [
    "PASS_ARM_MAX_OFF_NOSE_DEG",
    "VisualPassDetector",
    "VisualPassPlan",
    "VisualPassState",
]
