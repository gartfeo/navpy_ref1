"""Actuator-form NavLaw facade around legacy Roll-L1 navigation."""

from __future__ import annotations

from navpy.modules.navigation.nav.nav_law import (
    NavCommand,
    NavCommandMode,
    NavContext,
    NavLaw,
)
from navpy.modules.navigation.nav.roll_l1_core import RollL1PitchNav


class RollL1PitchNavLaw(NavLaw):
    """Adapt a narrow RollL1PitchNav owner to the NavLaw interface."""

    def __init__(self, inner: RollL1PitchNav) -> None:
        self._inner = inner

    def calc(self, ctx: NavContext) -> NavCommand:
        cmd_roll, cmd_pitch, cmd_thr = self._inner.calc(
            prev_loc=ctx.prev_loc,
            current_loc=ctx.current_loc,
            next_loc=ctx.next_loc,
            target_bearing_cd=ctx.target_bearing_cd,
            poi_ned=ctx.poi_ned,
            pitch_error=ctx.pitch_error,
            distance=ctx.distance,
        )
        return NavCommand(
            mode=NavCommandMode.ACTUATOR,
            cmd_roll_deg=cmd_roll,
            cmd_pitch_deg=cmd_pitch,
            cmd_thr=cmd_thr,
        )

    def reset(self) -> None:
        self._inner.reset()


__all__ = ["RollL1PitchNavLaw"]
