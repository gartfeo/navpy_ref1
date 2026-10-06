"""Vehicle actuation transaction for legacy final approach."""

from __future__ import annotations

import math
from collections.abc import Callable

from navpy.modules.navigation.legacy_final_approach_calculation import (
    LegacyFinalApproachCalculation,
)


class LegacyFinalApproachActuation:
    """Issue one calculated roll, pitch, and throttle command atomically."""

    def __init__(self, set_attitude: Callable[..., object]) -> None:
        self._set_attitude = set_attitude

    def execute(self, calculation: LegacyFinalApproachCalculation) -> None:
        cmd_roll = calculation.cmd_roll_deg
        cmd_pitch = calculation.cmd_pitch_deg
        self._set_attitude(
            math.radians(cmd_roll) if cmd_roll is not None else None,
            math.radians(cmd_pitch) if cmd_pitch is not None else None,
            thr=calculation.cmd_throttle,
        )


__all__ = ["LegacyFinalApproachActuation"]
