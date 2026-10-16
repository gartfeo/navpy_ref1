"""The bridge's simple launches (--launch launcher, --launch ground): a force on the body x axis.

The first launcher of the HITL chain, kept to compare with the catapult rail (rail.py, the
default): the aircraft waits at rest at a set pitch and height (plant_options.launcher_pose),
then the plant's "catapult" external force pushes it along its body x axis for a set time, with
the model's own aerodynamics and gravity acting; nothing guides it along a rail. On its skids
(--launch ground) the same force pushes it along the ground from its rest pose.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from tools.tuning.hitl_bridge.environment import US_PER_S, Wind
from tools.tuning.hitl_bridge.physics import PlantState
from tools.tuning.hitl_bridge.state_fields import on_launcher


@dataclass
class Catapult:
    """physics.Launch: a force on the body x axis for duration_s, delay_s after the armed bit first appears.

    holds: the catapult is a launcher that holds the aircraft at rest in its start pose until
    the stroke starts; the physics is not stepped before that. Without it the aircraft rests on
    its own contacts (the ground) from the start.
    """
    force_n: float
    duration_s: float
    delay_s: float
    armed_at_us: Optional[int] = None
    holds: bool = False

    def note(self, board_time_us: int, armed: bool) -> None:
        if armed and self.armed_at_us is None:
            self.armed_at_us = board_time_us

    def reset(self) -> None:
        self.armed_at_us = None

    def _stroke_start_us(self) -> Optional[float]:
        return None if self.armed_at_us is None else self.armed_at_us + self.delay_s * US_PER_S

    def force_at(self, board_time_us: float) -> float:
        start_us = self._stroke_start_us()
        if start_us is None:
            return 0.0
        end_us = start_us + self.duration_s * US_PER_S
        return self.force_n if start_us <= board_time_us < end_us else 0.0

    def holding(self, board_time_us: float) -> bool:
        """True while the launcher holds the aircraft: before the stroke (never without holds)."""
        start_us = self._stroke_start_us()
        return self.holds and (start_us is None or board_time_us < start_us)

    def held_state(self, plant_state: PlantState, board_time_us: float, wind: Wind) -> PlantState:
        """At rest on the launcher: the accelerometer reads its support (state_fields.on_launcher)."""
        return on_launcher(plant_state)
