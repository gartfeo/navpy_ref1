"""Fixed physics steps owed to a simulated gimbal by elapsed aircraft time.

The gimbal runner calls its step once per wall-clock slot and never catches
up (``gimbal_cadence.py``). When that thread gets little CPU, a plant that
advanced a fixed step per call fell behind a sped-up SITL: in run
``self-orbit-a`` it ran about 7 times slower than the aircraft and lost the
dock in a banked turn. Stepping by the aircraft's own boot clock keeps the
plant in the aircraft's time: it catches up when late and waits when early.
"""

from __future__ import annotations

from typing import Optional

# Bounds the work done in one call. Time owed beyond it is kept and paid in
# later calls, never dropped: a starved runner must still catch up.
MAX_STEPS_PER_CALL = 250


class AircraftStepClock:
    """Count whole physics steps between aircraft boot-time samples."""

    def __init__(self, step_ms: int, max_steps_per_call: int = MAX_STEPS_PER_CALL) -> None:
        if step_ms <= 0 or max_steps_per_call <= 0:
            raise ValueError("step_ms and max_steps_per_call must be positive")
        self._step_ms = step_ms
        self._max_steps = max_steps_per_call
        self._last_ms: Optional[int] = None
        self._owed_ms = 0

    def steps_for(self, boot_ms: Optional[int], *, pay_all: bool = False) -> int:
        """Steps owed up to ``boot_ms``; None (no aircraft clock) owes one step.

        ``pay_all`` pays the whole debt (before a command changes the plant).
        """
        if boot_ms is None:
            return 1
        last = self._last_ms
        self._last_ms = boot_ms
        if last is None or boot_ms < last:
            # First sample, or the aircraft restarted: its clock went back.
            self._owed_ms = 0
            return 0
        self._owed_ms += boot_ms - last
        steps = self._owed_ms // self._step_ms
        if not pay_all:
            steps = min(steps, self._max_steps)
        self._owed_ms -= steps * self._step_ms
        return steps

    def settle_remainder_ms(self) -> int:
        """Hand over the part-step still owed, so a command never inherits it."""
        remainder, self._owed_ms = self._owed_ms, 0
        return remainder


__all__ = ["AircraftStepClock", "MAX_STEPS_PER_CALL"]
