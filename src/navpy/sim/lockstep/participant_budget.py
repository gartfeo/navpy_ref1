"""Process freeze and CPU budget: a NavPy participant's handoffs per tick (dev sim step 4).

Step 4 of ``docs/design/deterministic_dev_sim.md``. Both models cap the baton
handoffs of one tick (``BatonRuntime.tick(handoffs=...)``); compute still takes
no sim time, so a starved process is one that gets fewer handoffs:

- ``process_freeze`` (a GC pass, a host stall): from a freeze tick the
  participant runs no thread for ``min_ticks``-``max_ticks`` ticks. The next
  freeze is due ``min_gap_s``-``max_gap_s`` of sim time after this one was due
  (the first that long after the participant's first tick) and starts on the
  first tick at or after it, so tick rounding does not add up. Its inputs
  still queue and sim time goes on. On resume, work already runnable runs
  first, then the work the queued inputs woke, in tick order, then the timers
  that came due, in ``(due, park seq)`` order with their deadlines unchanged.
- ``cpu_budget`` (a host under load): at most ``handoffs`` handoffs per tick;
  the rest stay runnable, and due timers stay due, for the next tick.

A participant reports its end only from a tick that leaves nothing runnable
or due, so work a cap left at the flush still runs, and its faults still count.

The swarm draw turns each model on per seed (``sim_profile.realize``); each
freeze's gap and length come from the model stream
``participant.<ordinal>.<generation>``, a gap then a length per freeze. The
draw reaches a NavPy role through its role config (:data:`SIM_MODELS_KEY`), so it
is a run input in the manifest. A model that is off sets no key; with none the
loop runs as before step 4. The same key carries the sim camera's model
(``camera_frames``, step 5), which the NavPy host applies, not this loop.
"""

from __future__ import annotations

import math
import random
from collections.abc import Mapping

from navpy.sim.lockstep.camera_frames import CAMERA_FRAMES, camera_frame_factory
from navpy.sim.lockstep.sim_profile import Number, Realization, model_stream

PROCESS_FREEZE = "process_freeze"
CPU_BUDGET = "cpu_budget"
MODELS = (CPU_BUDGET, PROCESS_FREEZE)
#: Every model a NavPy's role config may carry: these two and the sim camera's (``camera_frames``).
PARTICIPANT_MODELS = (CAMERA_FRAMES, *MODELS)
#: The role config key of the on participant models and their drawn parameters.
SIM_MODELS_KEY = "sim_models"


def _int(params: Mapping[str, Number], name: str, low: int) -> int:
    value = params[name]
    if isinstance(value, bool) or not isinstance(value, int) or value < low:
        raise ValueError(f"{name} must be an int of at least {low}, got {value!r}")
    return value


def _seconds(params: Mapping[str, Number], name: str) -> float:
    value = params[name]
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not (
            math.isfinite(value) and value > 0):
        raise ValueError(f"{name} must be a finite number above 0, got {value!r}")
    return float(value)


def _bounds(low: Number, high: Number, what: str) -> None:
    if low > high:
        raise ValueError(f"{what}: the minimum {low} is above the maximum {high}")


class FreezeSchedule:
    """One participant's freezes, asked once per tick in tick order."""

    def __init__(self, params: Mapping[str, Number], stream: random.Random) -> None:
        self._min_ticks = _int(params, "min_ticks", 1)
        self._max_ticks = _int(params, "max_ticks", 1)
        self._min_gap_s = _seconds(params, "min_gap_s")
        self._max_gap_s = _seconds(params, "max_gap_s")
        _bounds(self._min_ticks, self._max_ticks, "freeze ticks")
        _bounds(self._min_gap_s, self._max_gap_s, "freeze gap")
        self._stream = stream
        self._next_us: int | None = None
        self._left = 0

    def _gap_us(self) -> int:
        return round(1e6 * (self._min_gap_s + (self._max_gap_s - self._min_gap_s) * self._stream.random()))

    def _ticks(self) -> int:
        span = self._max_ticks - self._min_ticks
        return self._min_ticks + min(span, math.floor(self._stream.random() * (span + 1)))

    def frozen(self, t_us: int) -> bool:
        """Whether the tick at ``t_us`` runs no thread."""
        if self._next_us is None:
            self._next_us = t_us + self._gap_us()
        if self._left:
            self._left -= 1
            return True
        if t_us < self._next_us:
            return False
        self._left = self._ticks() - 1
        self._next_us += self._gap_us()
        return True


class TickBudget:
    """A participant's handoff cap per tick: 0 in a freeze, else its CPU budget, else none."""

    def __init__(self, models: Mapping[str, Mapping[str, Number]], seed: int, ordinal: int,
                 generation: int) -> None:
        unknown = sorted(set(models) - set(PARTICIPANT_MODELS))
        if unknown:
            raise ValueError(f"unknown participant models {unknown}; known: {sorted(PARTICIPANT_MODELS)}")
        freeze = models.get(PROCESS_FREEZE)
        self._freeze = None if freeze is None else FreezeSchedule(
            freeze, model_stream(seed, PROCESS_FREEZE, f"participant.{ordinal}.{generation}"))
        cpu = models.get(CPU_BUDGET)
        self._handoffs = None if cpu is None else _int(cpu, "handoffs", 1)

    def handoffs(self, t_us: int) -> int | None:
        """The cap of the tick at ``t_us``; called once per tick, in tick order."""
        if self._freeze is not None and self._freeze.frozen(t_us):
            return 0
        return self._handoffs


def participant_models(realization: Realization) -> dict[str, dict[str, Number]]:
    """The NavPy role config's :data:`SIM_MODELS_KEY` entry: each on model and its draw.

    Checked here, so a bad profile refuses the run before anything starts."""
    models = {name: dict(draw.params) for name in PARTICIPANT_MODELS
              if (draw := realization.model(name)) is not None and draw.on}
    TickBudget(models, realization.seed, 1, 1)
    camera_frame_factory(models, realization.seed, 1, 1)
    return models


__all__ = [
    "SIM_MODELS_KEY", "CPU_BUDGET", "FreezeSchedule", "MODELS", "PARTICIPANT_MODELS", "PROCESS_FREEZE",
    "TickBudget",
    "participant_models",
]
