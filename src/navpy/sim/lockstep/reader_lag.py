"""Reader lag: a NavPy's MAVLink readers fall behind while it keeps running (dev sim step 9).

Step 9 of ``docs/design/deterministic_dev_sim.md``. Under host load a wall
NavPy's reader can fall behind for tens of seconds while NAV keeps running on
late poses (todo 65, ``.logs/dds-step9``); ``process_freeze`` stops every
thread instead. The ``reader_lag`` model gives a NavPy lag episodes:

- an episode's start is due ``min_gap_s``-``max_gap_s`` of sim time after the
  previous one was due (the first that long after sim time 0) and starts then,
  or when the previous one ends if that is later;
- its delay rises linearly from 0 to ``min_peak_ticks``-``max_peak_ticks``
  ticks over ``min_rise_s``-``max_rise_s``, then falls linearly to 0 over
  ``min_drain_s``-``max_drain_s``;
- each draws a gap, a peak, a rise and a drain, in that order, from the model
  stream ``participant.<ordinal>.<generation>``.

Every lockstep bus reader of the NavPy dispatches a packet no earlier than its
delivery time plus the delay then (:meth:`ReaderLag.due_us`), in order, waiting
on a baton timer while the NavPy's other threads run. The schedule is a
function of sim time alone, so the buses may ask in any order.
"""

from __future__ import annotations

import math
import random
from collections.abc import Mapping
from dataclasses import dataclass

from navpy.sim.lockstep.sim_profile import Number

READER_LAG = "reader_lag"
TICK_US = 20_000


@dataclass(frozen=True)
class LagEpisode:
    start_us: int
    peak_us: int
    rise_us: int
    drain_us: int

    @property
    def end_us(self) -> int:
        return self.start_us + self.rise_us + self.drain_us

    def delay_us(self, t_us: int) -> int:
        if t_us <= self.start_us or t_us >= self.end_us:
            return 0
        if t_us <= self.start_us + self.rise_us:
            return round(self.peak_us * (t_us - self.start_us) / self.rise_us)
        return round(self.peak_us * (self.end_us - t_us) / self.drain_us)


def _range(params: Mapping[str, Number], low: str, high: str, floor: float) -> tuple[float, float]:
    lo, hi = params[low], params[high]
    for name, value in ((low, lo), (high, hi)):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not (
                math.isfinite(value) and value > floor):
            raise ValueError(f"{name} must be a finite number above {floor}, got {value!r}")
    if lo > hi:
        raise ValueError(f"{low} {lo} is above {high} {hi}")
    return float(lo), float(hi)


class ReaderLag:
    """One NavPy's lag episodes, drawn as far as asked."""

    def __init__(self, params: Mapping[str, Number], stream: random.Random) -> None:
        for name in ("min_peak_ticks", "max_peak_ticks"):
            if isinstance(params[name], bool) or not isinstance(params[name], int):
                raise ValueError(f"{name} must be an int, got {params[name]!r}")
        self._peak = _range(params, "min_peak_ticks", "max_peak_ticks", 0)
        self._rise = _range(params, "min_rise_s", "max_rise_s", 0)
        self._drain = _range(params, "min_drain_s", "max_drain_s", 0)
        self._gap = _range(params, "min_gap_s", "max_gap_s", 0)
        self._stream = stream
        self._episodes: list[LagEpisode] = []
        self._due_us = 0

    def _uniform(self, bounds: tuple[float, float]) -> float:
        return bounds[0] + (bounds[1] - bounds[0]) * self._stream.random()

    def _draw(self) -> None:
        self._due_us += round(1e6 * self._uniform(self._gap))
        lo, hi = int(self._peak[0]), int(self._peak[1])
        peak = lo + min(hi - lo, math.floor(self._stream.random() * (hi - lo + 1)))
        rise, drain = round(1e6 * self._uniform(self._rise)), round(1e6 * self._uniform(self._drain))
        start = max(self._due_us, self._episodes[-1].end_us if self._episodes else 0)
        self._episodes.append(LagEpisode(start, peak * TICK_US, rise, drain))

    def delay_us(self, t_us: int) -> int:
        """The delay at sim time ``t_us``."""
        while not self._episodes or self._episodes[-1].start_us <= t_us:
            self._draw()
        return max(episode.delay_us(t_us) for episode in self._episodes)

    def due_us(self, delivered_us: int) -> int:
        """When a packet delivered at ``delivered_us`` may be dispatched."""
        return delivered_us + self.delay_us(delivered_us)


__all__ = ["LagEpisode", "READER_LAG", "ReaderLag"]
