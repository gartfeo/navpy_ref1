"""Recovery's clocks: aircraft time while it advances, wall time while it stalls (docs/design/navpy_recovery.md).

Timers compare readings of one clock that only moves forward. Each step adds the aircraft-time delta when aircraft
time advanced, else the wall delta, so a link that freezes aircraft time after it moved still lets the hold and the
deadlines expire. When aircraft time moves again after a stall, the stall's wall time is not counted twice: the clock
moves to the aircraft-time position measured from the last aircraft step, or stays if wall time already passed it.
A reset of aircraft time (autopilot reboot) restarts the aircraft anchor; the clock never goes back.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from navpy.core.kernel import runtime_time as rt


@dataclass(frozen=True)
class TimeSample:
    aircraft_s: float | None
    wall_s: float


class RecoveryClock:
    def __init__(self) -> None:
        self.now_s = 0.0
        self._wall_s: float | None = None
        self._aircraft_s: float | None = None   # aircraft time at the last anchor
        self._anchor_now_s = 0.0                # the clock's reading at that anchor

    def advance(self, sample: TimeSample) -> float:
        """The clock's reading for ``sample``, in seconds since the first sample."""
        aircraft = sample.aircraft_s
        if self._wall_s is not None:
            if aircraft is not None and self._aircraft_s is not None and aircraft > self._aircraft_s:
                self.now_s = max(self.now_s, self._anchor_now_s + aircraft - self._aircraft_s)
            else:
                self.now_s += max(sample.wall_s - self._wall_s, 0.0)
        if aircraft is not None and (self._aircraft_s is None or aircraft != self._aircraft_s):
            self._aircraft_s, self._anchor_now_s = aircraft, self.now_s
        self._wall_s = sample.wall_s
        return self.now_s


#: Wall time without a new aircraft-time sample after which aircraft time counts as frozen (a stalled link, a paused
#: SITL), and wall time carries the hang clock. ATTITUDE streams at several Hz, so half a second is several missed
#: samples; the frozen stretch is counted from the last sample, so the hang threshold itself is not delayed.
AIRCRAFT_TIME_STALE_S = 0.5


class HangClock:
    """The watchdog's clock (owner ruling on step 6, finding 7): aircraft time while fresh samples advance it, so a
    hang trips at the same aircraft time at any sim speed-up; wall time only while aircraft time is frozen for
    AIRCRAFT_TIME_STALE_S, counted from its last sample, so a stalled link or a frozen SITL still trips. A frozen
    stretch is counted once: when aircraft time moves again, it re-anchors there without adding its jump. A reset
    (autopilot reboot) re-anchors too; the clock never goes back."""

    def __init__(self) -> None:
        self.now_s = 0.0
        self._aircraft_s: float | None = None
        self._fresh_wall_s: float | None = None   # wall time of the last aircraft-time sample (or the first sample)
        self._frozen_to_s: float | None = None    # while frozen: wall time counted up to

    def advance(self, sample: TimeSample) -> float:
        aircraft, wall = sample.aircraft_s, sample.wall_s
        if self._fresh_wall_s is None:
            self._fresh_wall_s = wall
        if aircraft is not None and aircraft != self._aircraft_s:
            if self._frozen_to_s is None and self._aircraft_s is not None and aircraft > self._aircraft_s:
                self.now_s += aircraft - self._aircraft_s
            self._aircraft_s, self._fresh_wall_s, self._frozen_to_s = aircraft, wall, None
        elif wall - self._fresh_wall_s >= AIRCRAFT_TIME_STALE_S:
            counted_to = self._fresh_wall_s if self._frozen_to_s is None else self._frozen_to_s
            self.now_s += max(wall - counted_to, 0.0)
            self._frozen_to_s = max(wall, counted_to)
        return self.now_s


def attitude_boot_s(vehicle: object) -> float | None:
    """Latest ATTITUDE boot time (the observation clock), or None without one."""
    sample = getattr(vehicle, "attitude_sample", None)
    value = getattr(sample, "time_boot_s", None)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value)


class AircraftTimeClock:
    """A RecoveryClock read as a plain clock (``clock()`` -> seconds): aircraft time while it advances, wall time
    while it stalls. For timers outside the NAV loop, such as component restarts (core/kernel/restart_budget.py)."""

    def __init__(
        self,
        read_aircraft_s: Callable[[], float | None],
        wall_s: Callable[[], float] = rt.monotonic,
    ) -> None:
        self._read_aircraft_s = read_aircraft_s
        self._wall_s = wall_s
        self._clock = RecoveryClock()
        self._lock = rt.Lock()

    def __call__(self) -> float:
        try:
            aircraft_s = self._read_aircraft_s()
        except Exception:  # noqa: BLE001 - no aircraft time: wall time carries the clock
            aircraft_s = None
        with self._lock:
            return self._clock.advance(TimeSample(aircraft_s, self._wall_s()))


__all__ = ["AIRCRAFT_TIME_STALE_S", "AircraftTimeClock", "HangClock", "RecoveryClock", "TimeSample", "attitude_boot_s"]
