"""Recovery's clock: aircraft time while it advances, wall time while it stalls (docs/design/navpy_recovery.md).

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


__all__ = ["AircraftTimeClock", "RecoveryClock", "TimeSample", "attitude_boot_s"]
