"""Companion heartbeat pacing in aircraft time (docs/design/navpy_recovery.md, step 7).

The autopilot's NavPy watchdog (scripts/lua/nav_watchdog.lua) times NavPy's heartbeat on its own clock, with a fixed
timeout. So the heartbeat period is aircraft time: the wall wait is the period divided by the SITL's ``SIM_SPEEDUP``
(absent on real hardware, so 1). The speed-up is read from the parameter cache, which every PARAM_VALUE updates, so
it follows a change at run time (NAV's own speed-up lease, an operator's param set): the update wakes the wait, the
heartbeat goes out at once and the next period runs at the new rate, so no gap is ever paced at the old one. A SITL
running slower than its ``SIM_SPEEDUP`` only brings the heartbeats closer in aircraft time, never further apart.
Lockstep runs its SITL at speed-up 1 on a virtual clock, which is aircraft time already.

The heartbeat does not follow the aircraft-time samples NavPy receives: a stalled downlink would then stop it, and
the autopilot would take over from a NavPy that still commands over the uplink and holds by itself.
"""

from __future__ import annotations

import math
from collections.abc import Callable

from navpy.core.kernel import runtime_time as rt

SIM_SPEEDUP_PARAM = "SIM_SPEEDUP"


def _speedup(value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return 1.0
    value = float(value)
    return value if math.isfinite(value) and value > 0.0 else 1.0


class HeartbeatPacing:
    def __init__(
        self,
        read_speedup: Callable[[], object] = lambda: None,
    ) -> None:
        """``read_speedup``: the cached ``SIM_SPEEDUP`` (None when absent: real hardware). ``wake`` must be called
        when it changes, and on stop."""
        self._read_speedup = read_speedup
        self._wake = rt.Event()

    def wake(self) -> None:
        self._wake.set()

    def wait(self, stop: rt.Event, period_s: float) -> None:
        """Return once ``period_s`` of aircraft time has passed, or early when the speed-up changes or ``stop`` is
        set (the caller then sends at once, or ends)."""
        self._wake.clear()              # before the read: a change after it wakes this wait
        if stop.is_set():
            return
        speedup = _speedup(self._read_speedup())
        self._wake.wait(period_s / speedup)


__all__ = ["HeartbeatPacing", "SIM_SPEEDUP_PARAM"]
