"""NAV recovery's step, shared by the NAV loop and the in-process watchdog (docs/design/navpy_recovery.md, step 6).

The NAV loop steps recovery each period. While the NAV loop itself is stuck, the watchdog thread steps the same
recovery in its place (nav_watchdog.py): one stage machine, one mode request and one hold deadline, whoever steps.
The lock makes the steps one at a time; the watchdog waits for it only so long, since a step holds it at most for
its mode send's bounded wait (nav_mode_request.MODE_SEND_LOCK_TIMEOUT_S).
"""

from __future__ import annotations

from navpy.core.flow.nav_recovery import NavRecovery
from navpy.core.flow.nav_recovery_clock import TimeSample
from navpy.core.kernel import runtime_time as rt


class SharedRecoveryStep:
    def __init__(self, recovery: NavRecovery) -> None:
        self.recovery = recovery
        self._lock = rt.Lock()

    def step(self, cycle_ok: bool, sample: TimeSample, *, lock_timeout_s: float | None = None) -> bool:
        """False when ``lock_timeout_s`` passed before the step could run."""
        if not self._lock.acquire(timeout=-1 if lock_timeout_s is None else lock_timeout_s):
            return False
        try:
            self.recovery.step(cycle_ok, sample)
        finally:
            self._lock.release()
        return True


__all__ = ["SharedRecoveryStep"]
