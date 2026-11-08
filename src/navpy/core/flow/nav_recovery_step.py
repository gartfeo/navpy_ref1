"""NAV recovery's step, shared by the NAV loop and the in-process watchdog (docs/design/navpy_recovery.md, step 6).

The NAV loop steps recovery each period. While the NAV loop itself is stuck, the watchdog thread steps the same
recovery in its place (nav_watchdog.py): one stage machine, one mode request and one hold deadline, whoever steps.
The lock makes the steps one at a time; the watchdog waits for it only so long. A step never blocks (its mode
writes are the mode sender's), and its time sample is taken under the lock, so steps see time in order.
"""

from __future__ import annotations

from collections.abc import Callable

from navpy.core.flow.nav_mode_sender import ModeSender
from navpy.core.flow.nav_recovery import NavRecovery
from navpy.core.flow.nav_recovery_clock import TimeSample
from navpy.core.kernel import runtime_time as rt


class SharedRecoveryStep:
    def __init__(self, recovery: NavRecovery, sender: ModeSender | None = None) -> None:
        self.recovery = recovery
        #: Recovery's mode sender thread (nav_mode_sender.py), started and stopped with the watchdog.
        self.sender = sender
        self._lock = rt.Lock()

    def step(
        self, cycle_ok: bool, sample: Callable[[], TimeSample], *, lock_timeout_s: float | None = None,
    ) -> bool:
        """False when ``lock_timeout_s`` passed before the step could run. ``sample`` is read under the lock: a
        sample taken before waiting for it could be older than the other thread's last one."""
        if not self._lock.acquire(timeout=-1 if lock_timeout_s is None else lock_timeout_s):
            return False
        try:
            self.recovery.step(cycle_ok, sample())
        finally:
            self._lock.release()
        return True


__all__ = ["SharedRecoveryStep"]
