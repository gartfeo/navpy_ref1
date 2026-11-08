"""In-process watchdog for a stuck NAV loop or command worker (docs/design/navpy_recovery.md, step 6).

A thread of its own reads the two loops' progress beacons (core/kernel/progress_beacon.py). A pass that stays busy
for NAV_HANG_S of wall time, beyond the bounded waits it declared (a parameter reply, a mission transfer), is stuck.
A hang is the software's liveness, so it is measured in wall time; the flight timers it leads to (the hold's 30 s,
mode confirmation) are NAV recovery's, in aircraft time.

While a loop is stuck the watchdog:
- holds the command fence closed (MavCommandFence.hold_closed, the fence lock only, so a stuck sender cannot
  block it; no reopen opens it while held, whichever thread races) and sets WatchdogSignal.hung, which NAV
  recovery reads in its step: a hung period fails, and holds at once (LOITER if NavPy owns the mode, RTL after
  NAV_HOLD_RECOVERY_S);
- when the NAV loop itself is stuck, so cannot step recovery, steps that same recovery in its place
  (NavRecovery.step with a bounded wait for its lock). There is one stage machine, one mode request and one hold
  deadline, whichever thread steps; nothing is handed over when the hang ends.
It never touches NAV state (task, phase, decision). The ArduPilot Lua watchdog (step 7) covers a NavPy that is dead,
since this one dies with it.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from navpy.core.flow.nav_recovery_clock import TimeSample
from navpy.core.flow.nav_recovery_ports import WatchdogSignal
from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.progress_beacon import ProgressBeacon

#: A pass busy this long in wall time, beyond its declared bounded waits, is stuck. NAV's period is 20 ms and its
#: cycle has no other wait; the hold trigger's 1.5 s (ArduPlane's FS_SHORT_TIMEOUT) leaves room for a slow cycle.
NAV_HANG_S = 1.5
#: Wall time between checks: 30 within NAV_HANG_S.
NAV_WATCHDOG_TICK_S = 0.05
#: How long the watchdog waits for recovery's lock to step in the stuck NAV loop's place: a step never blocks (its
#: mode writes are the mode sender's), so a second is ample; one not taken is reported, and tried next check.
NAV_WATCHDOG_STEP_LOCK_S = 1.0
NAV_WATCHDOG_JOIN_TIMEOUT_S = 2.0


class _HangTracker:
    def __init__(self, beacon: ProgressBeacon) -> None:
        self._beacon = beacon
        self._passes = -1
        self._busy_since: float | None = None

    def hung(self, now: float) -> bool:
        reading = self._beacon.reading()
        if not reading.busy:
            self._busy_since = None
            return False
        if reading.passes != self._passes or self._busy_since is None:
            self._passes, self._busy_since = reading.passes, now
        return now - self._busy_since >= NAV_HANG_S + reading.allowance_s


@dataclass(frozen=True)
class NavWatchdogPorts:
    nav_progress: ProgressBeacon
    command_progress: ProgressBeacon
    signal: WatchdogSignal
    #: MavCommandFence.hold_closed: held while a loop is stuck, so no reopen opens it, whichever thread races.
    hold_fence: Callable[[bool], object]
    #: NAV recovery's step (NavRecovery.step): (cycle_ok, sample, lock_timeout_s) -> stepped; None without recovery.
    step_recovery: Callable[..., bool] | None
    #: Recovery's time sample (aircraft and wall time), as the NAV loop takes it.
    sample: Callable[[], TimeSample]
    report: Callable[[str], None]
    wall_s: Callable[[], float] = rt.monotonic


class NavWatchdog:
    """``check`` runs on the watchdog thread only (``start``/``stop``)."""

    def __init__(self, ports: NavWatchdogPorts) -> None:
        self._ports = ports
        self._nav = _HangTracker(ports.nav_progress)
        self._worker = _HangTracker(ports.command_progress)
        self._stuck: str | None = None      # what is reported stuck now
        self._step_missed = False
        self._stop = rt.Event()
        self._thread: rt.Thread | None = None

    def check(self) -> None:
        now = self._ports.wall_s()
        nav_hung = self._nav.hung(now)
        worker_hung = self._worker.hung(now)
        stuck = "NAV loop" if nav_hung else "command worker" if worker_hung else None
        # The veto first: from here no reopen opens the fence, even one recovery decided before this check.
        self._ports.hold_fence(stuck is not None)
        self._ports.signal.set_hung(stuck is not None)
        if stuck != self._stuck:
            self._report(f"NavPy {stuck} stuck: commands fenced" if stuck else "NavPy loops progressing again")
            self._stuck = stuck
        if nav_hung and self._ports.step_recovery is not None:
            stepped = self._ports.step_recovery(False, self._ports.sample, lock_timeout_s=NAV_WATCHDOG_STEP_LOCK_S)
            if not stepped and not self._step_missed:
                self._report("NavPy recovery step busy: the watchdog could not step in for the NAV loop")
            self._step_missed = not stepped

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = rt.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """First in shutdown: the loops' joins must not read as a hang."""
        self._stop.set()
        thread, self._thread = self._thread, None
        if thread is None:
            return
        thread.join(timeout=NAV_WATCHDOG_JOIN_TIMEOUT_S)
        if thread.is_alive():
            raise RuntimeError("NAV watchdog thread did not stop")

    def _run(self) -> None:
        while not self._stop.wait(NAV_WATCHDOG_TICK_S):
            try:
                self.check()
            except Exception as error:  # noqa: BLE001 - the watchdog must outlive what it watches
                self._report(f"NAV watchdog check failed: {error!r}")

    def _report(self, message: str) -> None:
        try:
            self._ports.report(message)
        except Exception:  # noqa: BLE001 - reporting must not stop the watchdog
            pass


__all__ = ["NAV_HANG_S", "NAV_WATCHDOG_TICK_S", "NavWatchdog", "NavWatchdogPorts"]
