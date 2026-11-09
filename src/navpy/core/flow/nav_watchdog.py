"""In-process watchdog for a stuck NAV loop or command worker (docs/design/navpy_recovery.md, step 6).

A thread of its own reads the two loops' progress beacons (core/kernel/progress_beacon.py). A pass that stays busy
for NAV_HANG_S, beyond the bounded waits it declared (a parameter reply, a mission transfer), is stuck. It is
measured in aircraft time, like the flight timers it leads to (owner ruling): at a sim speed-up it trips at the same
aircraft time. Wall time is only the backstop while aircraft time is frozen (HangClock), so a stalled link or a
frozen SITL still trips.

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

from navpy.core.flow.nav_recovery_clock import HangClock, TimeSample
from navpy.core.flow.nav_recovery_ports import WatchdogSignal
from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.progress_beacon import ProgressBeacon

#: A pass busy this long (HangClock: aircraft time, wall time while it is frozen), beyond its declared bounded waits, is stuck. NAV's period is 20 ms and its
#: cycle has no other wait; the hold trigger's 1.5 s (ArduPlane's FS_SHORT_TIMEOUT) leaves room for a slow cycle.
NAV_HANG_S = 1.5
#: Wall time between checks: 30 within NAV_HANG_S at speed-up 1.
NAV_WATCHDOG_TICK_S = 0.05
#: A check whose wall time since the previous check exceeds NAV_WATCHDOG_TICK_S by more than this woke late: the
#: watchdog thread itself did not run, as in a whole-process or host stall, so the loops it watches very likely did
#: not either, and their busy time measures the stall, not a hang. A late check gives no new hang verdict (it keeps
#: one that already holds); the next on-time check decides, so a loop still busy then is a hang (owner ruling, step
#: 10 item 2: the step 9 SITL saw a false "NAV loop stuck" on all three NavPys at once for about 1.7 s). Five ticks:
#: thread wake-up jitter under load is tens of milliseconds, and a real hang still trips within NAV_HANG_S plus one
#: tick of on-time checks.
NAV_WATCHDOG_LATE_S = 0.25
#: How long the watchdog waits for recovery's lock to step in the stuck NAV loop's place: a step never blocks (its
#: mode writes are the mode sender's), so a second is ample; one not taken is reported, and tried next check.
NAV_WATCHDOG_STEP_LOCK_S = 1.0
NAV_WATCHDOG_JOIN_TIMEOUT_S = 2.0


class _HangTracker:
    def __init__(self, beacon: ProgressBeacon) -> None:
        self._beacon = beacon
        self._passes = -1
        self._busy_since: tuple[float, float] | None = None   # (hang clock, wall) at the pass's start
        self._hung = False

    def hung(self, now: float, wall_s: float, late: bool) -> bool:
        """The verdict of a check: a hang (``_busy``), but from a ``late`` check only if it already held
        (NAV_WATCHDOG_LATE_S)."""
        busy = self._busy(now, wall_s)
        self._hung = busy and (self._hung or not late)
        return self._hung

    def _busy(self, now: float, wall_s: float) -> bool:
        """The pass's busy time, less its declared bounded waits, has reached NAV_HANG_S on the hang clock.

        The waits are wall-time bounds (a parameter reply), so they are taken as their share of the pass's wall time,
        converted at the hang clock's rate over the pass (step 10, item 10): at a sim speed-up k a declared wait of
        W s covers k x W s of aircraft time, not W s, and a parameter exchange in fast SITL is not a hang. At speed-up
        1, or with aircraft time frozen (wall time carries the hang clock), this is the plain NAV_HANG_S + W."""
        reading = self._beacon.reading()
        if not reading.busy:
            self._busy_since = None
            return False
        if reading.passes != self._passes or self._busy_since is None:
            self._passes, self._busy_since = reading.passes, (now, wall_s)
        elapsed, wall_elapsed = now - self._busy_since[0], wall_s - self._busy_since[1]
        if reading.allowance_s <= 0.0:
            return elapsed >= NAV_HANG_S
        if wall_elapsed <= reading.allowance_s:
            return False
        return elapsed - reading.allowance_s * (elapsed / wall_elapsed) >= NAV_HANG_S


@dataclass(frozen=True)
class NavWatchdogPorts:
    nav_progress: ProgressBeacon
    command_progress: ProgressBeacon
    signal: WatchdogSignal
    #: MavCommandFence.hold_closed: held while a loop is stuck, so no reopen opens it, whichever thread races.
    hold_fence: Callable[[bool], object]
    #: NAV recovery's step (NavRecovery.step): (cycle_ok, sample, lock_timeout_s) -> stepped; None without recovery.
    step_recovery: Callable[..., bool] | None
    #: Recovery's time sample (aircraft and wall time), as the NAV loop takes it; also the hang clock's.
    sample: Callable[[], TimeSample]
    report: Callable[[str], None]


class NavWatchdog:
    """``check`` runs on the watchdog thread only (``start``/``stop``)."""

    def __init__(self, ports: NavWatchdogPorts) -> None:
        self._ports = ports
        self._nav = _HangTracker(ports.nav_progress)
        self._worker = _HangTracker(ports.command_progress)
        self._clock = HangClock()
        self._stuck: str | None = None      # what is reported stuck now
        self._step_missed = False
        self._last_check_wall_s: float | None = None
        self._stop = rt.Event()
        self._thread: rt.Thread | None = None

    def check(self) -> None:
        sample = self._ports.sample()
        now = self._clock.advance(sample)
        last, self._last_check_wall_s = self._last_check_wall_s, sample.wall_s
        late = last is not None and sample.wall_s - last > NAV_WATCHDOG_TICK_S + NAV_WATCHDOG_LATE_S
        nav_hung = self._nav.hung(now, sample.wall_s, late)
        worker_hung = self._worker.hung(now, sample.wall_s, late)
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


__all__ = ["NAV_HANG_S", "NAV_WATCHDOG_LATE_S", "NAV_WATCHDOG_TICK_S", "NavWatchdog", "NavWatchdogPorts"]
