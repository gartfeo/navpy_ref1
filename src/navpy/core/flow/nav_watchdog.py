"""In-process watchdog for a hung NAV loop or command worker (docs/design/navpy_recovery.md, step 6).

A thread of its own reads the two loops' progress beacons (core/kernel/progress_beacon.py). A pass that stays busy
for NAV_HOLD_TRIGGER_S of recovery time is a hang. Then the watchdog:
- closes the command fence (it takes the fence lock only, so a hung sender cannot block it) and tells the NAV loop
  through WatchdogSignal that a loop is hung, so NAV counts its periods as failed (a hung worker: NAV's own hold);
- only when the NAV loop itself is hung, and so cannot hold, holds in its place under the ownership rule: it goes on
  with a copy of NAV's ownership record (what NAV published at its last step), asks for LOITER if NavPy owns the
  mode, and RTL after NAV_HOLD_RECOVERY_S in the hold, or when LOITER is not confirmed, as NAV recovery does. Its
  mode sends go through a bounded sender, so a hung send does not stop it.
When the hang ends it hands its record and what it did to the NAV loop (WatchdogSignal.report_hold), which takes the
hold over from there (nav_recovery.py). It never touches NAV state. The ArduPilot Lua watchdog (step 7) covers a NavPy
that is dead, since this one dies with it.

Times are recovery time: aircraft time while it advances, else wall time (nav_recovery_clock.AircraftTimeClock).
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from navpy.core.flow.nav_mode_request import ModeRequest
from navpy.core.flow.nav_ownership import ModeObservation, NavOwnership
from navpy.core.flow.nav_recovery import NAV_HOLD_RECOVERY_S, NAV_HOLD_TRIGGER_S
from navpy.core.flow.nav_recovery_ports import WatchdogHold, WatchdogSignal
from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.progress_beacon import ProgressBeacon
from navpy.core.ports.flight_mode import FlightMode

#: Wall time between checks: about 30 within the 1.5 s hang time at real speed.
NAV_WATCHDOG_TICK_S = 0.05
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
        return now - self._busy_since >= NAV_HOLD_TRIGGER_S


@dataclass(frozen=True)
class NavWatchdogPorts:
    nav_progress: ProgressBeacon
    command_progress: ProgressBeacon
    signal: WatchdogSignal
    close_fence: Callable[[], object]
    observe_mode: Callable[[], ModeObservation]
    #: Bounded (core/kernel/bounded_sender.py): returns even when the send hangs.
    request_mode: Callable[[FlightMode], object]
    report: Callable[[str], None]
    clock: Callable[[], float]


class NavWatchdog:
    """``check`` runs on the watchdog thread only (``start``/``stop``)."""

    def __init__(self, ports: NavWatchdogPorts) -> None:
        self._ports = ports
        self._nav = _HangTracker(ports.nav_progress)
        self._worker = _HangTracker(ports.command_progress)
        self._acting = False
        self._own: NavOwnership | None = None
        self._mode: ModeRequest | None = None
        self._hold_since: float | None = None
        self._critical = False
        self._stop = rt.Event()
        self._thread: rt.Thread | None = None

    def check(self) -> None:
        now = self._ports.clock()
        nav_hung = self._nav.hung(now)
        worker_hung = self._worker.hung(now)
        self._ports.signal.set_hung(nav_hung or worker_hung)
        if nav_hung or worker_hung:
            if not self._acting:
                self._acting = True
                self._ports.close_fence()
                self._ports.signal.mark_fenced()
                self._report(f"NavPy {'NAV loop' if nav_hung else 'command worker'} hung: commands fenced")
            if nav_hung:
                self._hold_in_place(now)
        elif self._acting:
            self._ports.signal.report_hold(WatchdogHold(self._own, self._critical))
            self._report("NavPy progressing again: NAV takes the hold over")
            self._acting, self._own, self._mode, self._hold_since, self._critical = False, None, None, None, False

    def _hold_in_place(self, now: float) -> None:
        if self._own is None:
            self._own = self._ports.signal.ownership() or NavOwnership()
            self._mode = ModeRequest(self._ports.request_mode, self._own, self._report)
        own, mode = self._own, self._mode
        assert mode is not None
        observation = self._ports.observe_mode()
        own.observe(observation)
        if self._critical:
            if observation.mode is not FlightMode.RTL and own.may_request_mode():
                if mode.pending is FlightMode.RTL:
                    mode.retry(observation, now)
                else:
                    mode.send(FlightMode.RTL, observation, now)
        elif self._hold_since is None:
            if own.may_request_mode():
                self._hold_since = now
                self._report("NavPy NAV loop hung: LOITER")
                if observation.mode is not FlightMode.LOITER:
                    mode.send(FlightMode.LOITER, observation, now)
        elif mode.not_confirmed(observation, now):
            self._enter_critical(observation, now, "LOITER not confirmed")
        elif now - self._hold_since >= NAV_HOLD_RECOVERY_S and own.may_request_mode():
            self._enter_critical(observation, now, f"hung {NAV_HOLD_RECOVERY_S:g} s in the hold")

    def _enter_critical(self, observation: ModeObservation, now: float, why: str) -> None:
        assert self._own is not None and self._mode is not None
        self._critical = True
        self._mode.clear()
        owned = self._own.may_request_mode()
        self._report(f"NavPy critical ({why}): {'RTL' if owned else 'commands stay fenced'}")
        if owned:
            self._mode.send(FlightMode.RTL, observation, now)

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


__all__ = ["NAV_WATCHDOG_TICK_S", "NavWatchdog", "NavWatchdogPorts"]
