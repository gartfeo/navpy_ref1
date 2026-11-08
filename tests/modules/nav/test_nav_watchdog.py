"""In-process watchdog for a hung NAV loop or command worker (docs/design/navpy_recovery.md, step 6).

A hung loop gives the fence; a hung NAV loop also gives the hold in its place (LOITER once if NavPy owns the mode,
RTL after 30 s or when LOITER is not confirmed); a hung worker leaves the hold to NAV. When the hang ends, NAV takes
the hold over and resumes as after any hold. The watchdog never touches NAV state.
"""
from __future__ import annotations

import threading
from contextlib import ExitStack
from unittest.mock import Mock

from navpy.core.flow.nav_mode_request import MODE_CONFIRM_S
from navpy.core.flow.nav_ownership import Ownership
from navpy.core.flow.nav_recovery import NAV_HOLD_RECOVERY_S, NAV_HOLD_TRIGGER_S, NavRecovery, RecoveryStage
from navpy.core.flow.nav_recovery_clock import TimeSample
from navpy.core.flow.nav_recovery_ports import NavRecoveryPorts, RecoveryHold, WatchdogSignal
from navpy.core.flow.nav_watchdog import NavWatchdog, NavWatchdogPorts
from navpy.core.kernel.bounded_sender import BoundedSender
from navpy.core.kernel.progress_beacon import ProgressBeacon
from navpy.core.ports.flight_mode import FlightMode
from tests.modules.nav.test_nav_recovery import HEARTBEAT_PERIODS, PERIOD_S, FakeAutopilot


class WatchdogRig:
    """NAV recovery and the watchdog on one vehicle, one clock; NAV owns GUIDED at the start."""

    def __init__(self, autopilot: FakeAutopilot | None = None) -> None:
        self.ap = FakeAutopilot() if autopilot is None else autopilot
        self.fence = Mock()
        self.signal = WatchdogSignal()
        self.nav_beacon, self.worker_beacon = ProgressBeacon(), ProgressBeacon()
        self.reports: list[str] = []
        self.hold = RecoveryHold(self.ap.observe, self.signal)   # as nav_composition wires it
        self.recovery = NavRecovery(
            NavRecoveryPorts(
                close_fence=self.fence.close, reopen_fence=self.fence.reopen, observe_mode=self.ap.observe,
                request_mode=self.ap.request, logger=Mock(), watchdog=self.signal,
            ),
            self.hold,
        )
        self.watchdog = NavWatchdog(NavWatchdogPorts(
            nav_progress=self.nav_beacon, command_progress=self.worker_beacon, signal=self.signal,
            close_fence=self.fence.close, observe_mode=self.ap.observe, request_mode=self.ap.request,
            report=self.reports.append, clock=lambda: self.t,
        ))
        self.t, self.periods = 100.0, 0
        self.hold.guided_requested()
        self.run(1.0)
        assert self.recovery.ownership.state is Ownership.OWNED
        self.ap.requests.clear()

    def run(self, seconds: float, *, nav_runs: bool = True, cycle_ok: bool = True) -> None:
        for _ in range(round(seconds / PERIOD_S)):
            self.t += PERIOD_S
            self.periods += 1
            if self.periods % HEARTBEAT_PERIODS == 0:
                self.ap.heartbeat()
            if nav_runs:
                with self.nav_beacon.busy():
                    self.recovery.step(cycle_ok, TimeSample(aircraft_s=self.t, wall_s=self.t))
            self.watchdog.check()

    def hang(self, seconds: float, *beacons: ProgressBeacon) -> None:
        """The given loops stay inside one pass for ``seconds``; a hung NAV loop takes no step."""
        with ExitStack() as stack:
            for beacon in beacons:
                stack.enter_context(beacon.busy())
            self.run(seconds, nav_runs=self.nav_beacon not in beacons)


def test_a_slow_but_moving_loop_is_not_a_hang():
    rig = WatchdogRig()
    rig.run(NAV_HOLD_TRIGGER_S * 4)
    rig.fence.close.assert_not_called()
    assert not rig.signal.hung and rig.ap.requests == [] and rig.reports == []


def test_a_hung_nav_loop_gives_the_fence_and_one_loiter():
    rig = WatchdogRig()
    rig.hang(NAV_HOLD_TRIGGER_S - PERIOD_S, rig.nav_beacon)
    rig.fence.close.assert_not_called()
    rig.hang(NAV_HOLD_TRIGGER_S + 2.0, rig.nav_beacon)
    rig.fence.close.assert_called_once()
    assert rig.signal.hung
    assert rig.ap.requests == [FlightMode.LOITER] and rig.ap.mode is FlightMode.LOITER
    # The watchdog never touches NAV state: NAV is still where it stopped.
    assert rig.recovery.stage is RecoveryStage.NORMAL


def test_nav_takes_the_hold_over_and_resumes_in_auto_with_the_task():
    rig = WatchdogRig()
    rig.hang(NAV_HOLD_TRIGGER_S + 3.0, rig.nav_beacon)
    assert rig.ap.mode is FlightMode.LOITER
    rig.run(PERIOD_S)                      # NAV runs again; its decision already sees the hold
    assert rig.hold.holding
    rig.run(PERIOD_S)                      # NAV's step takes the watchdog's hold over
    assert not rig.signal.hung
    assert rig.recovery.stage is RecoveryStage.HOLD
    assert rig.recovery.ownership.state is Ownership.OWNED   # the watchdog's LOITER is NavPy's
    rig.run(NAV_HOLD_TRIGGER_S + 2.0)      # healthy for the trigger time: AUTO, as after any hold
    assert rig.ap.requests[-1] is FlightMode.AUTO and rig.ap.mode is FlightMode.AUTO
    assert rig.recovery.stage is RecoveryStage.NORMAL and rig.hold.resumed
    rig.fence.reopen.assert_called_once()


def test_a_nav_loop_hung_for_the_recovery_time_gives_rtl():
    rig = WatchdogRig()
    rig.hang(NAV_HOLD_TRIGGER_S + NAV_HOLD_RECOVERY_S + 2.0, rig.nav_beacon)
    assert rig.ap.requests == [FlightMode.LOITER, FlightMode.RTL] and rig.ap.mode is FlightMode.RTL
    rig.run(2 * PERIOD_S)
    assert rig.recovery.stage is RecoveryStage.CRITICAL


def test_loiter_not_confirmed_gives_rtl():
    rig = WatchdogRig(FakeAutopilot(takes=False))
    rig.hang(NAV_HOLD_TRIGGER_S + MODE_CONFIRM_S + 1.0, rig.nav_beacon)
    assert FlightMode.RTL in rig.ap.requests
    assert any("LOITER not confirmed" in report for report in rig.reports)


def test_a_mode_the_pilot_takes_during_the_hang_is_never_touched():
    rig = WatchdogRig()
    rig.hang(NAV_HOLD_TRIGGER_S + 2.0, rig.nav_beacon)
    rig.ap.mode = FlightMode.FBWA          # the pilot takes over
    rig.hang(NAV_HOLD_RECOVERY_S + 5.0, rig.nav_beacon)
    assert rig.ap.requests == [FlightMode.LOITER]
    assert rig.ap.mode is FlightMode.FBWA


def test_a_hung_command_worker_gives_the_fence_and_nav_holds():
    rig = WatchdogRig()
    rig.hang(2 * NAV_HOLD_TRIGGER_S + 1.0, rig.worker_beacon)
    rig.fence.close.assert_called()
    assert rig.signal.hung
    # NAV counts its periods failed while the worker is hung, and holds on its own: one LOITER, from NAV.
    assert rig.recovery.stage is RecoveryStage.HOLD
    assert rig.ap.requests == [FlightMode.LOITER]
    rig.run(NAV_HOLD_TRIGGER_S + 2.0)
    assert rig.ap.mode is FlightMode.AUTO and rig.recovery.stage is RecoveryStage.NORMAL


def test_a_short_worker_hang_that_closed_the_fence_still_reopens_it():
    rig = WatchdogRig()
    rig.hang(NAV_HOLD_TRIGGER_S + 0.25, rig.worker_beacon)
    assert rig.hold.holding                            # the decision sees the watchdog's fence as the hold
    rig.run(2 * PERIOD_S)
    assert rig.recovery.stage is RecoveryStage.HOLD    # the watchdog's closed fence is NAV's hold now
    rig.run(NAV_HOLD_TRIGGER_S + 2.0)
    assert rig.recovery.stage is RecoveryStage.NORMAL
    assert rig.fence.reopen.called


def test_a_hung_send_does_not_stop_the_watchdog():
    release = threading.Event()
    reports: list[str] = []
    sent: list[FlightMode] = []

    def hung_send(mode: FlightMode) -> None:
        sent.append(mode)
        release.wait(2.0)

    sender = BoundedSender(hung_send, reports.append, timeout_s=0.05)
    try:
        assert sender(FlightMode.LOITER) is False
        assert sender(FlightMode.RTL) is False          # skipped: the earlier send still holds the link
        assert sent == [FlightMode.LOITER]
        assert any("left behind" in r for r in reports) and any("skipped" in r for r in reports)
    finally:
        release.set()


def test_the_watchdog_thread_starts_checks_and_stops():
    beacon = ProgressBeacon()
    checks = threading.Event()
    ports = NavWatchdogPorts(
        nav_progress=beacon, command_progress=ProgressBeacon(), signal=WatchdogSignal(),
        close_fence=Mock(), observe_mode=Mock(), request_mode=Mock(), report=Mock(),
        clock=lambda: checks.set() or 0.0,
    )
    watchdog = NavWatchdog(ports)
    watchdog.start()
    try:
        assert checks.wait(1.0)
    finally:
        watchdog.stop()


def test_the_nav_loop_is_busy_through_its_cycle_and_recovery_step_only():
    from navpy.core.flow.navigation_loop import NavigationLoop

    readings = []
    cycle, clock = Mock(), Mock()
    clock.scheduler_wall_period.return_value = 0.0
    loop = NavigationLoop(cycle, clock, Mock(), on_period=lambda ok: readings.append(loop.progress.reading()))
    cycle.run.side_effect = lambda: readings.append(loop.progress.reading()) or loop.stop_event.set()

    loop._run()

    assert [(r.passes, r.busy) for r in readings] == [(1, True), (1, True)]
    assert not loop.progress.reading().busy


def test_each_command_worker_pass_is_watched_and_states_its_fence_generation():
    from navpy.app.nav_recovery_link import command_formation
    from navpy.core.flow.nav_recovery_ports import NavRecoveryLink

    stated = []
    link = NavRecoveryLink(
        close_fence=Mock(), reopen_fence=Mock(), observe_mode=Mock(), latch_fence=Mock(),
        formed_now=lambda: _recording(stated),
    )
    with command_formation(link)():
        assert link.command_progress.reading().busy and stated == ["formed"]
    assert not link.command_progress.reading().busy


def _recording(stated):
    from contextlib import contextmanager

    @contextmanager
    def formed():
        stated.append("formed")
        yield

    return formed()


def test_the_application_starts_the_watchdog_and_stops_it_before_the_loop():
    from navpy.core.flow.nav_application import NavApplication

    order = []
    loop, shutdown, watchdog = Mock(), Mock(), Mock()
    loop.start.side_effect = lambda rate: order.append("loop start")
    watchdog.start.side_effect = lambda: order.append("watchdog start")
    watchdog.stop.side_effect = lambda: order.append("watchdog stop")
    shutdown.run.side_effect = lambda loop_stop: order.append("shutdown")
    application = NavApplication(loop, Mock(), shutdown, Mock(), watchdog)

    application.start(50.0)
    application.stop()

    assert order == ["loop start", "watchdog start", "watchdog stop", "shutdown"]


def test_the_runtime_builds_a_watchdog_on_the_link_with_a_bounded_sender():
    from navpy.app.flow.nav_watchdog_composition import nav_watchdog
    from navpy.core.flow.nav_recovery_ports import NavRecoveryLink

    link = NavRecoveryLink(close_fence=Mock(), reopen_fence=Mock(), observe_mode=Mock(), latch_fence=Mock())
    assert nav_watchdog(Mock(), None, ProgressBeacon(), Mock()) is None
    watchdog = nav_watchdog(Mock(), link, ProgressBeacon(), Mock())
    assert isinstance(watchdog._ports.request_mode, BoundedSender)
    assert watchdog._ports.command_progress is link.command_progress and watchdog._ports.signal is link.watchdog


def test_the_watchdog_never_imports_nav_state():
    """It acts through its ports only: no NAV loop state, decision or recovery stage is reachable from it."""
    import ast
    from pathlib import Path

    import navpy.core.flow.nav_watchdog as module

    tree = ast.parse(Path(module.__file__).read_text(encoding="utf-8"))
    imported = {node.module for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)}
    assert not imported & {
        "navpy.core.flow.nav_state", "navpy.core.flow.navigation_decision", "navpy.core.flow.navigation_loop",
    }
    names = {alias.name for node in ast.walk(tree) if isinstance(node, ast.ImportFrom) for alias in node.names}
    assert "NavRecovery" not in names and "RecoveryHold" not in names
