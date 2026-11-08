"""In-process watchdog for a stuck NAV loop or command worker (docs/design/navpy_recovery.md, step 6).

A stuck loop gives the fence, closed for as long as the hang lasts, and a failed, held recovery period. While the NAV
loop itself is stuck the watchdog steps NAV recovery in its place: one stage machine, one mode request, one hold
deadline. A hang is wall time beyond the waits a pass declared bounded. The watchdog never touches NAV state.
"""
from __future__ import annotations

import threading
from contextlib import ExitStack
from types import SimpleNamespace
from unittest.mock import MagicMock, Mock

from navpy.core.flow.nav_mode_request import MODE_CONFIRM_S
from navpy.core.flow.nav_ownership import Ownership
from navpy.core.flow.nav_recovery import NAV_HOLD_RECOVERY_S, NAV_HOLD_TRIGGER_S, NavRecovery, RecoveryStage
from navpy.core.flow.nav_recovery_clock import TimeSample
from navpy.core.flow.nav_recovery_ports import NavRecoveryPorts, RecoveryHold, WatchdogSignal
from navpy.core.flow.nav_recovery_step import SharedRecoveryStep
from navpy.core.flow.nav_watchdog import NAV_HANG_S, NavWatchdog, NavWatchdogPorts
from navpy.core.kernel.progress_beacon import ProgressBeacon, bounded_wait
from navpy.core.ports.flight_mode import FlightMode
from tests.modules.nav.test_nav_recovery import HEARTBEAT_PERIODS, PERIOD_S, FakeAutopilot


class Fence:
    def __init__(self) -> None:
        self.open = True
        self.closes = 0
        self.reopens = 0

    def close(self) -> None:
        self.open = False
        self.closes += 1

    def reopen(self) -> None:
        self.open = True
        self.reopens += 1


class WatchdogRig:
    """NAV recovery and the watchdog on one vehicle, wired as nav_watchdog_composition wires them; one clock for
    wall and aircraft time (speed-up 1). NAV owns GUIDED at the start unless ``owned`` is False."""

    def __init__(self, autopilot: FakeAutopilot | None = None, *, owned: bool = True) -> None:
        self.ap = FakeAutopilot() if autopilot is None else autopilot
        self.fence = Fence()
        self.signal = WatchdogSignal()
        self.nav_beacon, self.worker_beacon = ProgressBeacon(), ProgressBeacon()
        self.reports: list[str] = []
        self.hold = RecoveryHold(self.ap.observe)
        self.recovery = NavRecovery(
            NavRecoveryPorts(
                close_fence=self.fence.close, reopen_fence=self.fence.reopen, observe_mode=self.ap.observe,
                request_mode=self.ap.request, logger=Mock(), watchdog=self.signal,
            ),
            self.hold,
        )
        self.shared = SharedRecoveryStep(self.recovery)
        self.t, self.periods = 100.0, 0
        self.watchdog = NavWatchdog(NavWatchdogPorts(
            nav_progress=self.nav_beacon, command_progress=self.worker_beacon, signal=self.signal,
            close_fence=self.fence.close, step_recovery=self.shared.step, sample=self.sample,
            report=self.reports.append, wall_s=lambda: self.t,
        ))
        if owned:
            self.hold.guided_requested()
            self.run(1.0)
            assert self.recovery.ownership.state is Ownership.OWNED
            self.ap.requests.clear()

    def sample(self) -> TimeSample:
        return TimeSample(aircraft_s=self.t, wall_s=self.t)

    def run(self, seconds: float, *, nav_runs: bool = True, cycle_ok: bool = True) -> None:
        for _ in range(round(seconds / PERIOD_S)):
            self.t += PERIOD_S
            self.periods += 1
            if self.periods % HEARTBEAT_PERIODS == 0:
                self.ap.heartbeat()
            if nav_runs:
                with self.nav_beacon.busy():
                    self.shared.step(cycle_ok, self.sample())
            self.watchdog.check()

    def hang(self, seconds: float, *beacons: ProgressBeacon) -> None:
        """The given loops stay inside one pass for ``seconds``; a stuck NAV loop takes no step of its own."""
        with ExitStack() as stack:
            for beacon in beacons:
                stack.enter_context(beacon.busy())
            self.run(seconds, nav_runs=self.nav_beacon not in beacons)

    @property
    def stage(self) -> RecoveryStage:
        return self.recovery.stage


def test_a_stuck_nav_loop_gives_the_fence_and_one_loiter_from_recovery():
    rig = WatchdogRig()
    rig.hang(NAV_HANG_S - PERIOD_S, rig.nav_beacon)
    assert rig.fence.open and rig.stage is RecoveryStage.NORMAL
    rig.hang(NAV_HANG_S + 2.0, rig.nav_beacon)
    assert not rig.fence.open and rig.signal.hung
    assert rig.stage is RecoveryStage.HOLD                     # recovery's own stage, stepped by the watchdog
    assert rig.ap.requests == [FlightMode.LOITER] and rig.ap.mode is FlightMode.LOITER
    assert rig.hold.holding


def test_after_the_hang_the_same_hold_resumes_in_auto_with_the_task():
    rig = WatchdogRig()
    rig.hang(NAV_HANG_S + 3.0, rig.nav_beacon)
    rig.run(NAV_HOLD_TRIGGER_S + 2.0)                         # healthy for the trigger time: AUTO, as after any hold
    assert rig.ap.requests == [FlightMode.LOITER, FlightMode.AUTO] and rig.ap.mode is FlightMode.AUTO
    assert rig.stage is RecoveryStage.NORMAL and rig.hold.resumed and rig.fence.open


def test_finding1_a_hang_after_a_critical_rtl_never_sends_loiter():
    rig = WatchdogRig()
    rig.run(NAV_HOLD_TRIGGER_S + NAV_HOLD_RECOVERY_S + 2.0, cycle_ok=False)
    assert rig.stage is RecoveryStage.CRITICAL and rig.ap.mode is FlightMode.RTL
    requests = list(rig.ap.requests)
    rig.hang(NAV_HANG_S + 5.0, rig.nav_beacon)
    assert rig.ap.requests == requests and rig.ap.mode is FlightMode.RTL
    assert rig.stage is RecoveryStage.CRITICAL


def test_finding2_nav_recovering_while_the_worker_stays_stuck_keeps_the_hold_to_rtl():
    rig = WatchdogRig()
    with rig.worker_beacon.busy():
        rig.hang(NAV_HANG_S + 3.0, rig.nav_beacon)             # both stuck: LOITER
        assert rig.ap.mode is FlightMode.LOITER
        rig.run(NAV_HOLD_RECOVERY_S)                          # NAV runs again, the worker stays stuck
    assert rig.recovery.ownership.state is Ownership.OWNED    # the LOITER is recovery's own request
    assert rig.ap.requests == [FlightMode.LOITER, FlightMode.RTL] and rig.ap.mode is FlightMode.RTL
    assert not rig.fence.open


def test_finding5_nav_does_not_reopen_the_fence_while_a_loop_is_stuck():
    rig = WatchdogRig()
    rig.run(NAV_HOLD_TRIGGER_S + 0.5, cycle_ok=False)         # hold, LOITER
    rig.ap.takes = False                                      # AUTO will be slow to show
    rig.run(NAV_HOLD_TRIGGER_S + PERIOD_S)                    # healthy: AUTO sent, RESUMING
    assert rig.stage is RecoveryStage.RESUMING
    with rig.worker_beacon.busy():
        rig.run(NAV_HANG_S + PERIOD_S)                        # the worker sticks: hang seen, fence closed
        assert not rig.fence.open and rig.signal.hung
        rig.ap.mode = FlightMode.AUTO                         # AUTO shows now, during the hang
        rig.run(1.5)
        assert rig.recovery.ownership.ended is not None       # handed over
        assert not rig.fence.open and rig.stage is RecoveryStage.HOLD
    rig.run(NAV_HOLD_TRIGGER_S + 1.0)                         # unstuck and healthy: reopened, approach resumes
    assert rig.fence.open and rig.stage is RecoveryStage.NORMAL and rig.hold.resumed


def test_finding6_guided_sent_just_before_the_hang_is_recovery_s_and_held():
    rig = WatchdogRig(FakeAutopilot(FlightMode.AUTO), owned=False)
    rig.hold.guided_requested()                               # the cycle sent GUIDED, then stuck before recovery
    rig.ap.request(FlightMode.GUIDED)
    rig.hang(NAV_HANG_S + 3.0, rig.nav_beacon)
    assert rig.recovery.ownership.state is Ownership.OWNED
    assert rig.ap.requests[-1] is FlightMode.LOITER and rig.ap.mode is FlightMode.LOITER
    rig.hang(NAV_HOLD_RECOVERY_S, rig.nav_beacon)
    assert rig.ap.mode is FlightMode.RTL


def test_finding7_one_deadline_a_hang_and_later_failures_rtl_at_30_s_from_the_hold():
    rig = WatchdogRig()
    rig.hang(NAV_HANG_S + NAV_HOLD_RECOVERY_S - 5.0, rig.nav_beacon)
    assert rig.ap.mode is FlightMode.LOITER and rig.stage is RecoveryStage.HOLD
    rig.run(6.0, cycle_ok=False)                              # NAV runs again, its cycles keep failing
    assert rig.ap.mode is FlightMode.RTL and rig.stage is RecoveryStage.CRITICAL


def test_finding7_an_unconfirmed_loiter_keeps_its_deadline_after_the_hang():
    rig = WatchdogRig(FakeAutopilot(takes=False))
    rig.hang(NAV_HANG_S + 1.0, rig.nav_beacon)                # LOITER sent, not taken
    rig.run(MODE_CONFIRM_S + 1.0)                             # NAV back and healthy: the request still must show
    assert FlightMode.RTL in rig.ap.requests and rig.stage is RecoveryStage.CRITICAL
    assert not rig.fence.open


def test_finding8_a_declared_bounded_wait_is_not_a_hang_but_beyond_it_is():
    rig = WatchdogRig()
    with rig.nav_beacon.busy():
        with bounded_wait(6.0):                               # a parameter exchange: 3 attempts of 2 s
            rig.run(NAV_HANG_S + 5.0, nav_runs=False)
            assert rig.fence.open and not rig.signal.hung     # checked while still busy
        rig.run(1.0 + PERIOD_S, nav_runs=False)            # timed from the first check that saw it busy
        assert not rig.fence.open and rig.signal.hung


def test_finding8_a_slow_but_moving_loop_is_not_a_hang():
    rig = WatchdogRig()
    for _ in range(10):
        with rig.nav_beacon.busy():
            rig.run(NAV_HANG_S - PERIOD_S, nav_runs=False)    # slow passes, each checked while busy
            assert rig.fence.open and not rig.signal.hung
    assert rig.reports == []


def test_finding8_the_vehicles_reply_waits_declare_their_bound():
    from navpy.core.kernel import runtime_time as rt
    from navpy.plugins.vehicle.ardupilot.mission_inbox import MissionInbox

    beacon = ProgressBeacon()
    with beacon.busy():
        MissionInbox().wait_after(0, lambda message: False, deadline=rt.monotonic() + 0.05)
        assert beacon.reading().allowance_s > 0


def test_finding3_a_mode_send_never_waits_long_nor_writes_late():
    from navpy.plugins.vehicle.ardupilot.mav_transport import MavTransport
    from navpy.plugins.vehicle.ardupilot.mode_control import ModeControl

    connection = MagicMock()
    connection.mode_mapping.return_value = {"LOITER": 12}
    lock = threading.RLock()
    transport = MavTransport(connection, lock)
    control = ModeControl(SimpleNamespace(target_system=1), transport, Mock(), SimpleNamespace(value=Mock()))
    held, release = threading.Event(), threading.Event()

    def stuck_sender() -> None:
        with lock:
            held.set()
            release.wait(2.0)

    thread = threading.Thread(target=stuck_sender, daemon=True)
    thread.start()
    assert held.wait(1.0)
    assert control.set(FlightMode.LOITER, lock_timeout_s=0.05) is False
    release.set()
    thread.join(1.0)
    connection.mav.set_mode_send.assert_not_called()          # nothing left behind to write once the lock frees
    assert control.set(FlightMode.LOITER, lock_timeout_s=0.05) is True
    connection.mav.set_mode_send.assert_called_once()


def test_finding3_recovery_s_mode_requests_use_the_bounded_send():
    from navpy.app.nav_recovery_link import nav_recovery_link
    from navpy.core.flow.nav_mode_request import MODE_SEND_LOCK_TIMEOUT_S

    vehicle = Mock()
    link = nav_recovery_link(Mock(), vehicle)
    link.request_mode(FlightMode.LOITER)
    vehicle.set_mode.assert_called_once_with(FlightMode.LOITER, lock_timeout_s=MODE_SEND_LOCK_TIMEOUT_S)


def test_finding4_the_whole_worker_pass_is_watched_including_source_dispatch():
    from navpy.core.approach.navigation_command_worker import (
        NavigationCommandWorker,
        NavigationCommandWorkerPorts,
    )

    beacon = ProgressBeacon()
    stop, in_dispatch = threading.Event(), threading.Event()
    seen = []

    def dispatch() -> bool:
        seen.append(beacon.reading().busy)
        in_dispatch.set()
        stop.set()
        return False

    runtime = MagicMock()
    runtime.has_command_pending_or_in_flight.return_value = False
    session = MagicMock()
    session.return_value.__enter__.return_value = runtime
    worker = NavigationCommandWorker(NavigationCommandWorkerPorts(
        stop_event=stop, command_event=threading.Event(), wall_period_s=lambda period: 0.001,
        runtime_session=session, logger=Mock(), source_dispatch=dispatch,
    ), beacon.busy)
    worker.run()
    assert in_dispatch.is_set() and seen == [True]            # source dispatch runs inside the watched pass
    assert not beacon.reading().busy                          # and the pass ends with the run


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


def test_the_runtime_wires_one_recovery_to_the_loop_and_the_watchdog():
    from navpy.app.flow.nav_watchdog_composition import nav_recovery, nav_watchdog
    from navpy.core.flow.nav_recovery_ports import NavRecoveryLink

    link = NavRecoveryLink(
        close_fence=Mock(), reopen_fence=Mock(), observe_mode=Mock(), latch_fence=Mock(), request_mode=Mock(),
    )
    shared = nav_recovery(Mock(), RecoveryHold(), Mock(), link)
    watchdog = nav_watchdog(link, ProgressBeacon(), shared, Mock(), Mock())
    assert watchdog._ports.step_recovery == shared.step     # one recovery, stepped by the loop or the watchdog
    recovery = shared.recovery
    assert recovery._ports.request_mode is link.request_mode and recovery._ports.watchdog is link.watchdog
    assert nav_recovery(Mock(), RecoveryHold(), Mock(), None) is None
    assert nav_watchdog(None, ProgressBeacon(), None, Mock(), Mock()) is None


def test_the_watchdog_never_imports_nav_state():
    """It acts through its ports only: no NAV loop state or decision is reachable from it."""
    import ast
    from pathlib import Path

    import navpy.core.flow.nav_watchdog as module

    tree = ast.parse(Path(module.__file__).read_text(encoding="utf-8"))
    imported = {node.module for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)}
    assert not imported & {
        "navpy.core.flow.nav_state", "navpy.core.flow.navigation_decision", "navpy.core.flow.navigation_loop",
    }


def test_the_watchdog_thread_starts_checks_and_stops():
    checks = threading.Event()
    watchdog = NavWatchdog(NavWatchdogPorts(
        nav_progress=ProgressBeacon(), command_progress=ProgressBeacon(), signal=WatchdogSignal(),
        close_fence=Mock(), step_recovery=None, sample=Mock(), report=Mock(),
        wall_s=lambda: checks.set() or 0.0,
    ))
    watchdog.start()
    try:
        assert checks.wait(1.0)
    finally:
        watchdog.stop()


def test_the_shared_step_gives_up_when_its_lock_is_held_too_long():
    """The watchdog never waits on a stuck NAV step for long: one step at a time, bounded wait for the other."""
    recovery = Mock()
    shared = SharedRecoveryStep(recovery)
    shared._lock.acquire()
    try:
        assert shared.step(False, Mock(), lock_timeout_s=0.01) is False
    finally:
        shared._lock.release()
    recovery.step.assert_not_called()
    assert shared.step(False, Mock(), lock_timeout_s=0.01) is True
    recovery.step.assert_called_once()
