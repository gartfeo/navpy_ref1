"""In-process watchdog for a stuck NAV loop or command worker (docs/design/navpy_recovery.md, step 6).

A stuck loop gives the fence, closed for as long as the hang lasts, and a failed, held recovery period. While the NAV
loop itself is stuck the watchdog steps NAV recovery in its place: one stage machine, one mode request, one hold
deadline. A hang is wall time beyond the waits a pass declared bounded. The watchdog never touches NAV state.
"""
from __future__ import annotations

import threading
from contextlib import ExitStack
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import MagicMock, Mock

from navpy.core.flow.nav_mode_request import MODE_CONFIRM_S
from navpy.core.flow.nav_ownership import Ownership
from navpy.core.flow.nav_recovery import NAV_HOLD_RECOVERY_S, NAV_HOLD_TRIGGER_S, NavRecovery, RecoveryStage
from navpy.core.flow.nav_recovery_clock import TimeSample
from navpy.core.flow.nav_recovery_ports import NavRecoveryPorts, RecoveryHold, WatchdogSignal
from navpy.core.flow.nav_recovery_step import SharedRecoveryStep
from navpy.core.flow.nav_recovery_clock import AIRCRAFT_TIME_STALE_S, HangClock
from navpy.core.flow.nav_watchdog import NAV_HANG_S, NavWatchdog, NavWatchdogPorts
from navpy.core.kernel.progress_beacon import ProgressBeacon, bounded_wait
from navpy.core.ports.flight_mode import FlightMode
from navpy.plugins.vehicle.ardupilot.mav_command_fence import MavCommandFence
from tests.modules.nav.test_nav_recovery import HEARTBEAT_PERIODS, PERIOD_S, FakeAutopilot


class WatchdogRig:
    """NAV recovery and the watchdog on one vehicle, wired as nav_watchdog_composition wires them. Wall time ``t``;
    aircraft time ``aircraft`` advances ``speedup`` times as fast (1 unless set), or not at all while ``frozen``.
    NAV owns GUIDED at the start unless ``owned`` is False."""

    def __init__(
        self, autopilot: FakeAutopilot | None = None, *, owned: bool = True, request_mode=None,
    ) -> None:
        self.ap = FakeAutopilot() if autopilot is None else autopilot
        self.fence = MavCommandFence()
        self.signal = WatchdogSignal()
        self.nav_beacon, self.worker_beacon = ProgressBeacon(), ProgressBeacon()
        self.reports: list[str] = []
        self.hold = RecoveryHold(self.ap.observe)
        self.recovery = NavRecovery(
            NavRecoveryPorts(
                close_fence=self.fence.close, reopen_fence=self.fence.try_reopen, observe_mode=self.ap.observe,
                request_mode=self.ap.request if request_mode is None else request_mode, logger=Mock(),
                watchdog=self.signal,
            ),
            self.hold,
        )
        self.shared = SharedRecoveryStep(self.recovery)
        self.t, self.periods = 100.0, 0
        self.aircraft, self.speedup, self.frozen = 50.0, 1.0, False
        self.watchdog = NavWatchdog(NavWatchdogPorts(
            nav_progress=self.nav_beacon, command_progress=self.worker_beacon, signal=self.signal,
            hold_fence=self.fence.hold_closed, step_recovery=self.shared.step, sample=self.sample,
            report=self.reports.append,
        ))
        if owned:
            self.hold.guided_requested()
            self.run(1.0)
            assert self.recovery.ownership.state is Ownership.OWNED
            self.ap.requests.clear()

    def sample(self) -> TimeSample:
        return TimeSample(aircraft_s=self.aircraft, wall_s=self.t)

    def run(self, seconds: float, *, nav_runs: bool = True, cycle_ok: bool = True) -> None:
        for _ in range(round(seconds / PERIOD_S)):
            self.t += PERIOD_S
            if not self.frozen:
                self.aircraft += PERIOD_S * self.speedup
            self.periods += 1
            if self.periods % HEARTBEAT_PERIODS == 0:
                self.ap.heartbeat()
            if nav_runs:
                with self.nav_beacon.busy():
                    self.shared.step(cycle_ok, self.sample)
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
    assert rig.fence.is_open and rig.stage is RecoveryStage.NORMAL
    rig.hang(NAV_HANG_S + 2.0, rig.nav_beacon)
    assert not rig.fence.is_open and rig.signal.hung
    assert rig.stage is RecoveryStage.HOLD                     # recovery's own stage, stepped by the watchdog
    assert rig.ap.requests == [FlightMode.LOITER] and rig.ap.mode is FlightMode.LOITER
    assert rig.hold.holding


def test_after_the_hang_the_same_hold_resumes_in_auto_with_the_task():
    rig = WatchdogRig()
    rig.hang(NAV_HANG_S + 3.0, rig.nav_beacon)
    rig.run(NAV_HOLD_TRIGGER_S + 2.0)                         # healthy for the trigger time: AUTO, as after any hold
    assert rig.ap.requests == [FlightMode.LOITER, FlightMode.AUTO] and rig.ap.mode is FlightMode.AUTO
    assert rig.stage is RecoveryStage.NORMAL and rig.hold.resumed and rig.fence.is_open


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
    assert not rig.fence.is_open


def test_finding5_nav_does_not_reopen_the_fence_while_a_loop_is_stuck():
    rig = WatchdogRig()
    rig.run(NAV_HOLD_TRIGGER_S + 0.5, cycle_ok=False)         # hold, LOITER
    rig.ap.takes = False                                      # AUTO will be slow to show
    rig.run(NAV_HOLD_TRIGGER_S + PERIOD_S)                    # healthy: AUTO sent, RESUMING
    assert rig.stage is RecoveryStage.RESUMING
    with rig.worker_beacon.busy():
        rig.run(NAV_HANG_S + PERIOD_S)                        # the worker sticks: hang seen, fence closed
        assert not rig.fence.is_open and rig.signal.hung
        rig.ap.mode = FlightMode.AUTO                         # AUTO shows now, during the hang
        rig.run(1.5)
        assert rig.recovery.ownership.ended is not None       # handed over
        assert not rig.fence.is_open and rig.stage is RecoveryStage.HOLD
    rig.run(NAV_HOLD_TRIGGER_S + 1.0)                         # unstuck and healthy: reopened, approach resumes
    assert rig.fence.is_open and rig.stage is RecoveryStage.NORMAL and rig.hold.resumed


def test_finding6_guided_sent_just_before_the_hang_is_recovery_s_and_held():
    rig = WatchdogRig(FakeAutopilot(FlightMode.AUTO, takes=False), owned=False)
    with rig.nav_beacon.busy():
        rig.hold.guided_requested()                           # the cycle sent GUIDED, then stuck before recovery
        rig.run(NAV_HANG_S + 0.5, nav_runs=False)
        # The hold began before GUIDED showed: not NavPy's mode yet, nothing to hold (round 2, finding 8).
        assert rig.stage is RecoveryStage.HOLD
        assert rig.recovery.ownership.state is Ownership.NOT_OWNED and rig.ap.requests == []
        rig.ap.mode, rig.ap.takes = FlightMode.GUIDED, True   # NavPy's GUIDED shows only now
        rig.run(3.0, nav_runs=False)
        assert rig.recovery.ownership.state is Ownership.OWNED
        assert rig.ap.requests == [FlightMode.LOITER] and rig.ap.mode is FlightMode.LOITER
        rig.run(NAV_HOLD_RECOVERY_S, nav_runs=False)
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
    assert not rig.fence.is_open


def test_finding8_a_declared_bounded_wait_is_not_a_hang_but_beyond_it_is():
    rig = WatchdogRig()
    with rig.nav_beacon.busy():
        with bounded_wait(6.0):                               # a parameter exchange: 3 attempts of 2 s
            rig.run(NAV_HANG_S + 5.0, nav_runs=False)
            assert rig.fence.is_open and not rig.signal.hung     # checked while still busy
        rig.run(1.0 + PERIOD_S, nav_runs=False)            # timed from the first check that saw it busy
        assert not rig.fence.is_open and rig.signal.hung


def test_finding8_a_slow_but_moving_loop_is_not_a_hang():
    rig = WatchdogRig()
    for _ in range(10):
        with rig.nav_beacon.busy():
            rig.run(NAV_HANG_S - PERIOD_S, nav_runs=False)    # slow passes, each checked while busy
            assert rig.fence.is_open and not rig.signal.hung
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

    vehicle, guard = Mock(), Mock()
    link = nav_recovery_link(MavCommandFence(), vehicle)
    link.request_mode(FlightMode.LOITER, guard)
    vehicle.set_mode.assert_called_once_with(FlightMode.LOITER, lock_timeout_s=MODE_SEND_LOCK_TIMEOUT_S, guard=guard)


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
    threads = nav_watchdog(link, ProgressBeacon(), shared, Mock(), Mock())
    watchdog = threads._watchdog
    assert watchdog._ports.step_recovery == shared.step     # one recovery, stepped by the loop or the watchdog
    assert threads._sender is shared.sender and shared.recovery._ports.mode_sender is shared.sender
    recovery = shared.recovery
    assert shared.sender._send is link.request_mode and recovery._ports.watchdog is link.watchdog
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
        hold_fence=Mock(), step_recovery=None, sample=lambda: checks.set() or TimeSample(0.0, 0.0), report=Mock(),
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
        assert shared.step(False, Mock, lock_timeout_s=0.01) is False
    finally:
        shared._lock.release()
    recovery.step.assert_not_called()
    assert shared.step(False, Mock, lock_timeout_s=0.01) is True
    recovery.step.assert_called_once()


# --- Codex round 2 on the step 6 rework ---------------------------------------------------------------------------


def _owned_recovery(ap, **ports):
    hold = RecoveryHold(ap.observe)
    fence = MavCommandFence()
    recovery = NavRecovery(NavRecoveryPorts(
        close_fence=fence.close, reopen_fence=fence.try_reopen, observe_mode=ap.observe,
        request_mode=ports.pop("request_mode", ap.request), logger=Mock(), **ports,
    ), hold)
    t = [100.0]

    def step(cycle_ok: bool, seconds: float = PERIOD_S, shared=None) -> None:
        for _ in range(round(seconds / PERIOD_S)):
            t[0] += PERIOD_S
            if round(t[0] / PERIOD_S) % HEARTBEAT_PERIODS == 0:
                ap.heartbeat()
            sample = TimeSample(aircraft_s=t[0], wall_s=t[0])
            if shared is None:
                recovery.step(cycle_ok, sample)
            else:
                shared.step(cycle_ok, lambda: sample)

    hold.guided_requested()
    step(True, 1.0)
    assert recovery.ownership.state is Ownership.OWNED
    ap.requests.clear()
    return recovery, hold, fence, step


def test_round2_finding1_a_write_that_never_returns_never_blocks_recovery():
    """A blocked write is the mode sender's; recovery's step, on whichever thread, returns at once."""
    from navpy.core.flow.nav_mode_sender import ModeSender

    ap = FakeAutopilot()
    write_entered, release = threading.Event(), threading.Event()

    def blocked_write(mode, guard):
        if guard():
            write_entered.set()
            release.wait(5.0)                                 # a serial write that does not return
        return True

    sender = ModeSender(blocked_write, ap.observe, Mock())
    recovery, hold, fence, step = _owned_recovery(ap, mode_sender=sender)
    shared = SharedRecoveryStep(recovery, sender)
    sender.start()
    try:
        stepped = threading.Event()
        nav = threading.Thread(target=lambda: (step(False, NAV_HOLD_TRIGGER_S + 1.0, shared), stepped.set()))
        nav.start()
        assert write_entered.wait(2.0)                        # the LOITER write is stuck in the sender
        assert stepped.wait(2.0)                              # and recovery's steps went on
        assert recovery.stage is RecoveryStage.HOLD
        assert shared.step(False, lambda: TimeSample(200.0, 200.0), lock_timeout_s=0.1)   # the watchdog's step too
    finally:
        release.set()
        sender.stop()


def test_round2_finding2_a_pilot_mode_shown_during_the_lock_wait_cancels_the_write():
    from navpy.plugins.vehicle.ardupilot.mav_transport import MavTransport
    from navpy.plugins.vehicle.ardupilot.mode_control import ModeControl
    from navpy.core.flow.nav_mode_sender import ModeSender

    ap = FakeAutopilot()                                      # GUIDED, owned
    connection = MagicMock()
    connection.mode_mapping.return_value = {"LOITER": 12}
    lock = threading.RLock()
    control = ModeControl(SimpleNamespace(target_system=1), MavTransport(connection, lock), Mock(),
                          SimpleNamespace(value=Mock()))
    sender = ModeSender(lambda mode, guard: control.set(mode, lock_timeout_s=2.0, guard=guard), ap.observe, Mock(),
                        threaded=False)
    held, release = threading.Event(), threading.Event()

    def stuck_sender() -> None:
        with lock:
            held.set()
            release.wait(2.0)

    blocker = threading.Thread(target=stuck_sender, daemon=True)
    blocker.start()
    assert held.wait(1.0)
    writer = threading.Thread(target=lambda: sender.send_now(FlightMode.LOITER, FlightMode.GUIDED))
    writer.start()                                            # recovery decided LOITER from GUIDED; waits for the lock
    ap.mode = FlightMode.FBWA                                 # the pilot's mode shows meanwhile
    ap.heartbeat()
    release.set()
    writer.join(3.0)
    connection.mav.set_mode_send.assert_not_called()
    assert [e.heartbeat_count for e in sender.take_events()] == [None]


def test_round2_finding3_an_unsent_request_never_makes_the_pilots_mode_look_owned():
    ap = FakeAutopilot()
    recovery, hold, fence, step = _owned_recovery(ap, request_mode=lambda mode: False)   # the lock never came
    step(False, NAV_HOLD_TRIGGER_S + 0.25)                    # hold: LOITER decided, not written
    assert recovery.stage is RecoveryStage.HOLD
    ap.mode = FlightMode.LOITER                               # the pilot selects LOITER
    step(False, 1.0)
    assert recovery.ownership.state is Ownership.NOT_OWNED    # the pilot's, not NavPy's
    sent = []
    recovery._mode._sender._send = lambda mode, guard: guard() and sent.append(mode) is None
    step(True, NAV_HOLD_TRIGGER_S + NAV_HOLD_RECOVERY_S)
    assert sent == [] and ap.mode is FlightMode.LOITER       # no AUTO, no RTL over the pilot's LOITER


def test_round2_finding4_a_resume_racing_the_watchdog_cannot_reopen_the_fence():
    """Recovery has decided to reopen after a healthy hold; at that moment the watchdog thread sees the worker
    stuck (its check runs on a real second thread, between the decision and the reopen)."""
    rig = WatchdogRig(owned=False)
    rig.run(NAV_HOLD_TRIGGER_S + 0.5, cycle_ok=False)         # unowned hold: fenced
    assert rig.stage is RecoveryStage.HOLD and not rig.fence.is_open
    real_reopen = rig.fence.try_reopen
    races = []

    def reopen_racing_the_watchdog():
        checker = threading.Thread(target=rig.watchdog.check)
        checker.start()
        checker.join(2.0)
        races.append(rig.signal.hung)
        return real_reopen()

    rig.recovery._ports = replace(rig.recovery._ports, reopen_fence=reopen_racing_the_watchdog)
    with rig.worker_beacon.busy():                            # stuck from the start of the healthy phase
        rig.run(NAV_HOLD_TRIGGER_S + PERIOD_S)                # recovery resumes just as the hang is seen
        assert races and races[0] is True                     # the watchdog saw it in between
        assert not rig.fence.is_open and rig.stage is RecoveryStage.HOLD


def test_round2_finding5_the_hold_shows_before_its_loiter_is_requested():
    ap = FakeAutopilot()
    seen = []
    recovery_box = []

    def request(mode):
        seen.append((mode, recovery_box[0].hold.holding))
        ap.request(mode)

    recovery, hold, fence, step = _owned_recovery(ap, request_mode=request)
    recovery_box.append(recovery)
    step(False, NAV_HOLD_TRIGGER_S + 0.25)
    assert seen == [(FlightMode.LOITER, True)]


def test_round2_finding6_a_step_reads_its_time_after_the_other_thread_s_step():
    """A step waiting for the lock samples time only once it has it: never older than the step before it."""
    recovery = Mock()
    shared = SharedRecoveryStep(recovery)
    clock = [100.0]
    inside, release = threading.Event(), threading.Event()

    def first_step(cycle_ok, sample):
        inside.set()
        release.wait(2.0)

    recovery.step.side_effect = first_step
    watchdog = threading.Thread(target=lambda: shared.step(False, lambda: TimeSample(clock[0], clock[0])))
    watchdog.start()
    assert inside.wait(1.0)
    recovery.step.side_effect = None
    nav = threading.Thread(target=lambda: shared.step(True, lambda: TimeSample(clock[0], clock[0])))
    nav.start()                                               # NAV asks while the watchdog's step runs
    clock[0] = 110.0                                          # time moves on before NAV gets the lock
    release.set()
    watchdog.join(2.0)
    nav.join(2.0)
    samples = [c.args[1].aircraft_s for c in recovery.step.call_args_list]
    assert samples == [100.0, 110.0]


# --- Owner ruling on round-2 finding 7: the hang is measured in aircraft time --------------------------------------


def test_finding7_at_sim_speedup_10_the_hang_trips_at_the_aircraft_time_threshold():
    """Aircraft time 10x wall time: stuck for NAV_HANG_S of aircraft time is a hang, after a tenth of it in wall
    time, not ten times later."""
    rig = WatchdogRig()
    rig.speedup = 10.0
    with rig.nav_beacon.busy():
        aircraft_at_start = rig.aircraft
        while not rig.signal.hung:
            rig.run(PERIOD_S, nav_runs=False)
            assert rig.aircraft - aircraft_at_start <= NAV_HANG_S + 2 * PERIOD_S * rig.speedup
        assert rig.aircraft - aircraft_at_start >= NAV_HANG_S
    assert rig.ap.requests == [FlightMode.LOITER]


def test_finding7_with_aircraft_time_frozen_the_wall_time_backstop_trips():
    """A stalled link or a frozen SITL: no new aircraft time, so NAV_HANG_S of wall time trips the hang."""
    rig = WatchdogRig()
    rig.frozen = True
    with rig.nav_beacon.busy():
        rig.run(NAV_HANG_S - 2 * PERIOD_S, nav_runs=False)
        assert not rig.signal.hung
        rig.run(3 * PERIOD_S, nav_runs=False)
        assert rig.signal.hung
    assert not rig.fence.is_open


def test_finding7_wall_time_does_not_trip_a_hang_while_aircraft_time_runs_slower():
    """Aircraft time at half wall speed and fresh: NAV_HANG_S of wall time is not yet a hang, NAV_HANG_S of
    aircraft time is. Wall time counts only while aircraft time is frozen."""
    rig = WatchdogRig()
    rig.speedup = 0.5
    with rig.nav_beacon.busy():
        rig.run(NAV_HANG_S + 0.5, nav_runs=False)
        assert not rig.signal.hung
        rig.run(NAV_HANG_S, nav_runs=False)
        assert rig.signal.hung


def test_hang_clock_counts_a_frozen_stretch_once():
    clock = HangClock()
    assert clock.advance(TimeSample(10.0, 0.0)) == 0.0
    assert clock.advance(TimeSample(15.0, 0.5)) == 5.0                        # fresh: aircraft time (10x)
    assert clock.advance(TimeSample(15.0, 0.5 + AIRCRAFT_TIME_STALE_S / 2)) == 5.0   # not yet frozen
    assert clock.advance(TimeSample(15.0, 2.5)) == 7.0                        # frozen: wall since the last sample
    assert clock.advance(TimeSample(15.0, 3.0)) == 7.5
    assert clock.advance(TimeSample(45.0, 3.1)) == 7.5                        # moves again: re-anchors, no jump
    assert clock.advance(TimeSample(46.0, 3.2)) == 8.5
    assert clock.advance(TimeSample(1.0, 3.3)) == 8.5                         # autopilot reboot: never back
    assert clock.advance(TimeSample(2.0, 3.4)) == 9.5


def test_hang_clock_without_aircraft_time_runs_on_wall_time():
    clock = HangClock()
    clock.advance(TimeSample(None, 0.0))
    assert clock.advance(TimeSample(None, AIRCRAFT_TIME_STALE_S / 2)) == 0.0
    assert clock.advance(TimeSample(None, 2.0)) == 2.0
