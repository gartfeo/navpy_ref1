"""The run's quiesce (hitl_final_approach.md section 9, step 16): one idempotent step for terminal, deadline, crash
and abort, at the board boundary (the real router with navpy as its reader, the real barrier, the router's write
log), with late queues and reconnects; navpy's own quiesce when it is stopped without an outcome; the runner's exit
paths; and the rule that a deadline never yields DOCKED."""
from __future__ import annotations

import json
import signal
import socket
import threading
from pathlib import Path

import pytest
from pymavlink.dialects.v20 import ardupilotmega as mavlink2

from navpy.app.router_barrier import BarrierResult, RouterBarrier
from navpy.app.runtime_resources import RuntimeResourceStack
from navpy.app.terminal_handoff import (TRIGGER_SHUTDOWN, TRIGGER_STOP, LinkFence, TerminalHandoff, own_quiesce,
                                        print_terminal_record)
from navpy.core.approach.vision_nav.episode_latch import FinalApproachTerminal as Terminal
from tests.modules.navigation.test_terminal_handoff import IDENTITY, RecordingGimbal, ScriptedBarrier
from tests.tools.tuning import test_approach_runner as runner_rig
from tests.tools.tuning.test_mav_router import (NAVPY, RUNNER, GatedBoard, UdpClient, heartbeat, receivers_current,
                                                wait_until)
from tools.tuning import approach, approach_quiesce as stop
from tools.tuning.approach_procs import Owned, ProcSpec, RunnerError
from tools.tuning.mav_router_core import MavRouter
from tools.tuning.mav_router_endpoint import Client
from tools.tuning.mp_mirror import LOOPBACK
from tools.tuning.navpy_handoff import NavpyHandoff
from tools.tuning.vehicle import MODE

M = mavlink2
WAIT_S = 2.0
QUEUED = 4                       # navpy packets queued behind the write in progress when the quiesce starts
ATTITUDE_TARGET = M.MAVLink_set_attitude_target_message(0, 1, 1, 0, [1, 0, 0, 0], 0, 0, 0, 0.5)
RTL = M.MAVLink_command_long_message(1, 1, M.MAV_CMD_DO_SET_MODE, 0, 1, MODE["RTL"], 0, 0, 0, 0, 0)


@pytest.fixture
def boundary(tmp_path):
    """The router on a held board, navpy its reader, the runner a full client, the write log in a file."""
    board, navpy, runner = GatedBoard(), UdpClient(NAVPY), UdpClient(RUNNER)
    log_path = tmp_path / "router_writes.csv"
    log = log_path.open("w", encoding="utf-8", buffering=1)
    router = MavRouter(board, [Client("navpy", navpy.port, False, reader=True), Client("runner", runner.port, False)],
                       announce=lambda line: None, control_port=0, write_log=log).start()
    ports = {name: c.sock.getsockname() for name, c in router.clients.items()}
    navpy.peer, runner.peer = ports["navpy"], ports["runner"]
    wait_until(lambda: receivers_current(router))
    rig = {"board": board, "navpy": navpy, "runner": runner, "router": router, "log": log_path}
    yield rig
    board.gate.set()
    router.close()
    log.close()
    rig["navpy"].sock.close()
    runner.sock.close()


def control(router: MavRouter) -> tuple[str, int]:
    return router.control.sock.getsockname()


def late_queue(rig: dict) -> None:
    """A navpy write in progress at the board, QUEUED more behind it, and one forbidden write (blocked)."""
    board, navpy, router = rig["board"], rig["navpy"], rig["router"]
    board.gate.clear()
    navpy.send(ATTITUDE_TARGET)
    for _ in range(1 + QUEUED):
        navpy.send(heartbeat())
    assert board.writing.wait(WAIT_S)
    wait_until(lambda: router.clients["navpy"].queued == QUEUED)


def released_after(board: GatedBoard, seconds: float = 0.1) -> threading.Timer:
    """The write in progress returns a little later, while the barrier waits for it."""
    timer = threading.Timer(seconds, board.gate.set)
    timer.start()
    return timer


def navpy_reconnects(rig: dict) -> None:
    """A new navpy process binds the same port and sends again: the closed endpoint refuses it."""
    old = rig["navpy"]
    port, peer = old.port, old.peer
    old.sock.close()
    fresh = UdpClient(NAVPY)
    fresh.sock.close()
    fresh.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    fresh.sock.bind((LOOPBACK, port))
    fresh.port, fresh.peer = port, peer
    rig["navpy"] = fresh
    fresh.send(heartbeat())
    fresh.send(ATTITUDE_TARGET)


def board_reconnects(rig: dict) -> None:
    """The board link drops and comes back (a reboot); navpy's endpoint stays closed across it."""
    board, router = rig["board"], rig["router"]
    opened = board.open_count
    board.reboot()
    wait_until(lambda: board.open_count == opened + 1 and router._link_up.is_set())


def refused(rig: dict) -> int:
    """navpy datagrams the router turned away: on the closed endpoint, or read across the board's reboot."""
    stats = rig["router"].report()["stats"]
    return stats.get("refused_closed_navpy", 0) + stats.get("dropped_stale_navpy", 0)


def runner_flies_rtl(rig: dict) -> None:
    writes = len(rig["board"].written)
    rig["runner"].send(RTL)
    wait_until(lambda: len(rig["board"].written) > writes)


def handoff_for(rig: dict, case: str, events: list) -> NavpyHandoff:
    """navpy's stop for each fault case, as the runner sees it."""
    navpy, router = rig["navpy"], rig["router"]

    def navpy_stop() -> dict:
        events.append("navpy stop")
        if case == "deadline":                     # navpy still writes while it stops: these queue late as well
            navpy.send(heartbeat())
            navpy.send(ATTITUDE_TARGET)
            return {"stop": "requested", "returncode": 0}
        if case == "terminal":
            return {"stop": "requested", "returncode": 0}
        return {"stop": "already exited", "returncode": -9}         # crash, abort: navpy is gone, its packets not

    return NavpyHandoff(navpy_stop, RouterBarrier(control(router), "navpy"),
                        lambda: pytest.fail("the router answered"), lambda: pytest.fail("the router answered"))


REASONS = {"terminal": stop.TERMINAL, "deadline": stop.DEADLINE, "crash": stop.CRASH, "abort": stop.ABORT}


@pytest.mark.parametrize("case", list(REASONS))
def test_no_navpy_write_reaches_the_board_after_the_drain_ack_in_any_fault_case(boundary, case):
    rig, events, records = boundary, [], []
    late_queue(rig)
    if case == "terminal":                         # navpy's own handoff closes first; the runner's close repeats it
        fence = LinkFence()
        navpy_handoff = TerminalHandoff(fence, RouterBarrier(control(rig["router"]), "navpy"),
                                        publish=records.append)
        navpy_handoff.on_terminal(Terminal.DOCKED, IDENTITY)
        timer = released_after(rig["board"])
        assert navpy_handoff.join(WAIT_S)
    else:
        timer = released_after(rig["board"])
    quiesce = stop.Quiesce(handoff_for(rig, case, events))
    record = quiesce.run(REASONS[case])
    timer.join()
    assert record["barrier"]["ok"] and "failed" not in record
    assert record["barrier"]["ack"]["already_closed"] is (case == "terminal")
    turned_away = refused(rig)
    navpy_reconnects(rig)
    board_reconnects(rig)
    rig["navpy"].send(heartbeat())                 # after the board came back: still closed
    runner_flies_rtl(rig)
    assert quiesce.run(stop.EXIT) is record        # idempotent: the same record, only the reason is added
    wait_until(lambda: refused(rig) >= turned_away + 3)
    result = stop.audit(rig["log"], record, records)
    assert result["forbidden"] == 0 and result["after_drain"] == 0 and result["ok"], result
    assert result["navpy_writes"] == 1             # only the write in progress at the barrier went out
    assert quiesce.describe() == {"reason": REASONS[case], "repeats": [stop.EXIT], "handoff": record}
    stats = rig["router"].report()["stats"]
    assert stats["blocked_navpy_SET_ATTITUDE_TARGET"] >= 1


def test_a_barrier_without_an_ack_fails_the_audit(tmp_path):
    dead = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dead.bind((LOOPBACK, 0))
    try:
        quiesce = stop.Quiesce(NavpyHandoff(lambda: {"stop": "already exited"},
                                            RouterBarrier(dead.getsockname(), "navpy", timeout_s=0.2),
                                            lambda: {"ended": "terminated"},
                                            lambda: {"rtl": "direct", "confirmed": True}))
        record = quiesce.run(stop.CRASH)
    finally:
        dead.close()
    assert record["rtl"] == "direct" and "no ack" in record["failed"]
    (tmp_path / "w.csv").write_text("1.0,navpy,0,\n", encoding="utf-8")
    result = stop.audit(tmp_path / "w.csv", record, [])
    assert result["after_drain"] is None and not result["ok"]


def test_a_raising_handoff_is_recorded_and_the_quiesce_never_raises():
    class Broken:
        record = None

        def before_rtl(self):
            raise RuntimeError("handoff broke")

    assert stop.Quiesce(Broken()).run(stop.ABORT)["failed"] == "quiesce raised"


def test_the_runners_last_quiesce_needs_a_started_navpy(tmp_path):
    calls: list = []
    quiesce = stop.Quiesce(NavpyHandoff(lambda: calls.append("stop") or {}, None, None, None))

    class Group:
        owned: list = []

    assert quiesce.end(Group(), stop.CRASH) is None and calls == [] and quiesce.reasons == []


# ---- the outcome: a deadline never yields DOCKED ------------------------------------------------------
DOCKED = {"trigger": "terminal", "outcome": "docked", "barrier": "acked", "gimbal_stop": "sent"}
STOPPED = {"trigger": "stop", "outcome": None, "barrier": "acked", "gimbal_stop": "sent"}


def test_only_a_terminal_record_that_ended_the_observation_gives_the_outcome():
    assert stop.outcome([DOCKED], stop.ENDED_BY_TERMINAL)["outcome"] == "docked"
    assert stop.outcome([DOCKED], stop.ENDED_BY_DEADLINE)["outcome"] is None      # printed after the deadline
    assert stop.outcome([STOPPED], stop.ENDED_BY_DEADLINE)["outcome"] is None
    assert stop.outcome([STOPPED], stop.ENDED_BY_TERMINAL)["outcome"] is None
    assert stop.outcome([], None)["outcome"] is None                               # a crash before any record
    assert stop.outcome([DOCKED], None)["outcome"] is None                         # the flight raised


def test_navpys_failed_barrier_or_gimbal_stop_is_a_problem():
    result = stop.outcome([{**DOCKED, "barrier": "failed", "barrier_error": "no ack", "gimbal_stop": "timeout"}],
                          stop.ENDED_BY_TERMINAL)
    assert result["problems"] == ["navpy's own barrier failed: no ack", "navpy's gimbal stop: timeout"]
    assert stop.outcome([{**DOCKED, "gimbal_stop": "no_gimbal"}], stop.ENDED_BY_TERMINAL)["problems"] == []


class ObservedBoard:
    """Each heartbeat wait advances the clock by one poll and may let navpy print part of its output."""

    def __init__(self, clock: list, out: Path, writes: dict) -> None:
        self.clock, self.out, self.writes, self.waits = clock, out, writes, 0

    def wait_message(self, kind, timeout):
        self.waits += 1
        with self.out.open("a", encoding="utf-8") as out:
            out.write(self.writes.get(self.waits, ""))
        self.clock[0] += timeout


RECORD_LINE = "NAVPY_TERMINAL_V1 " + json.dumps(DOCKED)


@pytest.mark.parametrize("writes, ended_by", [
    ({2: RECORD_LINE + "\n"}, stop.ENDED_BY_TERMINAL),                      # read before the deadline
    ({3: RECORD_LINE + "\n"}, stop.ENDED_BY_DEADLINE),                      # printed during the last wait
    ({2: RECORD_LINE[:30], 3: RECORD_LINE[30:] + "\n"}, stop.ENDED_BY_DEADLINE),   # ended only after the deadline
    ({1: RECORD_LINE[:30], 2: RECORD_LINE[30:] + "\n"}, stop.ENDED_BY_TERMINAL),   # split across two reads
    ({2: "NAVPY_TERMINAL_V1 {not json\n"}, stop.ENDED_BY_DEADLINE),         # a marker is not a record
])
def test_only_a_complete_record_read_before_the_deadline_ends_the_observation(tmp_path, monkeypatch, writes, ended_by):
    clock = [100.0]
    monkeypatch.setattr(stop.time, "monotonic", lambda: clock[0])
    out = tmp_path / "navpy.out"
    out.write_text("", encoding="utf-8")
    board = ObservedBoard(clock, out, writes)
    group = type("Group", (), {"check": lambda self: None})()
    assert stop.observe(board, group, out, 3 * stop.TERMINAL_POLL_S - 0.5) == ended_by


def test_navpys_records_are_read_from_its_output(tmp_path, capsys):
    print_terminal_record(STOPPED)
    out = tmp_path / "navpy.out"
    out.write_text("starting\n" + capsys.readouterr().out + "NAVPY_STOPPED_V1\n", encoding="utf-8")
    assert stop.navpy_records(out) == [STOPPED]
    assert stop.navpy_records(tmp_path / "missing.out") == []


# ---- navpy's own quiesce when stopped without an outcome ------------------------------------------------
def scripted(events: list, result: BarrierResult | None = None) -> tuple[TerminalHandoff, ScriptedBarrier]:
    fence = LinkFence()
    barrier = ScriptedBarrier(events, fence, result or BarrierResult({"op": "closed", "client": "navpy"}))
    return TerminalHandoff(fence, barrier, publish=lambda record: events.append(("record", record))), barrier


def test_a_stop_quiesces_without_an_outcome_and_a_later_docked_is_never_reported():
    events: list = []
    handoff, barrier = scripted(events)
    gimbal = handoff.gimbal_fence.wrap(lambda **_: RecordingGimbal(events))(server_ip=LOOPBACK, port=1)
    barrier.release.set()
    handoff.quiesce(TRIGGER_STOP)
    handoff.on_terminal(Terminal.DOCKED, IDENTITY)         # latched after the deadline's stop
    assert handoff.join(WAIT_S)
    records = [e[1] for e in events if e[0] == "record"]
    assert len(records) == 1 and records[0]["trigger"] == "stop" and records[0]["outcome"] is None
    assert records[0]["gimbal_stop"] == "sent" and barrier.calls == 1
    again = handoff.gimbal_fence.wrap(lambda **_: RecordingGimbal(events))(server_ip=LOOPBACK, port=1)  # reconnect
    again.requestGimbalSpeed(30, 0)
    gimbal.requestGimbalSpeed(30, 0)
    assert handoff.gimbal_fence.dropped == 2
    assert [e for e in events if e[0] == "gimbal"] == [("gimbal", 0x07), ("gimbal", 0x05)]


def test_a_terminal_outcome_before_the_stop_keeps_its_record():
    events: list = []
    handoff, barrier = scripted(events)
    barrier.release.set()
    handoff.on_terminal(Terminal.MISSED, IDENTITY)
    assert handoff.finish(TRIGGER_STOP)
    records = [e[1] for e in events if e[0] == "record"]
    assert [(r["trigger"], r["outcome"]) for r in records] == [("terminal", "missed")]


def test_finish_reports_a_record_that_is_not_out_in_time():
    events: list = []
    handoff, barrier = scripted(events)
    assert handoff.finish(TRIGGER_SHUTDOWN, timeout_s=0.1) is False
    barrier.release.set()
    assert handoff.join(WAIT_S)


@pytest.mark.parametrize("stopped, trigger", [(True, "stop"), (False, "shutdown")])
def test_navpys_cleanup_quiesces_first(stopped, trigger):
    """Owned last, the handoff closes first: navigation and vision stop behind closed fences, the gimbal stopped."""
    events: list = []
    handoff, barrier = scripted(events)
    handoff.gimbal_fence.wrap(lambda **_: RecordingGimbal(events))(server_ip=LOOPBACK, port=1)
    barrier.release.set()
    resources = RuntimeResourceStack()
    resources.own_quiescence("vision", lambda: events.append(("vision stops", handoff.fence.closed)))
    own_quiesce(resources.own_quiescence, handoff, lambda: stopped)
    resources.close()
    kinds = [e[0] for e in events]
    assert kinds == ["gimbal", "gimbal", "barrier", "record", "vision stops"]
    assert events[-1] == ("vision stops", True) and events[3][1]["trigger"] == trigger


@pytest.mark.parametrize("fails", ["stopped before start", "navigation start", "controller"])
def test_navpys_start_failing_at_any_point_quiesces_before_any_component_stops(monkeypatch, fails):
    """The quiesce is owned after each writer: wherever the start ends, the fences close before anything stops."""
    from types import SimpleNamespace
    from unittest.mock import MagicMock

    from navpy.app import runtime_composition as composition
    from navpy.app.runtime_stop import StoppedBeforeStart

    events: list = []
    handoff, barrier = scripted(events)
    barrier.release.set()

    def stops(name):
        return lambda *_: events.append((f"{name} stops", handoff.fence.closed))

    def start():
        if fails == "navigation start":
            raise RuntimeError("navigation failed to start")

    def controller(*_args, **_kw):
        if fails == "controller":
            raise RuntimeError("controller failed")
        return SimpleNamespace(stop=stops("controller"), set_network=MagicMock())

    vision = SimpleNamespace(stop=stops("vision"), geo_ref=object(), coordination=object(), approach_kind=object(),
                             dock_contact=None, profile=object(), is_quiescent=True)
    foundation = SimpleNamespace(vehicle=MagicMock(), vision_args=object(), logger=MagicMock(), zc_util=None,
                                 navigation_args=object(), scheduler_cadence=None, network=MagicMock(),
                                 vehicle_close_gate=SimpleNamespace(stop_vision=lambda stop, _q: stop()),
                                 terminal_handoff=handoff, camera_frames=None)
    for name, value in [("VisionController", lambda *_a, **_k: vision), ("MissionPlanner", lambda *_a: object()),
                        ("MissionPlannerArgs", lambda _a: object()), ("NavArgs", lambda *_a: object()),
                        ("Navigation", lambda *_a, **_k: SimpleNamespace(start=start, stop=stops("navigation"))),
                        ("NavController", controller)]:
        monkeypatch.setattr(composition, name, value)
    resources = RuntimeResourceStack()
    with pytest.raises((StoppedBeforeStart, RuntimeError)):
        composition._build_session(object(), foundation, resources,
                                   {"stop_requested": lambda: fails == "stopped before start"})
    resources.close()
    stopped = [e for e in events if e[0].endswith(" stops")]
    assert stopped and all(closed for _, closed in stopped)
    records = [e[1] for e in events if e[0] == "record"]
    assert len(records) == 1 and events.index(("record", records[0])) < events.index(stopped[0])
    assert records[0]["trigger"] == ("stop" if fails == "stopped before start" else "shutdown")


def test_a_stop_during_the_vehicle_build_quiesces_before_the_vehicle_closes(monkeypatch):
    from types import SimpleNamespace
    from unittest.mock import MagicMock

    from navpy.app import runtime_composition as composition

    events: list = []
    handoff, barrier = scripted(events)
    barrier.release.set()
    stopped = {"now": False}

    def create_vehicle(_args, _logger, *, bus, stop_requested):
        stopped["now"] = True                            # the stop arrives during the vehicle build
        return SimpleNamespace(close=lambda: events.append(("vehicle closed", handoff.fence.closed)))

    for name, value in [("stop_hooks", lambda *_a: {"stop_requested": lambda: stopped["now"],
                                                     "acknowledge_stop": lambda: events.append(("ack",))}),
                        ("ConnArgs", lambda _a: SimpleNamespace(source_system=1)), ("LoggerArgs", lambda _a: object()),
                        ("initialize_logger", lambda *_a: MagicMock()), ("build_terminal_handoff", lambda _a: handoff),
                        ("open_vehicle_bus", lambda *_a: object()), ("create_vehicle", create_vehicle),
                        ("create_network", MagicMock(side_effect=AssertionError("built past the stop")))]:
        monkeypatch.setattr(composition, name, value)
    monkeypatch.setattr(composition.GroundStationLoggerNetwork, "create", MagicMock())
    assert composition.compose_runtime(SimpleNamespace()).run() == 0
    kinds = [e[0] for e in events]
    assert kinds == ["barrier", "record", "vehicle closed", "ack"]
    assert events[1][1]["trigger"] == "stop" and events[2] == ("vehicle closed", True)


def test_a_failed_vehicle_build_still_quiesces_and_publishes(monkeypatch):
    from types import SimpleNamespace
    from unittest.mock import MagicMock

    from navpy.app import runtime_composition as composition

    events: list = []
    handoff, barrier = scripted(events)
    barrier.release.set()

    def create_vehicle(_args, _logger, *, bus, stop_requested):
        events.append(("vehicle rolled back",))        # vehicle_build_transaction's own rollback, then the raise
        raise TimeoutError("no heartbeat")

    for name, value in [("ConnArgs", lambda _a: SimpleNamespace(source_system=1)), ("LoggerArgs", lambda _a: object()),
                        ("initialize_logger", lambda *_a: MagicMock()), ("build_terminal_handoff", lambda _a: handoff),
                        ("open_vehicle_bus", lambda *_a: object()), ("create_vehicle", create_vehicle)]:
        monkeypatch.setattr(composition, name, value)
    monkeypatch.setattr(composition.GroundStationLoggerNetwork, "create", MagicMock())
    with pytest.raises(TimeoutError):
        composition.compose_runtime(SimpleNamespace())
    assert [e[0] for e in events] == ["vehicle rolled back", "barrier", "record"]
    assert events[-1][1]["trigger"] == "shutdown" and handoff.fence.closed


@pytest.mark.filterwarnings("ignore::pytest.PytestUnhandledThreadExceptionWarning")     # the raise under test
def test_a_handoff_that_raises_is_not_a_published_record():
    events: list = []
    fence = LinkFence()

    class Raising(ScriptedBarrier):
        def close_endpoint(self):
            raise OSError("no socket")

    handoff = TerminalHandoff(fence, Raising(events, fence, BarrierResult({})), publish=events.append)
    assert handoff.finish(TRIGGER_SHUTDOWN, timeout_s=WAIT_S) is False
    assert events == [] and fence.closed


def test_without_router_control_nothing_is_owned():
    owned: list = []
    own_quiesce(lambda label, action: owned.append(label), None, lambda: True)
    assert owned == []


def test_a_write_after_the_stop_quiesce_is_fenced():
    events: list = []
    handoff, barrier = scripted(events)

    class Link:
        written: list = []

        def write(self, buf: bytes) -> None:
            self.written.append(buf)

    link = Link()
    handoff.fence.install(link, threading.RLock())
    barrier.release.set()
    handoff.quiesce(TRIGGER_STOP)
    link.write(b"navigation's stop: AUTO restart")
    assert handoff.join(WAIT_S) and link.written == [] and handoff.fence.dropped == 1


# ---- the runner's exit paths ----------------------------------------------------------------------------
class RecordingQuiesce(stop.Quiesce):
    def __init__(self, log: list) -> None:
        super().__init__(NavpyHandoff(lambda: log.append("navpy stop") or {}, None, None, None))
        self._log = log

    def run(self, reason: str) -> dict:
        self._log.append(f"quiesce {reason}")
        self.reasons.append(reason)
        self._handoff.record = self._handoff.record or {"barrier": {"ok": True, "ack": None}}
        return self._handoff.record


def _runner(tmp_path, monkeypatch, fly, census=None):
    log: list = []

    class FakeBoard:
        def __init__(self, *args, **kwargs) -> None:
            self.workdir, self.record = Path(args[1]), {"hitl": {"bridge_exit": 0}}

        def start(self):
            log.append("board start")

        def stop(self):
            ignored = all(signal.getsignal(s) is signal.SIG_IGN for s in (signal.SIGINT, signal.SIGTERM))
            log.append("board stop" if ignored else "board stop, signals not ignored")

        def report(self):
            return {"link_health": {"ok": True}}

    def start(group, spec):
        log.append(f"start {spec.role}")
        owned = Owned(ProcSpec(spec.role, (), None, 0.0), runner_rig.FakeHandle(log, spec.role), None, "t")
        group.owned.append(owned)
        return owned

    for name, value in [("_record", lambda *a: {"run": {}, "commissioning": runner_rig.NO_FAULT.record()}),
                        ("_addresses", lambda: ("172.0.0.2", "172.0.0.1")), ("Board", FakeBoard),
                        ("board_param_files", lambda *a: []), ("_board_scripts", lambda *a: None), ("_fly", fly)]:
        monkeypatch.setattr(approach, name, value)
    monkeypatch.setattr(approach.scenario_run, "service", runner_rig.fake_service)
    monkeypatch.setattr(approach.approach_records, "write_record",
                        lambda record, run_dir: runner_rig._written(run_dir / "approach_record.json"))
    monkeypatch.setattr(approach.chain, "port_free", lambda protocol, port: True)
    monkeypatch.setattr(approach.chain, "unreal_running", lambda: None)
    passing = {"ok": True, "roles": {}, "problems": []}
    monkeypatch.setattr(approach.owners, "census", census or (lambda *a, **k: passing))
    monkeypatch.setattr(approach.handles, "windows_udp_in_use", lambda ports: None)
    monkeypatch.setattr(approach.chain, "publisher_profile", lambda paths: tmp_path / "profile.json")
    monkeypatch.setattr(approach.chain, "win_path", lambda path: "C:/x")
    monkeypatch.setattr(approach.ProcessGroup, "start", start)
    monkeypatch.setattr(approach.stop.Quiesce, "of", classmethod(lambda cls, group: RecordingQuiesce(log)))
    code = approach.approach(runner_rig.approach_args(tmp_path))
    run_dir = next((tmp_path / "runs").iterdir())
    return code, log, json.loads((run_dir / "approach_run.json").read_text(encoding="utf-8"))


def _raises(exc: BaseException):
    def fly(board, group, run, quiesce):
        raise exc
    return fly


def test_a_signal_as_the_run_arms_still_quiesces_and_stops(tmp_path, monkeypatch):
    _interrupted_once(monkeypatch, stop.Aborted("signal 15"), 1)
    code, log, run = _runner(tmp_path, monkeypatch, _raises(AssertionError("not reached")))
    assert code == 1 and "board stop" in log and run["error"] == "Aborted: signal 15"
    assert run["handoff"] is None                    # navpy never started: nothing to quiesce


@pytest.mark.parametrize("exc", [stop.Aborted("signal 1"), KeyboardInterrupt()])
def test_a_signal_as_the_cleanup_begins_still_quiesces_and_stops(tmp_path, monkeypatch, exc):
    real, calls = stop.shield, []

    def shield(saved):
        calls.append(1)
        if len(calls) == 1:
            raise exc                                # a signal at shield()'s very start, before its own retry
        real(saved)
    monkeypatch.setattr(stop, "shield", shield)
    before = signal.getsignal(signal.SIGTERM), signal.getsignal(signal.SIGINT)
    code, log, run = _runner(tmp_path, monkeypatch, _raises(stop.Aborted("signal 15")))
    assert len(calls) == 2 and "board stop" in log and run["quiesce"]["reason"] == stop.ABORT
    assert (signal.getsignal(signal.SIGTERM), signal.getsignal(signal.SIGINT)) == before


@pytest.mark.parametrize("exc, reason", [(RunnerError("navpy exited with code -9 during the run"), stop.CRASH),
                                         (KeyboardInterrupt(), stop.ABORT), (stop.Aborted("signal 15"), stop.ABORT)])
def test_a_crash_or_an_abort_quiesces_before_the_boards_stop(tmp_path, monkeypatch, exc, reason):
    before = signal.getsignal(signal.SIGTERM), signal.getsignal(signal.SIGINT)
    code, log, run = _runner(tmp_path, monkeypatch, _raises(exc))
    assert code == 1 and (signal.getsignal(signal.SIGTERM), signal.getsignal(signal.SIGINT)) == before
    assert "board stop" in log                       # a second signal during the cleanup is ignored
    assert log.index(f"quiesce {reason}") < log.index("board stop")
    assert (run["quiesce"]["reason"], run["quiesce"]["repeats"]) == (reason, []) and run["outcome"]["outcome"] is None


def test_a_deadline_quiesces_once_and_the_exit_only_repeats_it(tmp_path, monkeypatch):
    def fly(board, group, run, quiesce):
        run["flight"] = {"ended_by": stop.ENDED_BY_DEADLINE}
        quiesce.run(stop.DEADLINE)

    code, log, run = _runner(tmp_path, monkeypatch, fly)
    assert (run["quiesce"]["reason"], run["quiesce"]["repeats"]) == ("deadline", ["exit"])
    assert run["outcome"]["outcome"] is None and run["outcome"]["ended_by"] == "deadline"
    assert log.index("quiesce deadline") < log.index("board stop")
    assert code == 1 and not run["observation"]["ok"]      # no write log here: the audit cannot pass


def _census(bad_phase: str):
    """A census that finds a foreign owner of the emulator at *bad_phase* (preflight: running is False)."""
    calls = []

    def census(group, tables, own_pid, running):
        calls.append(running)
        phase = "preflight" if not running else ("ready" if len(calls) == 2 else "end")
        if phase != bad_phase:
            return {"ok": True, "roles": {}, "problems": []}
        return {"ok": False, "roles": {}, "problems": ["emulator: exactly 1 owner (emulator), found emulator, foreign"]}
    return census


def test_a_foreign_owner_before_the_run_refuses_before_anything_starts(tmp_path, monkeypatch):
    code, log, run = _runner(tmp_path, monkeypatch, _raises(AssertionError("not reached")), _census("preflight"))
    assert code == 1 and "owner census (preflight)" in run["error"]
    assert not [entry for entry in log if entry.startswith("start ") or entry == "board start"]


def test_a_foreign_owner_at_the_end_fails_the_run_and_every_process_still_stops(tmp_path, monkeypatch):
    def fly(board, group, run, quiesce):
        run["flight"] = {"ended_by": stop.ENDED_BY_DEADLINE}
        quiesce.run(stop.DEADLINE)

    code, log, run = _runner(tmp_path, monkeypatch, fly, _census("end"))
    assert code == 1 and "error" not in run and not run["owners"]["end"]["ok"]
    assert any(p.startswith("owners end: emulator") for p in approach.owners.verdicts(run)[1])
    assert "board stop" in log and not run["orphans"]
    started = [entry.split()[1] for entry in log if entry.startswith("start ")]
    assert started and all(f"{role} stop" in log for role in started)


def test_sigterm_raises_an_abort():
    with pytest.raises(stop.Aborted):
        stop.raise_aborted(signal.SIGTERM, None)


@pytest.mark.parametrize("name", [n for n in ("SIGTERM", "SIGHUP") if hasattr(signal, n)])
def test_sigterm_and_sighup_abort_a_run_and_are_ignored_in_its_cleanup(name):
    """A hangup (the WSL session went away) would end the runner at once, with nothing stopped."""
    signum, handlers = getattr(signal, name), {}
    before = {s: signal.getsignal(s) for s in (signal.SIGINT, *stop.ABORT_SIGNALS)}
    try:
        with pytest.raises(stop.Aborted):
            stop.arm(handlers)
            signal.raise_signal(signum)
        stop.shield(handlers)
        signal.raise_signal(signum)                 # ignored: the cleanup goes on
        signal.raise_signal(signal.SIGINT)
    finally:
        stop.restore(handlers)
    assert {s: signal.getsignal(s) for s in before} == before


def _interrupted_once(monkeypatch, exc: BaseException, after: int):
    """signal.signal that raises exc (a signal arriving mid-swap) right after its after-th call took effect."""
    real, calls = signal.signal, []

    def interrupted(signum, handler):
        previous = real(signum, handler)
        calls.append(signum)
        if len(calls) == after:
            raise exc
        return previous
    monkeypatch.setattr(stop.signal, "signal", interrupted)


@pytest.mark.parametrize("after", sorted({1, 2, len(stop.ABORT_SIGNALS) + 1}))
def test_a_signal_while_the_cleanup_shields_still_leaves_every_signal_ignored(monkeypatch, after):
    before = {s: signal.getsignal(s) for s in (signal.SIGINT, *stop.ABORT_SIGNALS)}
    handlers: dict = {}
    try:
        stop.arm(handlers)
        _interrupted_once(monkeypatch, stop.Aborted("signal 1"), after)
        stop.shield(handlers)                        # does not raise: the cleanup's stops all run
        assert all(signal.getsignal(s) is signal.SIG_IGN for s in before)
    finally:
        monkeypatch.undo()
        stop.restore(handlers)
    assert {s: signal.getsignal(s) for s in before} == before


def test_a_signal_while_the_run_arms_is_an_abort_that_loses_no_handler(monkeypatch):
    before = {s: signal.getsignal(s) for s in stop.ABORT_SIGNALS}
    handlers: dict = {}
    _interrupted_once(monkeypatch, stop.Aborted("signal 15"), 1)
    try:
        with pytest.raises(stop.Aborted):
            stop.arm(handlers)
    finally:
        monkeypatch.undo()
        stop.restore(handlers)
    assert {s: signal.getsignal(s) for s in before} == before
