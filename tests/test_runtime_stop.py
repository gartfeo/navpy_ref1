"""A stop request ends the run loop; the ack follows a clean cleanup only."""

from __future__ import annotations

import io

import pytest

from navpy.app import runtime_stop
from navpy.app.navpy_argparse import make_parser
from navpy.app.runtime_resources import RuntimeResourceStack, RuntimeSession
from navpy.app.runtime_stop import (
    NAVPY_STOP_REQUEST,
    NAVPY_STOPPED_MARKER,
    StdinStopRequest,
)


def _session(stack, events, *, requested):
    return RuntimeSession(
        resources=stack,
        start_vision=lambda: None,
        start_controller=lambda: None,
        health_check=lambda: None,
        ui_step=lambda: events.append("ui") or True,
        sleep=lambda _seconds: None,
        stop_requested=lambda: requested(),
        acknowledge_stop=lambda: events.append("ack"),
    )


def test_a_stop_request_ends_the_loop_and_acks_after_cleanup():
    events: list[str] = []
    stack = RuntimeResourceStack()
    stack.own("logger", lambda: events.append("logger closed"))
    # The request arrives during the second UI step.
    requested = lambda: events.count("ui") >= 2  # noqa: E731

    assert _session(stack, events, requested=requested).run() == 0
    assert events == ["ui", "ui", "logger closed", "ack"]


def test_a_failed_cleanup_sends_no_ack():
    events: list[str] = []
    stack = RuntimeResourceStack()
    stack.own("navigation", lambda: (_ for _ in ()).throw(TimeoutError("join")))

    with pytest.raises(Exception) as failure:
        _session(stack, events, requested=lambda: True).run()
    assert "join" in repr(failure.value.exceptions)
    assert "ack" not in events


def test_the_request_latches_only_on_the_exact_line():
    request = StdinStopRequest(io.StringIO(f"noise\n{NAVPY_STOP_REQUEST}x\n"))
    request.start()
    request._thread.join(5.0)
    assert request.requested() is False

    request = StdinStopRequest(io.StringIO(f"noise\n{NAVPY_STOP_REQUEST}\r\n"))
    request.start()
    request._thread.join(5.0)
    assert request.requested() is True


def test_the_ack_flushes_pose_cadence_first(monkeypatch, capsys):
    from navpy.core.kernel.logging import pose_cadence_debug

    order: list[str] = []
    monkeypatch.setattr(pose_cadence_debug, "stop_flusher", lambda: order.append("stop"))
    monkeypatch.setattr(pose_cadence_debug, "dump_all",
                        lambda *, strict: order.append(f"dump strict={strict}"))

    runtime_stop.acknowledge_stop()

    assert order == ["stop", "dump strict=True"]
    assert capsys.readouterr().out == f"{NAVPY_STOPPED_MARKER}\n"


def test_the_flag_is_opt_in():
    parser = make_parser()
    assert parser.parse_args(["-ss", "4"]).stop_on_stdin is False
    assert parser.parse_args(["-ss", "4", "--stop-on-stdin"]).stop_on_stdin is True


def test_a_failed_final_pose_write_raises_instead_of_acking(monkeypatch, tmp_path):
    from navpy.core.kernel.logging import pose_cadence_debug as pose

    monkeypatch.setattr(pose, "ENABLED", True)
    monkeypatch.setattr(pose, "_OUTPUT_DIR", str(tmp_path))
    monkeypatch.setattr(pose, "_buffers", {(4, "observation"): [(1.0, 2.0, 3.0, "ok")]})
    monkeypatch.setattr(pose, "_written", {})
    # A directory where the CSV must go makes the write fail.
    (tmp_path / f"pose_cadence_observation_uav_4_pid_{__import__('os').getpid()}.csv").mkdir()

    pose.dump_all()  # the periodic flush keeps going
    with pytest.raises(OSError):
        pose.dump_all(strict=True)


# -- lockstep: the lifecycle stop (``lockstep_slice.md``, "Participant stop") --------


@pytest.fixture
def baton(monkeypatch):
    """A baton bound through a test-only binding (the process binding is once only)."""
    from navpy.core.kernel import runtime_time as rt
    from navpy.sim.lockstep.baton import BatonRuntime
    from navpy.sim.lockstep.lockstep_runtime import LockstepRuntime

    runtime = BatonRuntime()
    binding = rt.RuntimeBinding()
    binding.bind(LockstepRuntime.create(runtime, epoch_us=0, seed=1, ordinal=1))
    monkeypatch.setattr(rt, "_binding", binding)
    return runtime


def _no_pose_flush(monkeypatch, order):
    from navpy.core.kernel.logging import pose_cadence_debug

    monkeypatch.setattr(pose_cadence_debug, "stop_flusher", lambda: order.append("stop"))
    monkeypatch.setattr(pose_cadence_debug, "dump_all",
                        lambda *, strict: order.append(f"dump strict={strict}"))


def test_a_lifecycle_stop_latches_and_acks_its_first_id_after_the_strict_flush(baton, monkeypatch):
    order: list = []
    _no_pose_flush(monkeypatch, order)
    request = runtime_stop.LifecycleStopRequest(lambda stop_id: order.append(("stopped", stop_id)))
    assert request.requested() is False
    baton.tick(20_000, lambda: request.deliver_stop((2, 5)))
    baton.tick(40_000, lambda: request.deliver_stop((2, 6)))  # a duplicate: the first id stands
    assert request.requested() is True
    baton.spawn(request.acknowledge, name="run-loop")
    baton.run_until_quiescent()
    assert order == ["stop", "dump strict=True", ("stopped", (2, 5))]


def test_a_drain_end_stops_the_loop_and_acks_with_no_event(baton, monkeypatch):
    order: list = []
    _no_pose_flush(monkeypatch, order)
    request = runtime_stop.LifecycleStopRequest(lambda stop_id: order.append(("stopped", stop_id)))
    baton.tick(20_000, request.deliver_end)
    assert request.requested() is True
    baton.spawn(request.acknowledge, name="run-loop")
    baton.run_until_quiescent()
    assert order == ["stop", "dump strict=True"]


def test_the_lifecycle_stop_drives_the_unchanged_run_loop(baton, monkeypatch):
    """The stop path is the stdin one: cleanup, then the ack; a failed cleanup acks nothing."""
    from navpy.core.kernel import runtime_time as rt

    _no_pose_flush(monkeypatch, [])
    for cleanup_fails in (False, True):
        events: list = []
        request = runtime_stop.LifecycleStopRequest(lambda stop_id: events.append(("stopped", stop_id)))
        outcome: dict = {}

        def run() -> None:
            stack = RuntimeResourceStack()
            if cleanup_fails:
                stack.own("navigation", lambda: (_ for _ in ()).throw(TimeoutError("join")))
            else:
                stack.own("logger", lambda: events.append("logger closed"))
            session = RuntimeSession(
                resources=stack, start_vision=lambda: None, start_controller=lambda: None,
                health_check=lambda: None, ui_step=lambda: True, sleep=rt.sleep,
                stop_requested=request.requested, acknowledge_stop=request.acknowledge,
            )
            try:
                outcome["code"] = session.run()
            except Exception as error:
                outcome["error"] = error

        baton.spawn(run, name="run-loop")
        baton.run_until_quiescent()
        assert outcome == {}  # parked in the loop's sleep until the record arrives
        baton.tick(baton.now_us + 20_000, lambda: request.deliver_stop((2, 1)))
        if cleanup_fails:
            assert "error" in outcome and ("stopped", (2, 1)) not in events
        else:
            assert outcome == {"code": 0}
            assert events == ["logger closed", ("stopped", (2, 1))]


def test_the_lifecycle_flag_is_opt_in_and_needs_the_participant(monkeypatch):
    from navpy.app.runtime_composition import compose_runtime
    from navpy.app.runtime_ports import RuntimePorts

    parser = make_parser()
    assert parser.parse_args(["-ss", "4"]).stop_on_lifecycle is False
    lifecycle = parser.parse_args(["-ss", "4", "--stop-on-lifecycle"])
    with pytest.raises(ValueError, match="lifecycle stop"):
        compose_runtime(lifecycle)  # wall ports: no lifecycle stop to latch
    both = parser.parse_args(["-ss", "4", "--stop-on-lifecycle", "--stop-on-stdin"])
    with pytest.raises(ValueError, match="exclusive"):
        compose_runtime(both, RuntimePorts(lifecycle_stop=object()))


def test_a_stop_during_startup_starts_nothing_and_acks_after_cleanup():
    """Todo 105: values read after the stop are defaults; never start on them."""
    events: list[str] = []
    stack = RuntimeResourceStack()
    stack.own("vehicle", lambda: events.append("vehicle closed"))
    session = RuntimeSession(
        resources=stack,
        start_vision=lambda: events.append("vision"),
        start_controller=lambda: events.append("controller"),
        health_check=lambda: None,
        ui_step=lambda: events.append("ui") or True,
        sleep=lambda _seconds: None,
        stop_requested=lambda: True,
        acknowledge_stop=lambda: events.append("ack"),
    )
    assert session.run() == 0
    assert events == ["vehicle closed", "ack"]


def test_the_vehicle_build_sees_the_stop_and_the_session_starts_nothing(monkeypatch):
    from types import SimpleNamespace
    from unittest.mock import MagicMock

    from navpy.app import runtime_composition as composition

    events: list[str] = []
    stopped = {"now": False}
    built_with = {}

    def create_vehicle(_args, _logger, *, bus, stop_requested):
        built_with["stop_requested"] = stop_requested
        stopped["now"] = True  # the stop arrives during the vehicle build
        return SimpleNamespace(close=lambda: events.append("vehicle closed"))

    monkeypatch.setattr(composition, "stop_hooks", lambda *_args: {
        "stop_requested": lambda: stopped["now"],
        "acknowledge_stop": lambda: events.append("ack"),
    })
    monkeypatch.setattr(composition, "ConnArgs", lambda _args: SimpleNamespace(source_system=1))
    monkeypatch.setattr(composition, "LoggerArgs", lambda _args: object())
    monkeypatch.setattr(composition.GroundStationLoggerNetwork, "create", MagicMock())
    monkeypatch.setattr(composition, "initialize_logger", lambda *_args: MagicMock())
    monkeypatch.setattr(composition, "build_terminal_handoff", lambda _args: None)
    monkeypatch.setattr(composition, "open_vehicle_bus", lambda *_args: object())
    monkeypatch.setattr(composition, "create_vehicle", create_vehicle)
    monkeypatch.setattr(composition, "create_network",
                        MagicMock(side_effect=AssertionError("built past the stop")))

    session = composition.compose_runtime(SimpleNamespace())

    assert built_with["stop_requested"]() is True
    assert session.run() == 0
    assert events == ["vehicle closed", "ack"]
