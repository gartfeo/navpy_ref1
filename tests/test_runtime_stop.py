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
