"""The supervisor starts the MAVLink record after verification and never lets it fail a launch."""

from __future__ import annotations

import contextlib
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "scripts"))

import swarm_run_runner as runner  # noqa: E402
import swarm_run_speed_control as speed_control  # noqa: E402
import swarm_run_verification_model as model  # noqa: E402
import swarm_run_verifier as verifier  # noqa: E402


class Process:
    def __init__(self, events, code=0):
        self.events, self.code = events, code

    def wait(self):
        self.events.append("wait")
        return self.code


class Recorder:
    tlog = Path("run/mavlink.tlog")

    def __init__(self, events, stopped=True, fails=False):
        self.events, self.stopped, self.fails = events, stopped, fails

    def stop(self):
        self.events.append("stop")
        if self.fails:
            raise RuntimeError("join broke")
        return self.stopped


def _supervise(events, *, passed=True, start_recorder=None, code=0):
    emitted = []
    launch = runner.LaunchServices(
        start=lambda command: (Process(events, code), []),
        verify=lambda *a: events.append("verify") or SimpleNamespace(rates={4: 10.0, 5: 10.0, 6: 10.0}),
        passed=lambda *a: passed,
        describe=lambda *a: "no heartbeat",
        speed_fault=lambda *a: False,
        force_speed=lambda *a: None,
        kill=lambda *a: events.append("kill"),
    )
    registry = MagicMock()
    registry.sitl_launch_lock.return_value = contextlib.nullcontext(True)
    runtime = runner.RuntimeServices(
        registry, MagicMock(), lambda chat: None,
        lambda text, err=False: emitted.append((text, err)),
        lambda *a, **k: None, lambda chat: None,
        lambda *a, **k: events.append("retire"), lambda *a, **k: "cmd",
        **({} if start_recorder is None else {"start_recorder": start_recorder}),
    )
    args = SimpleNamespace(instances=3, speedup=10, dist=0, single_boot=True)
    with pytest.raises(SystemExit) as exit_info:
        runner.run_supervisor(1, args, launch=launch, runtime=runtime, max_attempts=1)
    return exit_info.value.code, emitted


def test_records_after_verification_and_stops_after_the_swarm_exits():
    events = []

    def start(chat):
        events.append(("record", chat))
        return Recorder(events)

    code, emitted = _supervise(events, start_recorder=start, code=3)
    assert events == ["verify", ("record", 1), "wait", "retire", "stop"]
    assert code == 3
    assert ("SITL chat 1: MAVLink record to " + str(Recorder.tlog), False) in emitted


def test_a_failed_launch_records_nothing():
    events = []
    code, _ = _supervise(events, passed=False, start_recorder=lambda chat: events.append("record"))
    assert "record" not in events and code == 1


def test_nothing_records_by_default():
    events = []
    code, emitted = _supervise(events)
    assert events == ["verify", "wait", "retire"] and code == 0
    assert not [text for text, _ in emitted if "MAVLink" in text]


def test_a_recorder_that_cannot_start_changes_nothing_but_says_so():
    events = []

    def refused(chat):
        raise OSError(10048, "verify port in use")

    code, emitted = _supervise(events, start_recorder=refused, code=0)
    assert events == ["verify", "wait", "retire"] and code == 0
    assert any("MAVLink record not started" in text and err for text, err in emitted)


@pytest.mark.parametrize("recorder_kwargs, said", [
    ({"stopped": False}, "did not stop in time"),
    ({"fails": True}, "stop failed (join broke)"),
])
def test_a_recorder_that_will_not_stop_keeps_the_exit_code(recorder_kwargs, said):
    events = []
    code, emitted = _supervise(
        events, start_recorder=lambda chat: Recorder(events, **recorder_kwargs), code=5,
    )
    assert code == 5 and events[-1] == "stop"
    assert any(said in text and err for text, err in emitted)


def _identity(call) -> tuple:
    return call.kwargs["source_system"], call.kwargs["source_component"]


def test_the_launcher_speaks_as_255_190_on_the_verify_port():
    # The backend is 255/0 on the monitor port; the router would not mirror its
    # frames to the verify port if the launcher had used that id there.
    assert (model.LAUNCHER_SOURCE_SYSTEM, model.LAUNCHER_SOURCE_COMPONENT) == (255, 190)
    with patch.object(verifier.mavutil, "mavlink_connection", side_effect=OSError("stop")) as connect:
        verifier.verify_swarm(1, 3, 10, 0.0)
    assert _identity(connect.call_args) == (255, 190)
    with patch.object(speed_control.mavutil, "mavlink_connection", side_effect=OSError("stop")) as connect:
        speed_control.force_speedup(1, {4, 5, 6}, 10)
    assert _identity(connect.call_args) == (255, 190)
