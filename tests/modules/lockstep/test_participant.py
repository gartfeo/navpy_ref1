"""Participant process identity and the drain flush."""

from __future__ import annotations

import logging
import os

import pytest

from navpy.app import runtime_ready, runtime_stop
from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.config.logger_args import LoggerArgsStub
from navpy.core.kernel.logging import pose_cadence_debug as pose
from navpy.core.kernel.logging.log_output_latch import LatchingRotatingFileHandler, close_strictly
from navpy.core.kernel.logging.logger_factory import LOG_DIR_ENV, initialize_logger
from navpy.sim.lockstep.baton import BatonRuntime, LockstepFault
from navpy.sim.lockstep.lockstep_runtime import LockstepRuntime
from navpy.sim.lockstep.participant import LockstepIdentity, Participant

SYS_ID = 6


def _ready_record(capsys) -> dict:
    runtime_ready.emit_runtime_ready({"mission_items": 4})
    return runtime_ready.parse_runtime_ready(capsys.readouterr().out.strip())


def test_wall_mode_reports_the_os_pid(capsys):
    assert _ready_record(capsys) == {"mission_items": 4, "process_pid": os.getpid()}


def test_lockstep_reports_the_ordinal_and_generation(capsys, monkeypatch):
    monkeypatch.setattr(runtime_ready, "_identity", runtime_ready._identity)
    runtime_ready.set_process_identity(LockstepIdentity(ordinal=2, generation=5))
    assert _ready_record(capsys) == {
        "generation": 5, "mission_items": 4, "ordinal": 2, "process_pid": 2,
    }
    with pytest.raises(ValueError, match="start at 1"):
        LockstepIdentity(ordinal=0, generation=0)


@pytest.fixture
def participant(monkeypatch, tmp_path):
    """A participant bound through a test-only binding (the process binding is once only)."""
    runtime = LockstepRuntime.create(BatonRuntime(), epoch_us=0, seed=1, ordinal=1)
    binding = rt.RuntimeBinding()
    binding.bind(runtime)
    monkeypatch.setattr(rt, "_binding", binding)
    monkeypatch.setenv(LOG_DIR_ENV, str(tmp_path / "logs"))
    monkeypatch.setattr(pose, "ENABLED", True)
    monkeypatch.setattr(pose, "_OUTPUT_DIR", str(tmp_path / "pose"))
    monkeypatch.setattr(pose, "_buffers", {})
    monkeypatch.setattr(pose, "_written", {})
    yield Participant(runtime, LockstepIdentity(1, 0), uninstalls=())
    for handler in logging.getLogger(f"vehicle_logger_{SYS_ID}").handlers:
        handler.close()


class _FailingStream:
    """A file stream whose writes fail: the handler swallows it in ``handleError``."""

    def write(self, _text: str) -> None:
        raise OSError("disk full")

    def flush(self) -> None:
        pass

    def close(self) -> None:
        pass


def _break_the_file_sink() -> None:
    (handler,) = [h for h in logging.getLogger(f"vehicle_logger_{SYS_ID}").handlers
                  if isinstance(h, LatchingRotatingFileHandler)]
    handler.stream.close()
    handler.stream = _FailingStream()


def _strict(logger) -> list:
    return [lambda: close_strictly(logger), runtime_stop.flush_pose_cadence]


def _end(participant: Participant, flushes_for, *, same_turn: bool = False, before_end=None) -> dict:
    """Build the logger, then log, record and drain-flush at once.

    Normally the end is a later baton turn, so the log worker has started and
    parked; ``same_turn`` ends in the build turn, before the worker ever ran.
    """
    baton = participant.runtime.baton
    built = {}
    result = {}

    def build() -> None:
        built["logger"] = initialize_logger(LoggerArgsStub(), SYS_ID)
        if before_end is not None:
            before_end()
        if same_turn:
            end()

    def end() -> None:
        logger = built["logger"]
        log_file = os.path.join(logger.log_path, f"uav_{SYS_ID}_navigation.log")
        logger.info("before the end")
        pose.record_observation(SYS_ID, rt.monotonic(), 1.0, 2.0, "fresh")
        with open(log_file, encoding="utf-8") as fh:
            result["before"] = fh.read()  # the worker has not run since
        result["flushed"] = participant.drain_flush(flushes_for(logger))
        with open(log_file, encoding="utf-8") as fh:
            result["after"] = fh.read()

    baton.spawn(build, name="build")
    baton.run_until_quiescent()  # the log worker starts and parks on its queue
    if not same_turn:
        baton.spawn(end, name="end")
        baton.run_until_quiescent()
    return result


def test_the_drain_flush_puts_every_line_on_disk(participant, tmp_path):
    result = _end(participant, _strict)
    assert result["flushed"] is True
    assert result["before"] == ""
    assert "INFO before the end" in result["after"]
    (csv,) = (tmp_path / "pose").glob(f"pose_cadence_observation_uav_{SYS_ID}_*.csv")
    assert csv.read_text(encoding="utf-8") == "wall_s,obs_ts,src_now_s,outcome\n0.0,1.0,2.0,fresh\n"


def test_a_failed_flush_reports_not_flushed_and_the_rest_still_run(participant, tmp_path):
    # A directory where the CSV must go makes the strict pose write fail.
    (tmp_path / "pose").mkdir()
    (tmp_path / "pose" / f"pose_cadence_observation_uav_{SYS_ID}_pid_{os.getpid()}.csv").mkdir()
    result = _end(participant, lambda logger: list(reversed(_strict(logger))))
    assert result["flushed"] is False
    assert "INFO before the end" in result["after"]


def test_a_logger_closed_before_its_worker_ever_ran_keeps_every_line(participant):
    """Built, logged and closed in one baton turn: the cancelled worker's FIFO is drained."""
    result = _end(participant, _strict, same_turn=True)
    assert result["flushed"] is True
    assert result["before"] == ""
    lines = result["after"].splitlines()
    assert [line.split(" ", 3)[3] for line in lines] == ["before the end", "Logger closed"]


def test_a_line_the_file_sink_failed_to_write_reports_not_flushed(participant):
    """The real file handler swallows the write error; its latch still fails the drain."""
    result = _end(participant, _strict, before_end=_break_the_file_sink)
    assert result["flushed"] is False
    assert result["after"] == ""


def test_the_drain_flush_runs_only_on_the_baton(participant):
    with pytest.raises(LockstepFault, match="off the baton"):
        participant.drain_flush([])
