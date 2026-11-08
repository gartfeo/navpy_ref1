"""Step 13's path from the HITL runner into navpy: the run record's integration timing, the JSONL log and its
log_end, and when vision closes it (app/perception/integration_timing_input.py)."""
from __future__ import annotations

import argparse
import json
from dataclasses import asdict
from types import SimpleNamespace

import pytest

from navpy.app.perception.integration_timing_input import (
    JsonlTimingLog,
    load_integration_timing,
    open_integration_input,
)
from navpy.app.perception.real_detector_factory import RealDetectorFactory
from navpy.app.perception.vision_controller import VisionController
from navpy.app.vision_args import VisionArgs
from navpy.core.perception.camera.integration_timing import IntegrationTimingLimits
from tools.hitl.timing_inputs import TimingInputError, read_navpy_log


def _record(mode: str = "pc-sitl", **section) -> dict:
    timing = {"authority": "integration_uncertified", "source_id": "gstreamer+rtsp://x", "link_id": "udpin:x#1",
              "attitude_correction_s": 0.01, "gimbal_correction_s": -0.02,
              "limits": asdict(IntegrationTimingLimits()), **section}
    return {"run": {"run_id": "run-1", "mode": mode}, "integration_timing": timing}


def _write(tmp_path, record) -> str:
    path = tmp_path / "record.json"
    path.write_text(json.dumps(record), encoding="utf-8")
    return str(path)


def test_the_record_gives_the_run_config(tmp_path):
    config = load_integration_timing(_write(tmp_path, _record()))
    assert (config.run_id, config.mode, config.attitude_correction_s, config.gimbal_correction_s) == \
        ("run-1", "pc-sitl", 0.01, -0.02)
    assert config.limits == IntegrationTimingLimits()


@pytest.mark.parametrize("record", [_record(mode="flight"), _record(gimbal_correction_s=float("nan")),
                                    {"run": {"run_id": "run-1", "mode": "pc-sitl"}}])
def test_a_record_navpy_may_not_use_is_refused(tmp_path, record):
    with pytest.raises(ValueError):
        load_integration_timing(_write(tmp_path, record))


def test_the_log_counts_its_records_and_ends_once(tmp_path):
    path = tmp_path / "navpy.jsonl"
    log = JsonlTimingLog(path, "run-1")
    log({"event": "attitude_receipt", "run_id": "run-1", "time_boot_s": 1.0, "capture_receipt_s": 2.0})
    log({"event": "gimbal_receipt", "run_id": "run-1", "session": 1, "seq": 3, "receipt_s": 2.5})
    log.close()
    log.close()
    lines = path.read_text(encoding="utf-8").splitlines()
    assert json.loads(lines[-1]) == {"event": "log_end", "run_id": "run-1", "records": 2}
    assert len(lines) == 3 and read_navpy_log(lines).ended
    with pytest.raises(FileExistsError):
        JsonlTimingLog(path, "run-1")


def test_a_log_cut_at_a_line_boundary_has_no_end_and_a_wrong_end_is_refused():
    receipt = '{"event": "gimbal_receipt", "run_id": "r", "session": 1, "seq": 1, "receipt_s": 1.0}'
    assert not read_navpy_log([receipt]).ended
    with pytest.raises(TimingInputError, match="counts"):
        read_navpy_log([receipt, '{"event": "log_end", "run_id": "r", "records": 2}'])
    with pytest.raises(TimingInputError, match="after log_end"):
        read_navpy_log(['{"event": "log_end", "run_id": "r", "records": 0}', receipt])


def test_both_flags_or_neither_and_only_with_the_real_detector(tmp_path):
    record = _write(tmp_path, _record())
    assert open_integration_input("", "", "real") is None
    with pytest.raises(ValueError, match="together"):
        open_integration_input(record, "", "real")
    with pytest.raises(ValueError, match="real"):
        open_integration_input(record, str(tmp_path / "log.jsonl"), "sim")
    assert not (tmp_path / "log.jsonl").exists()
    opened = open_integration_input(record, str(tmp_path / "log.jsonl"), "real")
    assert opened.config.run_id == "run-1"
    opened.log.close()


def test_the_flags_reach_vision_args():
    parser = argparse.ArgumentParser()
    VisionArgs.add_args(parser)
    args = VisionArgs.from_args(parser.parse_args(["--integration-timing", "r.json", "--integration-log", "l"]))
    assert (args.integration_timing, args.integration_log) == ("r.json", "l")


def test_one_camera_mount_per_run(tmp_path):
    opened = open_integration_input(_write(tmp_path, _record()), str(tmp_path / "log.jsonl"), "real")
    factory = RealDetectorFactory(None, None, {}, "", False, backend="charuco", integration=opened)
    assert factory._take_integration(0) is opened
    # A restarted detector of the same mount takes it again (recovery step 5b); another mount may not.
    assert factory._take_integration(0) is opened
    with pytest.raises(ValueError, match="one camera mount"):
        factory._take_integration(1)
    opened.log.close()


class _Log:
    closed = 0

    def close(self):
        self.closed += 1


@pytest.mark.parametrize("quiescent", [True, False])
def test_vision_ends_the_log_only_after_a_quiescent_stop(quiescent):
    log = _Log()
    controller = VisionController.__new__(VisionController)
    controller._ports = SimpleNamespace(lifecycle=SimpleNamespace(stop=lambda: quiescent), integration_log=log,
                                        dock_contact=None)
    assert controller.stop() is quiescent
    assert log.closed == (1 if quiescent else 0)
