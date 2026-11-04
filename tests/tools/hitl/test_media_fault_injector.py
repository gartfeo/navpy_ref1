"""The commissioning injector (tools/hitl/media_fault_injector.py, media_fault_spec.py) and navpy's capture wrapper
dependency (app/perception/frame_capture_factory.py): delay, loss, aged frames, identity kept, profile refusal and
admission refusal. Synthetic frames only."""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from navpy.app.perception import frame_capture_factory as factory
from navpy.app.perception.integration_timing_input import open_integration_input
from navpy.core.perception.camera.frame_capture_runtime import FrameCaptureRuntime
from navpy.core.perception.camera.frame_publication import FramePublicationStore
from navpy.core.ports.frame_capture_ports import CaptureStamp, CaptureStampKind, FrameSourceIdentity
from tests.tools.hitl.test_approach_records import make, media_runtime, model_dir  # noqa: F401 - fixtures
from tools.hitl import approach_records as ar
from tools.hitl import check_integration_timing as cit
from tools.hitl import media_fault_injector as mfi
from tools.hitl import media_fault_spec as spec
from tools.hitl.json_checks import RecordError
from tools.hitl.timing_inputs import TimingInputError
from tools.tuning import approach_chain as chain

PERIOD_S = 1.0 / 30.0
SOURCE = "gstreamer+rtsp://127.0.0.1:8554/zr10?decoder=avdec_h265&latency_ms=0"


class _Logger:
    def __init__(self) -> None:
        self.lines: list[str] = []

    def info(self, message: str) -> None:
        self.lines.append(message)

    warning = error = info


class _Clock:
    def __init__(self) -> None:
        self.now = 100.0

    def __call__(self) -> float:
        return self.now


class _Backend:
    """Publishes synthetic decoded frames one frame period apart; the clock reads each frame's arrival."""

    def __init__(self, count: int, clock: _Clock, gaps: dict[int, float] | None = None, identity: bool = True) -> None:
        self.stamps = []
        self.frames = []
        self.closed = False
        self._clock, self._gaps, self._identity = clock, gaps or {}, identity
        self._count = count

    def run(self, publish, is_running) -> None:
        for index in range(self._count):
            self._clock.now += PERIOD_S + self._gaps.get(index, 0.0)
            source = FrameSourceIdentity("open-1", index, 1_000_000 + index * 33_333_333) if self._identity else None
            stamp = CaptureStamp(self._clock.now, CaptureStampKind.READ, source)
            frame = np.full((2, 2), index, dtype=np.uint8)
            self.stamps.append(stamp)
            self.frames.append(frame)
            publish(frame, stamp)

    def request_stop(self) -> bool:
        return True

    def close(self) -> None:
        self.closed = True


def _inject(fault: str, backend: _Backend, clock: _Clock, tmp_path: Path):
    wrapper = mfi.MediaFaultWrapper(spec.parse(fault), tmp_path / "fault.jsonl", clock)
    wrapped = wrapper(backend, _Logger())
    published: list[tuple[np.ndarray, CaptureStamp]] = []
    wrapped.run(lambda frame, stamp: published.append((frame, stamp)), lambda: True)
    wrapped.close()
    lines = [json.loads(line) for line in (tmp_path / "fault.jsonl").read_text(encoding="utf-8").splitlines()]
    return wrapped, published, lines


# ---- delay ----------------------------------------------------------------------------------------------------------
def test_one_frame_delay_publishes_each_frame_one_frame_late_with_its_own_stamp(tmp_path):
    clock = _Clock()
    backend = _Backend(10, clock)
    wrapped, published, lines = _inject("delay:1@3", backend, clock, tmp_path)
    # Frames 0-2 pass, then frame k is published when frame k+1 arrives; the last one is still held at the end.
    assert [stamp.source.sequence for _, stamp in published] == [0, 1, 2, 3, 4, 5, 6, 7, 8]
    for frame, stamp in published:
        index = stamp.source.sequence
        assert stamp is backend.stamps[index]            # 0 retimestamped: the very stamp the backend made
        assert frame is backend.frames[index]
    frames = [line for line in lines if line["event"] == "frame"]
    assert [line["action"] for line in frames] == ["pass"] * 3 + ["hold"] * 7
    for line in frames[4:]:                              # released in the place of the next decoded frame
        assert line["released"]["sequence"] == line["sequence"] - 1
        assert line["released"]["read_s"] == backend.stamps[line["sequence"] - 1].captured_at_s
        assert line["released"]["age_s"] == pytest.approx(PERIOD_S)
    assert "released" not in frames[3]                   # the first held frame fills the 1-frame slot
    assert lines[-1] == {"event": "close", "decoded": 10, "pass": 3, "hold": 7, "drop": 0, "released": 6,
                         "discarded_aged": 0, "left_held": 1}
    assert backend.closed and wrapped.counts["released"] == 6


def test_delay_holds_a_bounded_number_of_frames(tmp_path):
    clock = _Clock()
    backend = _Backend(20, clock)
    wrapped, published, _ = _inject(f"delay:{spec.MAX_DELAY_FRAMES}", backend, clock, tmp_path)
    assert [stamp.source.sequence for _, stamp in published] == list(range(20 - spec.MAX_DELAY_FRAMES))
    assert wrapped._held.maxlen == spec.MAX_DELAY_FRAMES + 1


def test_aged_frames_are_discarded_never_published_late(tmp_path):
    clock = _Clock()
    backend = _Backend(8, clock, gaps={5: 0.6})          # the stream pauses 0.6 s before frame 5
    wrapped, published, lines = _inject("delay:1", backend, clock, tmp_path)
    # Frame 4 was held across the pause: 0.63 s old on its own stamp when frame 5 came, so it is discarded.
    assert [stamp.source.sequence for _, stamp in published] == [0, 1, 2, 3, 5, 6]
    discarded = [line for line in lines if line["event"] == "frame" and "discarded_aged" in line]
    assert [line["discarded_aged"]["sequence"] for line in discarded] == [4]
    assert discarded[0]["discarded_aged"]["age_s"] > spec.MAX_HOLD_S
    assert all(stamp is backend.stamps[stamp.source.sequence] for _, stamp in published)
    assert wrapped.counts["discarded_aged"] == 1


def test_a_released_frame_is_published_on_its_age_check_before_any_slow_log_write(tmp_path):
    clock = _Clock()
    backend = _Backend(8, clock)
    wrapped = mfi.MediaFaultWrapper(spec.parse("delay:1"), tmp_path / "fault.jsonl", clock)(backend, _Logger())
    write = wrapped._log.write

    def slow_write(record):                              # each log line stalls the capture thread 0.3 s
        write(record)
        clock.now += 0.3
    wrapped._log.write = slow_write
    ages = []
    wrapped.run(lambda frame, stamp: ages.append(clock.now - stamp.captured_at_s), lambda: True)
    wrapped.close()
    # Each held frame is 0.33 s old when the next arrives (under the bound): it goes out at that age, not after the
    # 0.3 s write that follows.
    assert len(ages) == 7 and max(ages) == pytest.approx(0.3 + PERIOD_S)


def test_a_failed_close_summary_still_frees_the_held_frames_and_the_log(tmp_path):
    clock = _Clock()
    backend = _Backend(4, clock)
    wrapped = mfi.MediaFaultWrapper(spec.parse("delay:2"), tmp_path / "fault.jsonl", clock)(backend, _Logger())
    wrapped.run(lambda frame, stamp: None, lambda: True)

    def full_disk(record):
        raise OSError("no space left on device")
    wrapped._log.write = full_disk
    with pytest.raises(OSError):
        wrapped.close()
    assert backend.closed and not wrapped._held and wrapped._log._file.closed


# ---- loss -----------------------------------------------------------------------------------------------------------
def test_total_loss_publishes_nothing_after_its_start(tmp_path):
    clock = _Clock()
    backend = _Backend(12, clock)
    wrapped, published, lines = _inject("loss@4", backend, clock, tmp_path)
    assert [stamp.source.sequence for _, stamp in published] == [0, 1, 2, 3]
    assert all(stamp is backend.stamps[stamp.source.sequence] for _, stamp in published)
    assert [line["action"] for line in lines if line["event"] == "frame"] == ["pass"] * 4 + ["drop"] * 8
    assert lines[-1]["drop"] == 8 and lines[-1]["left_held"] == 0


def test_total_loss_from_the_first_frame(tmp_path):
    clock = _Clock()
    _, published, _ = _inject("loss", _Backend(5, clock), clock, tmp_path)
    assert published == []


def test_a_frame_without_identity_fails_the_capture(tmp_path):
    clock = _Clock()
    wrapper = mfi.MediaFaultWrapper(spec.parse("delay:1"), tmp_path / "fault.jsonl", clock)
    wrapped = wrapper(_Backend(3, clock, identity=False), _Logger())
    with pytest.raises(ValueError, match="source identity"):
        wrapped.run(lambda *_: None, lambda: True)
    wrapped.close()


def test_one_open_per_run_and_one_log_per_run(tmp_path):
    clock = _Clock()
    wrapper = mfi.MediaFaultWrapper(spec.parse("loss"), tmp_path / "fault.jsonl", clock)
    wrapper(_Backend(1, clock), _Logger())
    with pytest.raises(RuntimeError, match="one capture open"):
        wrapper(_Backend(1, clock), _Logger())
    with pytest.raises(FileExistsError):
        mfi.MediaFaultWrapper(spec.parse("loss"), tmp_path / "fault.jsonl", clock)(_Backend(1, clock), _Logger())


# ---- the fault profile ----------------------------------------------------------------------------------------------
@pytest.mark.parametrize("text, expected", [
    ("none", spec.NO_FAULT),
    ("delay:1", spec.FaultSpec(spec.DELAY, 1, 0)),
    ("delay:3@300", spec.FaultSpec(spec.DELAY, 3, 300)),
    ("loss", spec.FaultSpec(spec.LOSS, 0, 0)),
    ("loss@900", spec.FaultSpec(spec.LOSS, 0, 900)),
])
def test_fault_spellings(text, expected):
    assert spec.parse(text) == expected


@pytest.mark.parametrize("text", ["", "delay", "delay:0", "delay:9", "delay:-1", "drop", "loss:1", "none@3",
                                  "delay:1@", "delay:1@-2", "Loss"])
def test_bad_fault_spellings_are_refused(text):
    with pytest.raises(ValueError):
        spec.parse(text)


@pytest.mark.parametrize("fault, reason", [
    (spec.FaultSpec("stall"), "not one of"),
    (spec.FaultSpec(spec.DELAY, 1, 0, 0.6), "over the bound"),
    (spec.FaultSpec(spec.DELAY, 1, 0, 0.0), "positive finite"),
    (spec.FaultSpec(spec.DELAY, 1, 0, float("nan")), "positive finite"),
    (spec.FaultSpec(spec.LOSS, 2, 0), "no delay_frames"),
    (spec.FaultSpec(spec.DELAY, True, 0), "whole number"),
])
def test_malformed_fault_profiles_are_refused(fault, reason):
    assert any(reason in problem for problem in fault.problems())
    with pytest.raises(ValueError, match="no injectable fault"):
        mfi.MediaFaultWrapper(fault, Path("unused.jsonl"))


def test_no_fault_cannot_be_injected():
    with pytest.raises(ValueError, match="no injectable fault"):
        mfi.MediaFaultWrapper(spec.NO_FAULT, Path("unused.jsonl"))


@pytest.mark.parametrize("mode", ["flight", "qualification", "pixhawk-hil", "jetson-sitl"])
def test_only_sitl_commissioning_modes_may_inject(mode):
    assert spec.mode_problems(spec.parse("delay:1"), mode)
    assert spec.mode_problems(spec.NO_FAULT, mode) == []
    assert spec.mode_problems(spec.parse("loss"), "pc-sitl") == []


def test_the_record_holds_the_fault_and_refuses_a_malformed_one(model_dir):  # noqa: F811
    assert make(model_dir)["commissioning"] == spec.NO_FAULT.record()
    record = make(model_dir, commissioning=spec.parse("delay:1@300"))
    assert record["commissioning"] == {"fault": "delay", "delay_frames": 1, "after_frames": 300, "max_hold_s": 0.5}
    bad = json.loads(json.dumps(record))
    bad["commissioning"]["delay_frames"] = 0
    with pytest.raises(RecordError, match="record.commissioning"):
        ar.validate_record(bad)
    del bad["commissioning"]
    with pytest.raises(RecordError):
        ar.validate_record(bad)


# ---- admission ------------------------------------------------------------------------------------------------------
def test_a_commissioning_run_is_never_admitted(model_dir, tmp_path):  # noqa: F811
    assert spec.admission_problems(make(model_dir)) == []
    record = make(model_dir, commissioning=spec.parse("loss@300"))
    assert "never admitted to a ten-run series" in spec.admission_problems(record)[0]
    path = tmp_path / "record.json"
    path.write_text(json.dumps(record), encoding="utf-8")
    args = argparse.Namespace(run_record=str(path), navpy="absent", publisher=[], unreal="absent", emulator="absent")
    with pytest.raises(TimingInputError, match="commissioning run"):     # no timing fit or check reads it
        cit.load(args)


def _navpy_args(**changes):
    values = dict(detector_type="real", integration_timing="", integration_log="log.jsonl")
    values.update(changes)
    return SimpleNamespace(**values)


def test_the_launcher_refuses_without_a_commissioning_record(model_dir, tmp_path):  # noqa: F811
    _, problems = mfi.launch_problems(_navpy_args())
    assert "needs --integration-timing" in problems[0]
    plain = tmp_path / "plain.json"
    plain.write_text(json.dumps(make(model_dir)), encoding="utf-8")
    _, problems = mfi.launch_problems(_navpy_args(integration_timing=str(plain)))
    assert any("names no commissioning fault" in problem for problem in problems)
    faulty = tmp_path / "faulty.json"
    faulty.write_text(json.dumps(make(model_dir, commissioning=spec.parse("delay:1"))), encoding="utf-8")
    _, problems = mfi.launch_problems(_navpy_args(integration_timing=str(faulty), detector_type="sim"))
    assert any("real detector" in problem for problem in problems)
    fault, problems = mfi.launch_problems(_navpy_args(integration_timing=str(faulty)))
    assert problems == [] and fault == spec.parse("delay:1")
    broken = tmp_path / "broken.json"
    broken.write_text("{}", encoding="utf-8")
    _, problems = mfi.launch_problems(_navpy_args(integration_timing=str(broken)))
    assert any("run record refused" in problem for problem in problems)


def test_the_launcher_exits_before_navpy_on_a_refusal(tmp_path, monkeypatch):
    started = []
    monkeypatch.setattr("navpy.main.main", lambda args: started.append(args) or 0)
    code = mfi.main(["--fault-log", str(tmp_path / "f.jsonl"), "--", "-ss", "1", "--detector-type", "real"])
    assert code == 2 and started == [] and not (tmp_path / "f.jsonl").exists()


def test_the_runner_starts_navpy_through_the_injector_only_when_commissioning(tmp_path):
    paths = SimpleNamespace(python="py", run_dir=tmp_path, repo=tmp_path, windows_ip="10.0.0.1")
    plain = chain.navpy(paths, tmp_path / "r.json", 2).argv
    faulty = chain.navpy(paths, tmp_path / "r.json", 2, commissioning=True).argv
    assert "navpy.main" in plain and "tools.hitl.media_fault_injector" not in plain
    start = faulty.index("tools.hitl.media_fault_injector")
    assert faulty[start + 1:start + 4] == ("--fault-log", str(tmp_path / chain.FAULT_LOG), "--")
    assert "navpy.main" not in faulty
    assert faulty[start + 4:] == plain[plain.index("navpy.main") + 1:]


# ---- navpy's factory dependency -------------------------------------------------------------------------------------
def test_the_factory_wraps_only_the_decoded_encoded_stream(monkeypatch):
    opened = _Backend(0, _Clock())
    monkeypatch.setattr(factory.GstreamerFrameCapture, "open", staticmethod(lambda source, logger: opened))
    logger = _Logger()
    wrapped = factory.open_capture_backend(SOURCE, logger, wrap=lambda backend, _log: ("wrapped", backend))
    assert wrapped == ("wrapped", opened)
    for source in (0, "rtsp://camera/stream", "npyc://127.0.0.1:18110"):
        with pytest.raises(ValueError, match="only the decoded encoded stream"):
            factory.open_capture_backend(source, logger, wrap=lambda backend, _log: backend)


def test_the_factory_closes_the_backend_when_wrapping_fails(monkeypatch):
    opened = _Backend(0, _Clock())
    monkeypatch.setattr(factory.GstreamerFrameCapture, "open", staticmethod(lambda source, logger: opened))

    def refuse(_backend, _log):
        raise RuntimeError("refused")

    with pytest.raises(RuntimeError, match="refused"):
        factory.open_capture_backend(SOURCE, _Logger(), wrap=refuse)
    assert opened.closed
    monkeypatch.setattr(factory.GstreamerFrameCapture, "open", staticmethod(lambda source, logger: None))
    assert factory.open_capture_backend(SOURCE, _Logger(), wrap=refuse) is None


def test_navpy_admits_a_wrapper_only_with_integration_timing():
    with pytest.raises(ValueError, match="capture wrapper needs"):
        open_integration_input("", "", "real", capture_wrapper=lambda backend, _log: backend)
    with pytest.raises(ValueError, match="push mode"):
        FrameCaptureRuntime(None, _Logger(), FramePublicationStore(), capture_wrapper=lambda backend, _log: backend)


def test_published_frames_keep_the_backend_stamp_through_navpy(monkeypatch, tmp_path):
    """Through navpy's own capture runtime and publication store: every stamp the store saw is the backend's."""
    clock = _Clock()
    backend = _Backend(6, clock)
    monkeypatch.setattr(factory.GstreamerFrameCapture, "open", staticmethod(lambda source, logger: backend))
    seen: list[CaptureStamp | None] = []
    store = FramePublicationStore(observer=seen.append)
    wrapper = mfi.MediaFaultWrapper(spec.parse("delay:1"), tmp_path / "fault.jsonl", clock)
    runtime = FrameCaptureRuntime(SOURCE, _Logger(), store, capture_wrapper=wrapper)
    runtime.start()
    deadline = time.monotonic() + 5.0
    while runtime.has_capture_thread and time.monotonic() < deadline:     # the fake backend ends by itself
        time.sleep(0.01)
    assert runtime.stop()
    assert [stamp.source.sequence for stamp in seen] == [0, 1, 2, 3, 4]
    assert all(stamp is backend.stamps[stamp.source.sequence] for stamp in seen)
