"""Encoded RTSP capture: intervals kept, repeats dropped, a lost stream ends the run.

A scripted GStreamer stands in on any host; the tests at the end use the real
one where it exists (WSL, the Orin).
"""

from __future__ import annotations

import importlib.util
import itertools
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import pytest

from navpy.app.perception.frame_capture_factory import open_capture_backend
from navpy.core.perception.camera.frame_capture_runtime import FrameCaptureRuntime
from navpy.core.perception.camera.frame_publication import FramePublicationStore, FrameSnapshot
from navpy.core.perception.pipeline.real_detector_runtime_state import LoopTiming, RuntimeMetrics
from navpy.core.perception.pipeline.real_inference import DetectionLoop
from navpy.core.ports.frame_capture_ports import CaptureStampKind
from navpy.core.perception.profile import vision_profile_loader
from navpy.plugins.camera.gstreamer import frame_capture_gstreamer as module
from navpy.plugins.camera.gstreamer.encoded_frame_gate import EncodedStreamReset
from navpy.plugins.camera.gstreamer.encoded_source import parse_gstreamer_source
from navpy.plugins.camera.gstreamer.frame_capture_gstreamer import GstreamerFrameCapture
from tools.hitl.encoded_capture_score import score_intervals, score_los

SOURCE = "gstreamer+rtsp://127.0.0.1:18554/main.264?decoder=avdec_h265&latency_ms=50"
WIDTH, HEIGHT = 4, 2
NO_PTS = 2**64 - 1


class _Logger:
    def __init__(self):
        self.lines = []

    def info(self, message):
        self.lines.append(message)

    warning = error = info


class _Buffer:
    def __init__(self, pts, corrupted=False, size=WIDTH * 3 * HEIGHT):
        self.pts = pts
        self._corrupted = corrupted
        self._size = size

    def has_flags(self, _flags):
        return self._corrupted

    def get_size(self):
        return self._size

    def extract_dup(self, _offset, size):
        return bytes(range(size))


def _sample(pts, width=WIDTH, height=HEIGHT, corrupted=False):
    structure = SimpleNamespace(
        get_int=lambda name: (True, width if name == "width" else height),
    )
    caps = SimpleNamespace(get_structure=lambda _i: structure)
    return SimpleNamespace(
        get_buffer=lambda: _Buffer(pts, corrupted, width * 3 * height),
        get_caps=lambda: caps,
    )


class _Message:
    def __init__(self, type_, text="gone"):
        self.type = type_
        self._text = text

    def parse_error(self):
        return SimpleNamespace(message=self._text), "debug"

    parse_warning = parse_error


class _Pipeline:
    """One connection's script: samples, then bus messages once they run out."""

    def __init__(self, samples=(), messages=(), play=None, settings=None):
        self.samples = list(samples)
        self.messages = list(messages)
        self.play = play
        self.states = []
        self.values = {
            "buffer-mode": SimpleNamespace(value_nick="none"),
            "ntp-sync": False,
            "rfc7273-sync": False,
            "latency": 50,
            "sync": False,
            **(settings or {}),
        }

    def get_by_name(self, name):
        if name == "sink":
            return SimpleNamespace(
                emit=lambda _signal, _ns: self.samples.pop(0) if self.samples else None,
                get_property=self.values.__getitem__,
            )
        return SimpleNamespace(get_property=self.values.__getitem__)

    def get_bus(self):
        return SimpleNamespace(
            pop_filtered=lambda _types: self.messages.pop(0) if self.messages else None,
        )

    def set_state(self, state):
        self.states.append(state)
        return self.play if state == "playing" and self.play else "ok"


class _Gst:
    CLOCK_TIME_NONE = NO_PTS
    State = SimpleNamespace(PLAYING="playing", NULL="null")
    StateChangeReturn = SimpleNamespace(FAILURE="failure")
    MessageType = SimpleNamespace(ERROR=1, EOS=2, WARNING=4)
    BufferFlags = SimpleNamespace(CORRUPTED=8)

    def __init__(self, *connections, decoder=True):
        self.connections = list(connections)
        self.launched = []
        self.ElementFactory = SimpleNamespace(make=lambda *_: object() if decoder else None)

    def parse_launch(self, text):
        self.launched.append(text)
        return self.connections.pop(0)


def _open(gst, logger=None, observer=None):
    capture = GstreamerFrameCapture.open(SOURCE, logger or _Logger(), observer, lambda: gst)
    assert capture is not None
    return capture


def _run(capture, frames_then_stop=None):
    published = []

    def publish(frame, stamp):
        published.append((frame, stamp))

    def running():
        return frames_then_stop is None or len(published) < frames_then_stop

    capture.run(publish, running)
    return published


@pytest.fixture
def short_limits(monkeypatch):
    monkeypatch.setattr(module, "SAMPLE_GAP_LIMIT_S", 0.05)
    monkeypatch.setattr(module, "CONNECT_RETRY_S", 0.0)


def test_source_states_every_setting_or_is_refused():
    settings = parse_gstreamer_source(SOURCE)
    assert (settings.url, settings.decoder, settings.latency_ms) == (
        "rtsp://127.0.0.1:18554/main.264", "avdec_h265", 50)
    for bad in (
        "rtsp://127.0.0.1/x",
        "gstreamer+rtsp://127.0.0.1/x",
        "gstreamer+rtsp://127.0.0.1/x?decoder=avdec_h265",
        "gstreamer+rtsp://127.0.0.1/x?decoder=fakedec&latency_ms=50",
        "gstreamer+rtsp://127.0.0.1/x?decoder=avdec_h265&latency_ms=-1",
        "gstreamer+rtsp://127.0.0.1/x?decoder=avdec_h265&latency_ms=50&extra=1",
        "gstreamer+rtsp://127.0.0.1/x!fakesink?decoder=avdec_h265&latency_ms=50",
        "gstreamer+rtsp:///x?decoder=avdec_h265&latency_ms=50",
    ):
        with pytest.raises(ValueError):
            parse_gstreamer_source(bad)


def test_pipeline_pins_what_keeps_sender_intervals():
    text = parse_gstreamer_source(SOURCE).pipeline_text()
    for pin in (
        "protocols=udp", "buffer-mode=none", "ntp-sync=false", "rfc7273-sync=false",
        "latency=50", "sync=false", "drop=false",
    ):
        assert pin in text
    assert "videorate" not in text


def test_factory_has_no_fallback_for_this_source():
    with (
        patch("navpy.app.perception.frame_capture_factory.FfmpegFrameCapture") as ffmpeg,
        patch("navpy.app.perception.frame_capture_factory.open_source_capture") as opencv,
        patch("navpy.app.perception.frame_capture_factory.open_jetson_capture") as jetson,
        patch.object(GstreamerFrameCapture, "open", return_value=None),
    ):
        assert open_capture_backend(SOURCE, _Logger()) is None
    ffmpeg.open.assert_not_called()
    opencv.assert_not_called()
    jetson.assert_not_called()


def test_plain_rtsp_keeps_its_old_path():
    with patch("navpy.app.perception.frame_capture_factory.FfmpegFrameCapture") as ffmpeg, \
            patch.object(GstreamerFrameCapture, "open") as gstreamer:
        ffmpeg.open.return_value = "ffmpeg"
        assert open_capture_backend("rtsp://camera/stream", _Logger()) == "ffmpeg"
    gstreamer.assert_not_called()


def test_missing_bindings_or_decoder_refuse_the_open():
    def no_gi():
        raise ImportError("no gi")

    assert GstreamerFrameCapture.open(SOURCE, _Logger(), None, no_gi) is None
    assert GstreamerFrameCapture.open(SOURCE, _Logger(), None, lambda: _Gst(decoder=False)) is None


def test_hitl_profile_names_the_encoded_source_explicitly():
    _key, profile, _path = vision_profile_loader.resolve_profile("siyi_zr10_hitl_encoded", _Logger())
    source = profile["detector"]["camera_index"]
    settings = parse_gstreamer_source(source)
    assert settings.decoder == "nvh265dec"
    assert profile["detector"]["tracker"]["backend"] == "custom"
    assert "with_reid" in profile["detector"]["tracker"]    # the rest comes from siyi_zr10


def test_sender_intervals_reach_the_consumer_unchanged():
    # A gap and an uneven interval: nothing downstream may smooth or retime them.
    pts = [1_000, 34_000_000, 67_000_000, 200_000_000, 233_111_111]
    pipeline = _Pipeline([_sample(p) for p in pts])
    observed = []
    capture = _open(_Gst(pipeline), observer=lambda identity, at: observed.append(identity))

    published = _run(capture, frames_then_stop=len(pts))

    stamps = [stamp for _frame, stamp in published]
    assert [s.source.pts_ns for s in stamps] == pts
    assert [s.source.sequence for s in stamps] == [1, 2, 3, 4, 5]
    assert {s.source.open_id for s in stamps} == {capture.open_id}
    assert all(s.kind is CaptureStampKind.READ for s in stamps)
    assert [s.source for s in stamps] == observed
    assert published[0][0].shape == (HEIGHT, WIDTH, 3)
    assert published[0][0].flags.writeable


def test_each_open_has_its_own_identity():
    assert _open(_Gst()).open_id != _open(_Gst()).open_id


def test_a_repeated_pts_is_never_published_twice(short_limits):
    pipeline = _Pipeline([_sample(10), _sample(10), _sample(10), _sample(20)])
    capture = _open(_Gst(pipeline))

    published = _run(capture, frames_then_stop=2)

    assert [s.source.pts_ns for _f, s in published] == [10, 20]
    assert [s.source.sequence for _f, s in published] == [1, 2]
    assert capture.gate.duplicates == 2


def test_repeats_alone_do_not_keep_the_stream_alive(short_limits):
    pipeline = _Pipeline([_sample(10)])

    class _Forever(list):
        def __bool__(self):
            return True

        def pop(self, _index):
            return _sample(10)

    pipeline.samples = _Forever()
    with pytest.raises(EncodedStreamReset, match="no frame for"):
        _run(_open(_Gst(pipeline)))


def test_a_concealed_picture_is_dropped_and_counted():
    pipeline = _Pipeline([_sample(10, corrupted=True), _sample(20)])
    capture = _open(_Gst(pipeline))

    published = _run(capture, frames_then_stop=1)

    assert [s.source.pts_ns for _f, s in published] == [20]
    assert capture.gate.corrupted == 1


@pytest.mark.parametrize("second, reason", [
    (_sample(5), "backward"),
    (_sample(NO_PTS), "without a PTS"),
    (_sample(20, width=8), "size changed"),
])
def test_a_reset_ends_the_open(second, reason):
    gst = _Gst(_Pipeline([_sample(10), second]))
    with pytest.raises(EncodedStreamReset, match=reason):
        _run(_open(gst))
    assert len(gst.launched) == 1


@pytest.mark.parametrize("message", [_Message(2), _Message(1), None])
def test_a_disconnect_ends_the_open_and_never_reconnects(short_limits, message):
    pipeline = _Pipeline([_sample(10)], [message] if message else [])
    gst = _Gst(pipeline)     # a second connection would raise IndexError
    with pytest.raises(EncodedStreamReset):
        _run(_open(gst))
    assert len(gst.launched) == 1
    assert pipeline.states[-1] == "null"


def test_a_lost_stream_fails_the_run_and_a_new_run_gets_a_new_open(short_limits):
    gst = _Gst(_Pipeline([_sample(10)], [_Message(2)]))
    with patch(
        "navpy.core.perception.camera.frame_capture_runtime.open_capture_backend",
        side_effect=lambda *_: _open(gst),
    ) as opened:
        runtime = FrameCaptureRuntime(SOURCE, _Logger(), FramePublicationStore())
        runtime.start()
        for _ in range(500):
            if not runtime.has_capture_thread:
                break
            module.rt.sleep(0.01)
        with pytest.raises(EncodedStreamReset):
            runtime.raise_if_failed()
        assert runtime.is_running is False
        assert opened.call_count == 1      # nothing reopened it


def test_before_the_first_frame_a_refused_connection_is_tried_again(short_limits):
    refused = _Pipeline(messages=[_Message(1, "404")])
    no_play = _Pipeline(play="failure")
    serving = _Pipeline([_sample(10)])
    gst = _Gst(refused, no_play, serving)

    published = _run(_open(gst), frames_then_stop=1)

    assert len(published) == 1
    assert len(gst.launched) == 3
    assert refused.states[-1] == no_play.states[-1] == "null"


def test_no_stream_at_all_ends_at_the_deadline(short_limits, monkeypatch):
    monkeypatch.setattr(module, "FIRST_FRAME_DEADLINE_S", 0.0)
    with pytest.raises(EncodedStreamReset, match="no frame within"):
        _run(_open(_Gst(_Pipeline(messages=[_Message(1)]))))


def test_settings_that_did_not_take_refuse_the_run():
    gst = _Gst(_Pipeline(settings={"latency": 2000}))
    with pytest.raises(EncodedStreamReset, match="did not take"):
        _run(_open(gst))


def test_the_settings_are_recorded_as_read_back():
    logger = _Logger()
    capture = _open(_Gst(_Pipeline([_sample(10)])), logger)
    _run(capture, frames_then_stop=1)
    assert capture.recorded["buffer_mode"] == "none"
    assert capture.recorded["latency_ms"] == 50
    assert capture.recorded["ntp_sync"] is False
    assert capture.recorded["rfc7273_sync"] is False
    assert capture.recorded["appsink_sync"] is False
    assert any("encoded capture settings" in line for line in logger.lines)


def test_stop_ends_reading_and_close_is_safe_twice():
    pipeline = _Pipeline()
    capture = _open(_Gst(pipeline))
    capture.request_stop()
    assert _run(capture) == []
    capture.close()
    capture.close()


def test_a_padded_frame_loses_its_padding():
    data = bytes(range(16)) * 2     # 2 rows, 16 bytes each, 4 pixels of 12 bytes
    frame = module._bgr_pixels(data, 4, 2)
    assert frame.shape == (2, 4, 3)
    assert frame[1, 3, 2] == 11
    with pytest.raises(EncodedStreamReset):
        module._bgr_pixels(b"123", 4, 2)


def test_the_detector_measures_each_picture_once():
    frame = np.zeros((2, 2, 3), dtype=np.uint8)
    snapshots = [
        FrameSnapshot(frame, 2, 2, 7, 1.0),
        FrameSnapshot(frame, 2, 2, 7, 1.0),     # the loop outran the stream
        FrameSnapshot(frame, 2, 2, 8, 2.0),
        FrameSnapshot(frame, 2, 2, 8, 2.0),
    ]
    run_state = Mock()
    type(run_state).is_running = property(lambda _self: bool(snapshots))
    provider = Mock()
    provider.get_frame_snapshot.side_effect = lambda: snapshots.pop(0)
    detector = Mock()
    detector.detect.return_value = []
    generation = Mock()
    generation.publish_detection.return_value = True
    clock = itertools.count(1.0)
    loop = DetectionLoop(
        run_state, 0.5, provider, Mock(), detector, generation, RuntimeMetrics(), Mock(),
        LoopTiming(monotonic=lambda: next(clock), sleep=Mock()),
    )

    loop.run()

    assert detector.detect.call_count == 2
    assert generation.publish_detection.call_count == 2


def test_live_check_scores_kept_intervals_and_each_fault():
    sender = [i * 33_333_333 for i in range(100)]
    base = 7_000_000_000
    clean = [base + p for p in sender[10:90]]

    good = score_intervals(clean, sender)
    assert good.ok and good.sender_in_window == 80 and good.decoded == 80

    missing = clean[:30] + clean[31:]
    assert not score_intervals(missing, sender).ok
    assert score_intervals(missing, sender).missing_sender == 1

    retimed = list(clean)
    retimed[40] += 1_000_000
    assert score_intervals(retimed, sender).unmatched_decoded == 1

    repeated = clean[:30] + [clean[29]] + clean[30:]
    assert score_intervals(repeated, sender).repeated_pts == 1

    assert not score_intervals([], sender).ok
    assert not score_intervals([base, base + 5, base + 11], sender).ok


def test_live_check_scores_duplicate_los_updates():
    assert score_los([(1, 1.0), (2, 2.0)]) == {
        "los_updates": 2, "duplicate_pictures": 0, "duplicate_times": 0}
    assert score_los([(1, 1.0), (1, 2.0), (2, 2.0)]) == {
        "los_updates": 3, "duplicate_pictures": 1, "duplicate_times": 1}


_HAS_GI = importlib.util.find_spec("gi") is not None


@pytest.mark.skipif(not _HAS_GI, reason="GStreamer bindings exist only in WSL and on the Orin")
def test_real_gstreamer_takes_the_pinned_settings():
    gst = module._load_gst()
    capture = GstreamerFrameCapture.open(SOURCE, _Logger())
    assert capture is not None
    pipeline = gst.parse_launch(parse_gstreamer_source(SOURCE).pipeline_text())
    try:
        capture._record_settings(pipeline)
    finally:
        pipeline.set_state(gst.State.NULL)
    assert capture.recorded["buffer_mode"] == "none"
    assert capture.recorded["latency_ms"] == 50


@pytest.mark.skipif(not _HAS_GI, reason="GStreamer bindings exist only in WSL and on the Orin")
def test_real_gstreamer_refused_connection_never_publishes(monkeypatch):
    monkeypatch.setattr(module, "FIRST_FRAME_DEADLINE_S", 1.0)
    monkeypatch.setattr(module, "CONNECT_RETRY_S", 0.1)
    capture = GstreamerFrameCapture.open(
        "gstreamer+rtsp://127.0.0.1:1/none?decoder=avdec_h265&latency_ms=50", _Logger(),
    )
    with pytest.raises(EncodedStreamReset, match="no frame within"):
        _run(capture)
    assert capture.connect_attempts >= 1
