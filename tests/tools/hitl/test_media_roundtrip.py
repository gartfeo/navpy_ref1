"""The in-process encode/decode (tools/hitl/media_roundtrip.py) over a fake Gst, and over the real one where the
distro bindings are importable (the pc-wsl host)."""
from __future__ import annotations

from types import SimpleNamespace

import pytest

from tools.hitl import media_roundtrip as rt


class Caps:
    def __init__(self, fmt: str, width: int, height: int) -> None:
        self.values = {"format": fmt, "width": width, "height": height}

    def get_structure(self, index):
        return SimpleNamespace(get_value=self.values.get)

    def to_string(self) -> str:
        return f"video/x-raw, format=(string){self.values['format']}"


class FakePipeline:
    """Decodes `decoded` of the pushed frames in shape `shape`; optionally refuses to play or reports an error."""

    def __init__(self, gst, decoded=None, shape=("BGR", rt.WIDTH, rt.HEIGHT), plays=True, error=None, eos=True):
        self.gst, self.decoded, self.shape, self.plays, self.error, self.eos = gst, decoded, shape, plays, error, eos
        self.pushed, self.ended, self.states, self.samples = 0, False, [], []
        pipeline = self

        class Element:
            def emit(self, signal, *args):
                if signal == "push-buffer":
                    pipeline.pushed += 1
                    return gst.FlowReturn.OK
                if signal == "end-of-stream":
                    pipeline.ended = True
                    count = pipeline.pushed if pipeline.decoded is None else pipeline.decoded
                    pipeline.samples = [SimpleNamespace(get_caps=lambda: Caps(*pipeline.shape))] * count
                    return gst.FlowReturn.OK
                if signal == "try-pull-sample":
                    return pipeline.samples.pop() if pipeline.samples else None
                raise AssertionError(signal)

            def get_property(self, name):
                assert name == "eos"
                return pipeline.eos and pipeline.ended

        self.element = Element()

    def get_by_name(self, name):
        return self.element

    def set_state(self, state):
        self.states.append(state)
        return self.gst.StateChangeReturn.FAILURE if not self.plays and state == "PLAYING" else "SUCCESS"

    def get_bus(self):
        error = self.error

        def pop_filtered(kind):
            if error is None:
                return None
            return SimpleNamespace(parse_error=lambda: (SimpleNamespace(message=error), "debug info"))
        return SimpleNamespace(pop_filtered=pop_filtered)


def fake_gst(**pipeline_options):
    gst = SimpleNamespace(SECOND=10 ** 9, State=SimpleNamespace(PLAYING="PLAYING", NULL="NULL"),
                          StateChangeReturn=SimpleNamespace(FAILURE="FAILURE"),
                          FlowReturn=SimpleNamespace(OK="OK"), MessageType=SimpleNamespace(ERROR="ERROR"),
                          Buffer=SimpleNamespace(new_wrapped=lambda data: SimpleNamespace(size=len(data))))
    gst.texts = []

    def parse_launch(text):
        gst.texts.append(text)
        if pipeline_options.get("unbuildable"):
            raise RuntimeError("no element \"nvcudah265enc\"")
        gst.pipeline = FakePipeline(gst, **{k: v for k, v in pipeline_options.items() if k != "unbuildable"})
        return gst.pipeline
    gst.parse_launch = parse_launch
    return gst


def test_every_pushed_frame_is_decoded_in_the_output_shape():
    gst = fake_gst()
    found, problems = rt.roundtrip(gst, "nvcudah265enc", "nvh265dec")
    assert problems == [] and found["frames_pushed"] == found["frames_decoded"] == rt.FRAMES
    assert "! nvcudah265enc ! " in gst.texts[0] and "! nvh265dec ! " in gst.texts[0]
    assert "rtph265pay ! rtph265depay" in gst.texts[0]
    assert gst.pipeline.states == ["PLAYING", "NULL"]


def test_missing_frames_wrong_shape_or_errors_are_problems():
    _, problems = rt.roundtrip(fake_gst(decoded=rt.FRAMES - 3), "x265enc", "avdec_h265")
    assert f"roundtrip decoded {rt.FRAMES - 3} of {rt.FRAMES} frames" in problems
    _, problems = rt.roundtrip(fake_gst(shape=("I420", rt.WIDTH, rt.HEIGHT)), "x265enc", "avdec_h265")
    assert any("not BGR 640x360" in p for p in problems)
    _, problems = rt.roundtrip(fake_gst(error="Could not configure supporting library"), "x265enc", "avdec_h265")
    assert "roundtrip pipeline error: Could not configure supporting library (debug info)" in problems
    _, problems = rt.roundtrip(fake_gst(eos=False), "x265enc", "avdec_h265")
    assert any("did not reach end of stream" in p for p in problems)


def test_unbuildable_or_unplayable_pipeline_is_a_problem():
    found, problems = rt.roundtrip(fake_gst(unbuildable=True), "nvcudah265enc", "nvh265dec")
    assert problems == ['roundtrip pipeline cannot be built: no element "nvcudah265enc"']
    assert found["frames_decoded"] == 0
    gst = fake_gst(plays=False, error="Selected preset not supported")
    _, problems = rt.roundtrip(gst, "nvcudah265enc", "nvh265dec")
    assert problems[0] == "roundtrip pipeline will not play: Selected preset not supported (debug info)"
    assert gst.pipeline.states[-1] == "NULL"


def test_real_software_roundtrip_where_the_distro_bindings_exist():
    gi = pytest.importorskip("gi")
    gi.require_version("Gst", "1.0")
    from gi.repository import Gst
    Gst.init(None)
    found, problems = rt.roundtrip(Gst, "x265enc", "avdec_h265")
    assert problems == [] and found["frames_decoded"] == rt.FRAMES
