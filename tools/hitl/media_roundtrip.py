"""In-process encode/decode of real buffers for the media runtime check (check_media_runtime.py): appsrc -> selected
encoder -> h265parse -> rtph265pay -> rtph265depay -> h265parse -> selected decoder -> appsink, no network. Finding
and creating elements does not prove they pass frames: an NVENC element can register and still refuse to configure.
Uses only GStreamer 1.20 API (signals, not GstApp methods). Standard library only; Gst is passed in.
"""
from __future__ import annotations

from typing import Any

FRAMES = 30
WIDTH, HEIGHT, FPS = 640, 360, 30
INPUT_CAPS = f"video/x-raw,format=I420,width={WIDTH},height={HEIGHT},framerate={FPS}/1"
OUTPUT_FORMAT = "BGR"                    # what navpy's OpenCV code reads
# How long one decoded frame may take to arrive after the previous one before the run counts it missing. Encoders
# hold frames until end of stream (x265enc lookahead), so this bounds the whole flush, not one frame's latency.
PULL_TIMEOUT_S = 10
NOTHING_EXCHANGED = {"frames_pushed": 0, "frames_decoded": 0, "caps": None, "format": None, "width": None,
                     "height": None}


def _pipeline_text(encoder: str, decoder: str) -> str:
    return (f"appsrc name=src format=time caps={INPUT_CAPS} ! videoconvert ! {encoder} ! "
            f"h265parse config-interval=-1 ! rtph265pay ! rtph265depay ! h265parse ! {decoder} ! videoconvert ! "
            f"video/x-raw,format={OUTPUT_FORMAT} ! appsink name=sink sync=false")


def _frame(gst: Any, index: int) -> Any:
    size = WIDTH * HEIGHT * 3 // 2
    luma = WIDTH * HEIGHT
    data = bytes((16 + 7 * index + x) % 236 for x in range(256)) * (luma // 256) + bytes([128]) * (size - luma)
    buffer = gst.Buffer.new_wrapped(data)
    buffer.pts = index * gst.SECOND // FPS
    buffer.duration = gst.SECOND // FPS
    return buffer


def _error_text(pipeline: Any, gst: Any) -> str:
    message = pipeline.get_bus().pop_filtered(gst.MessageType.ERROR)
    if message is None:
        return ""
    error, debug = message.parse_error()
    return f": {error.message} ({debug})" if debug else f": {error.message}"


def roundtrip(gst: Any, encoder: str, decoder: str) -> tuple[dict[str, Any], list[str]]:
    """Push FRAMES frames through encoder and decoder; the counts and output caps, and every problem."""
    found: dict[str, Any] = dict(NOTHING_EXCHANGED)
    shapes = set()
    try:
        pipeline = gst.parse_launch(_pipeline_text(encoder, decoder))
    except Exception as exc:                # GLib.Error: an element or caps the pipeline text names is unusable
        return found, [f"roundtrip pipeline cannot be built: {exc}"]
    problems: list[str] = []
    source, sink = pipeline.get_by_name("src"), pipeline.get_by_name("sink")
    try:
        if pipeline.set_state(gst.State.PLAYING) == gst.StateChangeReturn.FAILURE:
            return found, [f"roundtrip pipeline will not play{_error_text(pipeline, gst)}"]
        for index in range(FRAMES):
            flow = source.emit("push-buffer", _frame(gst, index))
            if flow != gst.FlowReturn.OK:
                problems.append(f"roundtrip push of frame {index} returned {flow}{_error_text(pipeline, gst)}")
                break
            found["frames_pushed"] += 1
        source.emit("end-of-stream")
        while (sample := sink.emit("try-pull-sample", PULL_TIMEOUT_S * gst.SECOND)) is not None:
            found["frames_decoded"] += 1
            caps = sample.get_caps()
            structure = caps.get_structure(0)
            found["caps"] = caps.to_string()
            shape = (structure.get_value("format"), structure.get_value("width"), structure.get_value("height"))
            shapes.add(shape)
            found["format"], found["width"], found["height"] = shape
        error = _error_text(pipeline, gst)
        if error:
            problems.append(f"roundtrip pipeline error{error}")
        if not sink.get_property("eos"):
            problems.append(f"roundtrip did not reach end of stream within {PULL_TIMEOUT_S} s of the last frame")
    finally:
        pipeline.set_state(gst.State.NULL)
    if found["frames_decoded"] != FRAMES:
        problems.append(f"roundtrip decoded {found['frames_decoded']} of {FRAMES} frames")
    if shapes - {(OUTPUT_FORMAT, WIDTH, HEIGHT)}:
        problems.append(f"roundtrip output frames are {sorted(shapes)}, not {OUTPUT_FORMAT} {WIDTH}x{HEIGHT}")
    return found, problems
