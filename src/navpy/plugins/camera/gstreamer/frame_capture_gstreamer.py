"""Encoded RTSP video through GStreamer appsink, with no fallback.

Source ``gstreamer+rtsp://HOST:PORT/PATH?decoder=NAME&latency_ms=N``; only a
profile that names this scheme gets this backend. Anything that stops it from
opening is a failed start, never another decoder path.

rtspsrc is pinned to ``buffer-mode=none`` (RTP clock only), ``ntp-sync=false``
and ``rfc7273-sync=false``, with a fixed jitter-buffer latency, and appsink to
``sync=false``, so PTS keeps the sender's intervals and the receiver clock does
not hold frames back. A frame's PTS, its sequence in this open and the open's
identity travel in ``CaptureStamp.source``; the stamp itself is READ, taken when
the sample was pulled (certified exposure timing is not claimed here). The
sequence counts this open's accepted frames; the publisher's own frame numbers
do not cross the stream, so PTS is the key that joins to its records.

The decoder's plugin loads in ``open``, before any stream: its start can stall
a running stream (HITL step 8). Until the first frame, a refused connection is
retried, because MediaMTX refuses readers until the publisher is up. After the
first frame nothing reconnects: a disconnect or reset ends this open, the
capture worker records the failure, and only a new run may open the stream.

GStreamer 1.20 APIs only: this code also runs on the Orin.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Any, Optional

import numpy as np

from navpy.core.kernel import runtime_time as rt
from navpy.core.ports.frame_capture_ports import (
    CaptureLogger,
    CaptureRunState,
    CaptureStamp,
    CaptureStampKind,
    FramePublisher,
    FrameSourceIdentity,
)
from navpy.plugins.camera.gstreamer.encoded_frame_gate import (
    EncodedFrameGate,
    EncodedStreamReset,
    FrameFacts,
)
from navpy.plugins.camera.gstreamer.encoded_source import (
    EncodedSourceSettings,
    parse_gstreamer_source,
)

# Long enough for Unreal's and the publisher's start beside a loaded PC. Ends a run that never gets a stream.
FIRST_FRAME_DEADLINE_S = 300.0
CONNECT_RETRY_S = 1.0
# Transport death, not frame freshness (HITL steps 15 and 18 own that): rtspsrc's own UDP
# timeout is 5 s, and step 8 saw single stream gaps of up to 2.6 s under load.
SAMPLE_GAP_LIMIT_S = 5.0
_PULL_SLICE_NS = 100_000_000

FrameObserver = Callable[[FrameSourceIdentity, float], None]


def _load_gst() -> Any:
    import gi

    gi.require_version("Gst", "1.0")
    from gi.repository import Gst

    Gst.init(None)
    return Gst


class GstreamerFrameCapture:
    """Own one RTSP -> decoder -> appsink pipeline for one open."""

    def __init__(
        self,
        gst: Any,
        settings: EncodedSourceSettings,
        logger: CaptureLogger,
        observer: Optional[FrameObserver] = None,
    ) -> None:
        self._gst = gst
        self._settings = settings
        self._logger = logger
        self._observer = observer
        self._lock = rt.Lock()
        self._pipeline: Any = None
        self._stop_requested = False
        self._closed = False
        self.open_id = str(rt.uuid4())
        self.gate = EncodedFrameGate()
        self.connect_attempts = 0
        self.recorded: dict[str, object] = {}

    @classmethod
    def open(
        cls,
        source: str,
        logger: CaptureLogger,
        observer: Optional[FrameObserver] = None,
        load_gst: Callable[[], Any] = _load_gst,
    ) -> Optional["GstreamerFrameCapture"]:
        try:
            settings = parse_gstreamer_source(source)
            gst = load_gst()
        except (ValueError, ImportError) as error:
            logger.error(f"FrameProvider: cannot open {source}: {error}")
            return None
        if gst.ElementFactory.make(settings.decoder, None) is None:
            logger.error(f"FrameProvider: cannot open {source}: no decoder {settings.decoder}")
            return None
        capture = cls(gst, settings, logger, observer)
        logger.info(f"FrameProvider: {source} ready, open {capture.open_id}, decoder loaded")
        return capture

    def run(self, publish: FramePublisher, is_running: CaptureRunState) -> None:
        deadline = rt.monotonic() + FIRST_FRAME_DEADLINE_S
        try:
            while is_running() and not self._stop_requested:
                if self._read_one_connection(publish, is_running, deadline):
                    return
                if rt.monotonic() >= deadline:
                    raise EncodedStreamReset(f"no frame within {FIRST_FRAME_DEADLINE_S} s")
                rt.sleep(CONNECT_RETRY_S)
        finally:
            self._logger.info(f"FrameProvider: open {self.open_id} ended: {self._counts()}")

    def _read_one_connection(
        self,
        publish: FramePublisher,
        is_running: CaptureRunState,
        deadline: float,
    ) -> bool:
        """True when reading is over; False when the first connection may be tried again."""
        gst = self._gst
        pipeline = gst.parse_launch(self._settings.pipeline_text())
        with self._lock:
            if self._stop_requested:
                pipeline.set_state(gst.State.NULL)
                return True
            self._pipeline = pipeline
        self.connect_attempts += 1
        self._record_settings(pipeline)
        sink = pipeline.get_by_name("sink")
        bus = pipeline.get_bus()
        try:
            if pipeline.set_state(gst.State.PLAYING) == gst.StateChangeReturn.FAILURE:
                return self._connection_failed("the pipeline refused PLAYING")
            last_frame_at = None
            while is_running() and not self._stop_requested:
                sample = sink.emit("try-pull-sample", _PULL_SLICE_NS)    # no GstApp typelib needed
                read_at_s = rt.time()
                now = rt.monotonic()
                if sample is not None:
                    newest_pts_ns = self.gate.last_pts_ns
                    self._handle_sample(sample, read_at_s, publish)
                    if self.gate.last_pts_ns != newest_pts_ns:     # a repeat proves nothing is alive
                        last_frame_at = now
                else:
                    ended = self._bus_end(bus)
                    if ended is not None:
                        return self._connection_failed(ended)
                if self.gate.last_pts_ns is not None:
                    if last_frame_at is not None and now - last_frame_at > SAMPLE_GAP_LIMIT_S:
                        raise EncodedStreamReset(f"no frame for {SAMPLE_GAP_LIMIT_S} s")
                elif now >= deadline:
                    raise EncodedStreamReset(f"no frame within {FIRST_FRAME_DEADLINE_S} s")
            return True
        finally:
            self._release_pipeline(pipeline)

    def _connection_failed(self, reason: str) -> bool:
        if self.gate.last_pts_ns is not None:
            raise EncodedStreamReset(f"stream ended: {reason}")
        self._logger.warning(f"FrameProvider: {self._settings.url} not available yet: {reason}")
        return False

    def _handle_sample(self, sample: Any, read_at_s: float, publish: FramePublisher) -> None:
        gst = self._gst
        buffer = sample.get_buffer()
        structure = sample.get_caps().get_structure(0)
        ok_w, width = structure.get_int("width")
        ok_h, height = structure.get_int("height")
        if not (ok_w and ok_h):
            raise EncodedStreamReset("decoded frame without a picture size")
        pts = buffer.pts
        sequence = self.gate.admit(FrameFacts(
            None if pts == gst.CLOCK_TIME_NONE else int(pts),
            width,
            height,
            buffer.has_flags(gst.BufferFlags.CORRUPTED),
        ))
        if sequence is None:
            return
        identity = FrameSourceIdentity(self.open_id, sequence, int(pts))
        frame = _bgr_pixels(buffer.extract_dup(0, buffer.get_size()), width, height)
        if self._observer is not None:
            self._observer(identity, read_at_s)
        publish(frame, CaptureStamp(read_at_s, CaptureStampKind.READ, identity))

    def _bus_end(self, bus: Any) -> Optional[str]:
        gst = self._gst
        while True:
            message = bus.pop_filtered(
                gst.MessageType.ERROR | gst.MessageType.EOS | gst.MessageType.WARNING
            )
            if message is None:
                return None
            if message.type == gst.MessageType.EOS:
                return "end of stream"
            if message.type == gst.MessageType.ERROR:
                error, debug = message.parse_error()
                return f"{error.message} ({debug})"
            warning, debug = message.parse_warning()
            self._logger.warning(f"FrameProvider: GStreamer warning: {warning.message} ({debug})")

    def _record_settings(self, pipeline: Any) -> None:
        """Read the pins back from the element, so the record says what actually runs."""
        src = pipeline.get_by_name("src")
        sink = pipeline.get_by_name("sink")
        recorded = {
            "url": self._settings.url,
            "decoder": self._settings.decoder,
            "open_id": self.open_id,
            "buffer_mode": src.get_property("buffer-mode").value_nick,
            "ntp_sync": src.get_property("ntp-sync"),
            "rfc7273_sync": src.get_property("rfc7273-sync"),
            "latency_ms": src.get_property("latency"),
            "appsink_sync": sink.get_property("sync"),
        }
        expected = {
            "buffer_mode": "none",
            "ntp_sync": False,
            "rfc7273_sync": False,
            "latency_ms": self._settings.latency_ms,
            "appsink_sync": False,
        }
        wrong = {key: recorded[key] for key, value in expected.items() if recorded[key] != value}
        if wrong:
            raise EncodedStreamReset(f"pipeline settings did not take: {wrong}")
        if not self.recorded:
            self._logger.info(f"FrameProvider: encoded capture settings {recorded}")
        self.recorded = recorded

    def _counts(self) -> str:
        gate = self.gate
        return (
            f"accepted={gate.accepted} duplicates={gate.duplicates} corrupted={gate.corrupted} "
            f"first_pts_ns={gate.first_pts_ns} last_pts_ns={gate.last_pts_ns} "
            f"connect_attempts={self.connect_attempts}"
        )

    def _release_pipeline(self, pipeline: Any) -> None:
        with self._lock:
            if self._pipeline is pipeline:
                self._pipeline = None
        pipeline.set_state(self._gst.State.NULL)

    def request_stop(self) -> bool:
        self._stop_requested = True     # the read loop sees it within one pull slice
        return True

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._stop_requested = True
            pipeline, self._pipeline = self._pipeline, None
            self._closed = True
        if pipeline is not None:
            pipeline.set_state(self._gst.State.NULL)


def _bgr_pixels(data: bytes, width: int, height: int) -> np.ndarray:
    """A writable BGR frame; a decoder's row padding is cut off."""
    stride = len(data) // height if height > 0 else 0
    if height <= 0 or width <= 0 or stride < width * 3 or stride * height != len(data):
        raise EncodedStreamReset(f"{len(data)} bytes is no {width} x {height} BGR frame")
    rows = np.frombuffer(data, dtype=np.uint8).reshape(height, stride)
    return rows[:, :width * 3].reshape(height, width, 3).copy()


__all__ = [
    "CONNECT_RETRY_S",
    "FIRST_FRAME_DEADLINE_S",
    "FrameObserver",
    "GstreamerFrameCapture",
    "SAMPLE_GAP_LIMIT_S",
]
