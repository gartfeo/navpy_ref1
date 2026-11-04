"""Frame-capture backend selection."""

from __future__ import annotations

from navpy.core.kernel.exception_groups import BaseExceptionGroup

import os

import cv2

from navpy.plugins.camera.opencv.frame_capture_cv_lease import OpenCvCaptureLease
from navpy.plugins.camera.ffmpeg.frame_capture_ffmpeg import FfmpegFrameCapture
from navpy.plugins.camera.gstreamer.encoded_source import is_gstreamer_source
from navpy.plugins.camera.gstreamer.frame_capture_gstreamer import GstreamerFrameCapture
from navpy.plugins.camera.unreal.frame_capture_npyc import NpycFrameCapture, is_npyc_source
from navpy.plugins.camera.opencv.frame_capture_opencv import (
    OpenCvFrameCapture,
    open_jetson_capture,
    open_source_capture,
)
from navpy.core.ports.frame_capture_ports import (
    BoardClock,
    CaptureBackendWrapper,
    CaptureLogger,
    FrameCaptureBackend,
)

_IS_JETSON = os.path.exists("/etc/nv_tegra_release")


def _own_opencv_capture(
    capture: cv2.VideoCapture,
    *,
    require_open: bool = False,
) -> OpenCvFrameCapture | None:
    with OpenCvCaptureLease(capture) as lease:
        if require_open and not capture.isOpened():
            return None
        backend = OpenCvFrameCapture(capture)
        lease.transfer()
        return backend


def _read_dimensions(
    backend: OpenCvFrameCapture,
) -> tuple[int, int]:
    try:
        return backend.dimensions
    except BaseException as owner_error:
        try:
            backend.close()
        except BaseException as cleanup_error:
            raise BaseExceptionGroup(
                "OpenCV initialization and cleanup failed",
                [owner_error, cleanup_error],
            ) from None
        raise


def _wrapped(
    backend: FrameCaptureBackend | None,
    wrap: CaptureBackendWrapper,
    logger: CaptureLogger,
) -> FrameCaptureBackend | None:
    if backend is None:
        return None
    try:
        wrapped = wrap(backend, logger)
    except BaseException as wrap_error:
        try:
            backend.close()
        except BaseException as cleanup_error:
            raise BaseExceptionGroup(
                "capture wrapper and cleanup failed",
                [wrap_error, cleanup_error],
            ) from None
        raise
    logger.warning(f"FrameProvider: capture wrapped by {type(wrapped).__name__}")
    return wrapped


def open_capture_backend(
    source: int | str,
    logger: CaptureLogger,
    board_clock: BoardClock | None = None,
    wrap: CaptureBackendWrapper | None = None,
) -> FrameCaptureBackend | None:
    """Open the backend for ``source``; ``board_clock`` stamps npyc:// frames.

    ``wrap`` (HITL commissioning only) wraps the decoded encoded stream and nothing else.
    """
    if wrap is not None:
        if not is_gstreamer_source(source):
            raise ValueError(
                "a capture wrapper wraps only the decoded encoded stream "
                f"(gstreamer+rtsp://), not {source!r}"
            )
        return _wrapped(GstreamerFrameCapture.open(str(source), logger), wrap, logger)
    if is_npyc_source(source):
        return NpycFrameCapture.open(str(source), logger, board_clock)
    if is_gstreamer_source(source):
        return GstreamerFrameCapture.open(str(source), logger)      # no fallback
    if isinstance(source, str) and source.lower().startswith("rtsp://"):
        if _IS_JETSON:
            capture = open_jetson_capture(source, logger)
            if capture is not None:
                return _own_opencv_capture(capture)
        ffmpeg = FfmpegFrameCapture.open(source, logger)
        if ffmpeg is not None:
            return ffmpeg
        logger.warning(
            "FrameProvider: ffmpeg not available, falling back to OpenCV"
        )

    capture = open_source_capture(source, logger)
    if capture is None:
        return None
    backend = _own_opencv_capture(capture, require_open=True)
    if backend is None:
        return None
    width, height = _read_dimensions(backend)
    logger.info(
        f"FrameProvider: opened {source} ({width}x{height}) via OpenCV"
    )
    return backend


__all__ = ["open_capture_backend"]
