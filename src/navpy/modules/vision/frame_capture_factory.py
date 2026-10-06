"""Frame-capture backend selection."""

from __future__ import annotations

from navpy.exception_groups import BaseExceptionGroup

import os

import cv2

from navpy.modules.vision.frame_capture_cv_lease import OpenCvCaptureLease
from navpy.modules.vision.frame_capture_ffmpeg import FfmpegFrameCapture
from navpy.modules.vision.frame_capture_opencv import (
    OpenCvFrameCapture,
    open_jetson_capture,
    open_source_capture,
)
from navpy.modules.vision.frame_capture_ports import (
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


def open_capture_backend(
    source: int | str,
    logger: CaptureLogger,
) -> FrameCaptureBackend | None:
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
