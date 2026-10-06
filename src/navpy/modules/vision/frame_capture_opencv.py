"""OpenCV-backed webcam, stream, and Jetson frame capture."""

from __future__ import annotations

import os
import threading
import time
from typing import Optional

import cv2

from navpy.modules.vision.frame_capture_cv_lease import OpenCvCaptureLease
from navpy.modules.vision.frame_capture_ports import (
    CaptureLogger,
    CaptureRunState,
    CaptureStamp,
    CaptureStampKind,
    FramePublisher,
)


def build_jetson_gst_pipeline(url: str) -> str:
    """HEVC RTSP to NVDEC to BGR via appsink."""
    return (
        f"rtspsrc location={url} latency=0 ! "
        "rtph265depay ! h265parse ! nvv4l2decoder ! "
        "nvvidconv ! video/x-raw, format=BGRx ! "
        "videoconvert ! video/x-raw, format=BGR ! "
        "appsink drop=true max-buffers=1 sync=false"
    )


def open_jetson_capture(
    url: str,
    logger: CaptureLogger,
) -> Optional[cv2.VideoCapture]:
    pipeline = build_jetson_gst_pipeline(url)
    with OpenCvCaptureLease(
        cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    ) as lease:
        cap = lease.capture
        if not cap.isOpened():
            logger.warning("FrameProvider: gstreamer pipeline failed to open")
            return None
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        logger.info(
            f"FrameProvider: opened {url} ({width}x{height}) via GStreamer NVDEC"
        )
        return lease.transfer()


def open_webcam_capture(
    index: int,
    logger: CaptureLogger,
) -> Optional[cv2.VideoCapture]:
    """Open a webcam by index, trying common Windows backends."""
    backends = [cv2.CAP_DSHOW, cv2.CAP_MSMF, cv2.CAP_ANY]
    indices = [index] if index >= 0 else [0, 1, 2, 3]

    for candidate_index in indices:
        for backend in backends:
            with OpenCvCaptureLease(
                cv2.VideoCapture(candidate_index, backend)
            ) as lease:
                cap = lease.capture
                if not cap.isOpened():
                    continue
                try:
                    cap.set(
                        cv2.CAP_PROP_FOURCC,
                        cv2.VideoWriter_fourcc("M", "J", "P", "G"),
                    )
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                except cv2.error:
                    pass

                ok = False
                frame = None
                for _ in range(15):
                    ok, frame = cap.read()
                    if (
                        ok
                        and frame is not None
                        and frame.size > 0
                        and float(frame.mean()) > 5.0
                    ):
                        break
                    time.sleep(0.02)
                if (
                    ok
                    and frame is not None
                    and frame.size > 0
                    and float(frame.mean()) > 5.0
                ):
                    logger.info(
                        "FrameProvider: webcam opened "
                        f"index={candidate_index} backend={backend} "
                        f"size={frame.shape[1]}x{frame.shape[0]}"
                    )
                    return lease.transfer()

    logger.error(
        "FrameProvider: failed to open any webcam (all attempts black/not opened)"
    )
    return None


def open_stream_capture(url: str) -> Optional[cv2.VideoCapture]:
    """Open a stream URL or file path via OpenCV."""
    if url.lower().startswith("rtsp://"):
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
            "rtsp_transport;udp"
            "|stimeout;5000000"
            "|fflags;nobuffer"
            "|flags;low_delay"
            "|max_delay;0"
            "|probesize;32"
            "|analyzeduration;0"
        )

    with OpenCvCaptureLease(
        cv2.VideoCapture(url, cv2.CAP_FFMPEG)
    ) as primary:
        if primary.capture.isOpened():
            _configure_stream_capture(primary.capture)
            return primary.transfer()
    with OpenCvCaptureLease(cv2.VideoCapture(url)) as fallback:
        if not fallback.capture.isOpened():
            return None
        _configure_stream_capture(fallback.capture)
        return fallback.transfer()


def _configure_stream_capture(capture: cv2.VideoCapture) -> None:
    try:
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except cv2.error:
        pass


def open_source_capture(
    source: int | str | None,
    logger: CaptureLogger,
) -> Optional[cv2.VideoCapture]:
    if isinstance(source, int):
        return open_webcam_capture(source, logger)
    if isinstance(source, str):
        return open_stream_capture(source)
    return None


class OpenCvFrameCapture:
    """Continuously publish frames from one opened OpenCV resource."""

    def __init__(
        self,
        capture: cv2.VideoCapture,
    ) -> None:
        self._capture = capture
        self._capture_lock = threading.Lock()
        self._release_requested = threading.Event()
        self._released = False

    @property
    def dimensions(self) -> tuple[int, int]:
        return (
            int(self._capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
            int(self._capture.get(cv2.CAP_PROP_FRAME_HEIGHT)),
        )

    def run(
        self,
        publish: FramePublisher,
        is_running: CaptureRunState,
    ) -> None:
        while is_running():
            with self._capture_lock:
                if (
                    self._release_requested.is_set()
                    or self._released
                    or not is_running()
                ):
                    self._release_locked()
                    return
                grabbed = self._capture.grab()
                # Stamped when grab() RETURNS, before retrieve/decode/copy:
                # the earliest point this process can observe the frame. The
                # camera-path delay upstream of this stamp is invisible here,
                # which is exactly what CaptureStampKind.READ declares.
                grabbed_at_s = time.time()
                if grabbed:
                    ok, frame = self._capture.retrieve()
                else:
                    ok, frame = False, None
                if self._release_requested.is_set() or not is_running():
                    self._release_locked()
                    return
            if not grabbed:
                time.sleep(0.005)
                continue
            if ok and frame is not None:
                publish(
                    frame,
                    CaptureStamp(grabbed_at_s, CaptureStampKind.READ),
                )

    def request_stop(self) -> bool:
        self._release_requested.set()
        if not self._capture_lock.acquire(blocking=False):
            return False
        try:
            return self._release_locked()
        finally:
            self._capture_lock.release()

    def close(self) -> None:
        self._release_requested.set()
        with self._capture_lock:
            if not self._release_locked():
                raise RuntimeError("OpenCV capture could not be released")

    def _release_locked(self) -> bool:
        if self._released:
            return True
        try:
            self._capture.release()
        except cv2.error:
            return False
        self._released = True
        return True


__all__ = [
    "OpenCvFrameCapture",
    "build_jetson_gst_pipeline",
    "open_jetson_capture",
    "open_source_capture",
    "open_stream_capture",
    "open_webcam_capture",
]
