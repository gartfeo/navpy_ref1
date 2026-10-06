"""Low-latency FFmpeg RTSP frame capture."""

from __future__ import annotations

from navpy.exception_groups import BaseExceptionGroup

import subprocess
import threading
import time
from typing import Optional

import numpy as np

from navpy.modules.vision.frame_capture_ports import (
    CaptureLogger,
    CaptureRunState,
    CaptureStamp,
    CaptureStampKind,
    FramePublisher,
)
from navpy.modules.vision.frame_capture_ffmpeg_tools import (
    ffmpeg_binary,
    open_raw_video_process,
    probe_resolution,
)


class FfmpegFrameCapture:
    """Own one FFmpeg raw-video process and decode its frames."""

    def __init__(
        self,
        process: subprocess.Popen[bytes],
        width: int,
        height: int,
        logger: CaptureLogger,
    ) -> None:
        self._process = process
        self._width = int(width)
        self._height = int(height)
        self._frame_size = self._width * self._height * 3
        self._logger = logger
        self._lock = threading.RLock()
        self._stop_requested = False
        self._closed = False
        self._stderr_thread: threading.Thread | None = None

    @classmethod
    def open(
        cls,
        url: str,
        logger: CaptureLogger,
    ) -> Optional["FfmpegFrameCapture"]:
        resolution = probe_resolution(url, logger)
        if resolution is None:
            logger.warning(f"FrameProvider: ffprobe failed for {url}")
            return None
        width, height = resolution
        process = open_raw_video_process(url)
        if process is None:
            return None
        try:
            capture = cls(process, width, height, logger)
        except BaseException as owner_error:
            cleanup_errors: list[BaseException] = []
            for cleanup in (
                process.kill,
                lambda: process.wait(timeout=2.0),
            ):
                try:
                    cleanup()
                except BaseException as cleanup_error:
                    cleanup_errors.append(cleanup_error)
            if cleanup_errors:
                raise BaseExceptionGroup(
                    "ffmpeg ownership and rollback failed",
                    [owner_error, *cleanup_errors],
                ) from None
            raise
        try:
            stderr_thread = threading.Thread(
                target=capture._drain_stderr,
                args=(process.stderr,),
                daemon=True,
            )
            capture._stderr_thread = stderr_thread
            stderr_thread.start()
        except BaseException as start_error:
            try:
                capture.close()
            except BaseException as cleanup_error:
                raise BaseExceptionGroup(
                    "ffmpeg stderr worker startup and cleanup failed",
                    [start_error, cleanup_error],
                ) from None
            raise
        logger.info(
            f"FrameProvider: opened {url} ({width}x{height}) via ffmpeg pipe"
        )
        return capture

    def run(
        self,
        publish: FramePublisher,
        is_running: CaptureRunState,
    ) -> None:
        stdout = self._process.stdout
        if stdout is None:
            return
        while is_running():
            raw = stdout.read(self._frame_size)
            # Stamped when the frame's last byte leaves the pipe: the
            # earliest point this process can observe it. The rawvideo pipe
            # carries no PTS; everything upstream (camera, network, ffmpeg
            # process) is invisible here — CaptureStampKind.READ declares
            # exactly that.
            read_at_s = time.time()
            if len(raw) != self._frame_size:
                if is_running():
                    self._logger.warning("FrameProvider: ffmpeg pipe closed")
                break
            frame = np.frombuffer(raw, dtype=np.uint8).reshape(
                self._height,
                self._width,
                3,
            ).copy()
            publish(frame, CaptureStamp(read_at_s, CaptureStampKind.READ))

    def _drain_stderr(self, stream: object) -> None:
        if stream is None:
            return
        for raw in iter(stream.readline, b""):
            line = raw.decode("utf-8", errors="replace").rstrip()
            if line:
                self._logger.warning(f"ffmpeg: {line}")

    def request_stop(self) -> bool:
        with self._lock:
            if self._stop_requested:
                return True
            try:
                self._process.kill()
            except OSError:
                return False
            self._stop_requested = True
            return True

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            if not self.request_stop():
                raise OSError("ffmpeg process could not be stopped")
            try:
                self._process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self._stop_requested = False
                raise
            thread = self._stderr_thread
            if thread is not None:
                if thread is threading.current_thread():
                    raise RuntimeError(
                        "ffmpeg stderr worker cannot close itself"
                    )
                if not thread.is_alive():
                    self._closed = True
                    return
                thread.join(timeout=2.0)
                if thread.is_alive():
                    raise TimeoutError(
                        "ffmpeg stderr worker did not stop within 2.000s"
                    )
            self._closed = True


__all__ = [
    "FfmpegFrameCapture",
    "ffmpeg_binary",
    "probe_resolution",
]
