"""Narrow ports shared by frame-capture backends and their runtime."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Protocol

import numpy as np


class CaptureLogger(Protocol):
    def info(self, message: str) -> None: ...

    def warning(self, message: str) -> None: ...

    def error(self, message: str) -> None: ...


class CaptureStampKind(Enum):
    """What the capture timestamp physically describes.

    EXPOSURE: the stamp is exposure-domain truth (sim, tests, future PTS).
    READ: the stamp was taken when the backend could first observe the frame
    (after grab/pipe read); exposure happened EARLIER by the camera-path
    delay, which only a certified calibration can account for.
    """

    EXPOSURE = "exposure"
    READ = "read"


@dataclass(frozen=True)
class FrameSourceIdentity:
    """Which picture of which open: the media PTS, kept as the sender stamped it."""

    open_id: str
    sequence: int
    pts_ns: int


@dataclass(frozen=True)
class CaptureStamp:
    """Earliest observable capture time for one frame, and its meaning.

    `captured_at_s` is on the capture clock (``rt.perf_counter``), shared with
    the ATTITUDE ``capture_receipt_s`` and never the wall clock. A frame published
    WITHOUT a stamp is never frame-atomic downstream; capture time is never
    synthesized from publication time. `source` is set by a backend that reads
    a timestamped stream.
    """

    captured_at_s: float
    kind: CaptureStampKind
    source: FrameSourceIdentity | None = None


class BoardClock(Protocol):
    """Map a time on the autopilot (board) clock onto the capture clock.

    Returns None while no mapping is available (no ATTITUDE yet, a reboot,
    or a stamp too far from the newest ATTITUDE); a frame is then never
    given an EXPOSURE stamp.
    """

    def capture_time_s(self, board_time_us: int) -> float | None: ...


class FramePublisher(Protocol):
    def __call__(
        self,
        frame: np.ndarray | None,
        capture: CaptureStamp | None,
    ) -> None: ...


class CaptureRunState(Protocol):
    def __call__(self) -> bool: ...


class FrameCaptureBackend(Protocol):
    def run(
        self,
        publish: FramePublisher,
        is_running: CaptureRunState,
    ) -> None: ...

    def request_stop(self) -> bool: ...

    def close(self) -> None: ...


class CaptureBackendWrapper(Protocol):
    """Wraps one opened backend after decode: a HITL commissioning dependency, supplied by a launcher, never a profile.

    The wrapper owns the backend it returns; it keeps each frame's ``CaptureStamp`` as the backend made it.
    """

    def __call__(
        self,
        backend: FrameCaptureBackend,
        logger: CaptureLogger,
    ) -> FrameCaptureBackend: ...


__all__ = [
    "BoardClock",
    "CaptureBackendWrapper",
    "CaptureLogger",
    "CaptureRunState",
    "CaptureStamp",
    "CaptureStampKind",
    "FrameCaptureBackend",
    "FramePublisher",
    "FrameSourceIdentity",
]
