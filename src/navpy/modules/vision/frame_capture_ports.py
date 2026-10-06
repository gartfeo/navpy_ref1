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
class CaptureStamp:
    """Earliest observable capture time for one frame, and its meaning.

    `captured_at_s` shares the ATTITUDE receipt wall clock. A frame published
    WITHOUT a stamp is never frame-atomic downstream; capture time is never
    synthesized from publication time.
    """

    captured_at_s: float
    kind: CaptureStampKind


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


__all__ = [
    "CaptureLogger",
    "CaptureRunState",
    "CaptureStamp",
    "CaptureStampKind",
    "FrameCaptureBackend",
    "FramePublisher",
]
