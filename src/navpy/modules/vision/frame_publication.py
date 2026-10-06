"""Atomic latest-frame publication and waiter coordination."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional, Tuple

import numpy as np

from navpy.modules.vision.frame_capture_ports import CaptureStamp


@dataclass(frozen=True)
class FrameSnapshot:
    """One published frame and the metadata that identifies that publication.

    `published_at_s` is stamped HERE, at publication, and stays the
    receipt-liveness time. `capture` is what the BACKEND declared about when
    the frame physically existed; None means the source made no declaration
    and the frame can never be frame-atomic downstream.
    """

    frame: Optional[np.ndarray]
    width: int
    height: int
    sequence: int
    published_at_s: Optional[float]
    capture: Optional[CaptureStamp] = None


class FramePublicationStore:
    """Own the single atomic publication slot and all waiter state."""

    def __init__(
        self,
        wall_time: Callable[[], float] | None = None,
        monotonic: Callable[[], float] | None = None,
    ) -> None:
        self._condition = threading.Condition(threading.Lock())
        self._snapshot = FrameSnapshot(None, 0, 0, 0, None)
        self._is_running = False
        self._wall_time = wall_time
        self._monotonic = monotonic

    def set_running(self, running: bool) -> None:
        with self._condition:
            self._is_running = bool(running)
            if not self._is_running:
                self._condition.notify_all()

    def publish(
        self,
        frame: np.ndarray | None,
        capture: CaptureStamp | None = None,
    ) -> None:
        """Publish the newest frame state and wake waiting consumers."""
        with self._condition:
            if frame is None:
                width = 0
                height = 0
            else:
                height, width = frame.shape[:2]
            self._snapshot = FrameSnapshot(
                frame=frame,
                width=width,
                height=height,
                sequence=self._snapshot.sequence + 1,
                published_at_s=(
                    time.time()
                    if self._wall_time is None
                    else self._wall_time()
                ),
                capture=capture,
            )
            self._condition.notify_all()

    def get_frame(self) -> Tuple[Optional[np.ndarray], int, int]:
        with self._condition:
            snapshot = self._snapshot
            return snapshot.frame, snapshot.width, snapshot.height

    def get_frame_state(
        self,
    ) -> Tuple[Optional[np.ndarray], int, int, int]:
        with self._condition:
            snapshot = self._snapshot
            return (
                snapshot.frame,
                snapshot.width,
                snapshot.height,
                snapshot.sequence,
            )

    def snapshot(self) -> FrameSnapshot:
        with self._condition:
            return self._snapshot

    def sequence(self) -> int:
        with self._condition:
            return self._snapshot.sequence

    def wait_for_newer(
        self,
        after_seq: int,
        timeout: Optional[float] = None,
    ) -> Optional[Tuple[Optional[np.ndarray], int, int, int]]:
        """Block until a publication newer than ``after_seq`` is available."""
        now = time.monotonic if self._monotonic is None else self._monotonic
        deadline = None if timeout is None else (now() + timeout)
        with self._condition:
            while self._snapshot.sequence <= after_seq:
                if not self._is_running:
                    return None
                if deadline is None:
                    self._condition.wait()
                    continue
                remaining = deadline - now()
                if remaining <= 0.0:
                    return None
                self._condition.wait(remaining)
            snapshot = self._snapshot
            return (
                snapshot.frame,
                snapshot.width,
                snapshot.height,
                snapshot.sequence,
            )


__all__ = ["FramePublicationStore", "FrameSnapshot"]
