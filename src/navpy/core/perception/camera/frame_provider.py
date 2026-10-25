"""Stable public adapter for threaded and push-mode frame publication."""

from __future__ import annotations

from typing import Optional, Tuple

import numpy as np

from navpy.core.ports.frame_capture_ports import BoardClock, CaptureLogger
from navpy.core.perception.camera.frame_capture_runtime import (
    FrameProviderParts,
    build_frame_provider_parts,
)
from navpy.core.perception.camera.frame_publication import FrameSnapshot

# Newest frame age up to which the detector camera reports ready (camera
# HEARTBEAT ACTIVE; docs/design/gcs_preflight_camera_row.md). 25 or more frame
# periods at the ZR10's 25-30 fps, so only a stalled capture fails it; a stall
# shows within two 1 Hz camera heartbeats. Provisional until the Orin and
# ZR10 bench check measures the publication jitter.
DETECTOR_FRAME_FRESH_S = 1.0


class FrameProvider:
    """One-field adapter over capture lifecycle and atomic publications."""

    def __init__(
        self,
        source: int | str | None,
        logger: CaptureLogger,
        board_clock: BoardClock | None = None,
    ) -> None:
        self._parts: FrameProviderParts = build_frame_provider_parts(
            source,
            logger,
            board_clock,
        )

    def start(self) -> None:
        self._parts.capture.start()

    def stop(self) -> bool:
        return self._parts.capture.stop()

    def raise_if_failed(self) -> None:
        self._parts.capture.raise_if_failed()

    def push_frame(self, frame: Optional[np.ndarray]) -> None:
        """Take ownership of caller pixels before publishing them."""
        self._parts.publications.publish(
            None if frame is None else frame.copy()
        )

    def get_frame(self) -> Tuple[Optional[np.ndarray], int, int]:
        return self._parts.publications.get_frame()

    def get_frame_state(
        self,
    ) -> Tuple[Optional[np.ndarray], int, int, int]:
        return self._parts.publications.get_frame_state()

    def get_frame_snapshot(self) -> FrameSnapshot:
        return self._parts.publications.snapshot()

    def get_frame_seq(self) -> int:
        return self._parts.publications.sequence()

    def has_fresh_frame(self) -> bool:
        """Display only: the running capture published a frame recently."""
        return self._parts.publications.has_fresh_frame(DETECTOR_FRAME_FRESH_S)

    def wait_for_newer_frame(
        self,
        after_seq: int,
        timeout: Optional[float] = None,
    ) -> Optional[Tuple[Optional[np.ndarray], int, int, int]]:
        return self._parts.publications.wait_for_newer(after_seq, timeout)


__all__ = ["FrameProvider", "FrameSnapshot"]
