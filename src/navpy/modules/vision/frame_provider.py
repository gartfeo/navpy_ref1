"""Stable public adapter for threaded and push-mode frame publication."""

from __future__ import annotations

from typing import Optional, Tuple

import numpy as np

from navpy.modules.vision.frame_capture_ports import CaptureLogger
from navpy.modules.vision.frame_capture_runtime import (
    FrameProviderParts,
    build_frame_provider_parts,
)
from navpy.modules.vision.frame_publication import FrameSnapshot


class FrameProvider:
    """One-field adapter over capture lifecycle and atomic publications."""

    def __init__(
        self,
        source: int | str | None,
        logger: CaptureLogger,
    ) -> None:
        self._parts: FrameProviderParts = build_frame_provider_parts(
            source,
            logger,
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

    def wait_for_newer_frame(
        self,
        after_seq: int,
        timeout: Optional[float] = None,
    ) -> Optional[Tuple[Optional[np.ndarray], int, int, int]]:
        return self._parts.publications.wait_for_newer(after_seq, timeout)


__all__ = ["FrameProvider", "FrameSnapshot"]
