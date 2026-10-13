"""Ordered evidence for a pass latched without a new frame (dock lost centred).

The latch happens on the navigation thread, but command evidence is written
by the command worker's ordered postprocess FIFO after it leaves the runtime
session. Writing the pass row directly could overtake an already-issued
command's FINAL_APPROACH_CMD row. So the immutable pass frame is staged here
and taken by the worker as its next work item, ahead of any newer frame,
which puts its row behind all earlier command evidence in the same FIFO. A
phase reset before the worker took it records it inline, after the
postprocess fence drained that earlier evidence.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field

from navpy.core.approach.navigation_postprocess_dispatcher import PostprocessJob
from navpy.core.approach.vision_nav.command_postprocess import (
    FinalApproachPostprocessFence,
)
from navpy.core.kernel import runtime_time as rt
from navpy.core.ports.frame import FinalApproachVisionFrame


@dataclass
class FinalApproachPassEvidenceWork:
    """Worker-owned pass evidence; holds one postprocess fence slot."""

    frame: FinalApproachVisionFrame
    _record: Callable[[FinalApproachVisionFrame], None] = field(repr=False)
    _fence: FinalApproachPostprocessFence | None = field(repr=False)

    def postprocess_job(self) -> PostprocessJob | None:
        if self._fence is None:
            return None
        return PostprocessJob(execute=self._execute, abandon=self._release)

    def _execute(self) -> None:
        try:
            self._record(self.frame)
        finally:
            self._release()

    def _release(self) -> None:
        fence, self._fence = self._fence, None
        if fence is not None:
            fence.finish()


class FinalApproachPassEvidence:
    """One-shot staging of the pass frame between latch and evidence row."""

    def __init__(
        self,
        lock: rt.RLock,
        record: Callable[[FinalApproachVisionFrame], None],
        fence: FinalApproachPostprocessFence,
    ) -> None:
        self._lock = lock
        self._record = record
        self._fence = fence
        self._staged: FinalApproachVisionFrame | None = None

    @property
    def pending(self) -> bool:
        with self._lock:
            return self._staged is not None

    def stage(self, frame: FinalApproachVisionFrame) -> None:
        with self._lock:
            self._staged = frame

    def take_work(self) -> FinalApproachPassEvidenceWork | None:
        """Worker: own the staged evidence and its fence slot."""
        with self._lock:
            frame, self._staged = self._staged, None
            if frame is None:
                return None
            self._fence.begin()
            return FinalApproachPassEvidenceWork(frame, self._record, self._fence)

    def take_staged(self) -> FinalApproachVisionFrame | None:
        """Reset: take evidence the worker never took (record after drain)."""
        with self._lock:
            frame, self._staged = self._staged, None
            return frame

    def record_now(self, frame: FinalApproachVisionFrame) -> None:
        self._record(frame)


__all__ = ["FinalApproachPassEvidence", "FinalApproachPassEvidenceWork"]
