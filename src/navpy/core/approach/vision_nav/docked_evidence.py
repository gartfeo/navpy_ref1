"""Ordered DOCKED evidence for a contact latched off the command thread.

The contact latches on the telemetry thread, but command evidence is written
by the command worker's ordered postprocess FIFO after it leaves the runtime
session. Writing the DOCKED row directly could overtake an already-issued
command's FINAL_APPROACH_CMD row. So the immutable contact event is staged
here and taken by the worker as its next work item, ahead of any newer frame,
which puts its row behind all earlier command evidence in the same FIFO. A
phase reset before the worker took it records it inline, after the
postprocess fence drained that earlier evidence (and before the SNAP).
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field

from navpy.core.approach.navigation_postprocess_dispatcher import PostprocessJob
from navpy.core.approach.vision_nav.command_postprocess import (
    FinalApproachPostprocessFence,
)
from navpy.core.kernel import runtime_time as rt
from navpy.core.ports.dock_contact import DockContactEvent, DockContactIdentity


@dataclass
class FinalApproachDockedEvidenceWork:
    """Worker-owned DOCKED evidence; holds one postprocess fence slot."""

    event: DockContactEvent
    _record: Callable[[DockContactEvent], None] = field(repr=False)
    _fence: FinalApproachPostprocessFence | None = field(repr=False)

    def postprocess_job(self) -> PostprocessJob | None:
        if self._fence is None:
            return None
        return PostprocessJob(execute=self._execute, abandon=self._release)

    def _execute(self) -> None:
        try:
            self._record(self.event)
        finally:
            self._release()

    def _release(self) -> None:
        fence, self._fence = self._fence, None
        if fence is not None:
            fence.finish()


class FinalApproachDockedEvidence:
    """The episode's DOCK_ARMED row, then one staged DOCKED row.

    DOCK_ARMED is written at NAV entry, before any command row of the
    episode, so the eval can match a DOCKED row to its own episode.
    """

    def __init__(
        self,
        lock: rt.RLock,
        record: Callable[[DockContactEvent], None],
        fence: FinalApproachPostprocessFence,
        record_armed: Callable[[DockContactIdentity], None],
    ) -> None:
        self._lock = lock
        self._record = record
        self._fence = fence
        self._record_armed = record_armed
        self._staged: DockContactEvent | None = None

    def record_armed(self, identity: DockContactIdentity) -> None:
        self._record_armed(identity)

    @property
    def pending(self) -> bool:
        with self._lock:
            return self._staged is not None

    def stage(self, event: DockContactEvent) -> None:
        with self._lock:
            self._staged = event

    def take_work(self) -> FinalApproachDockedEvidenceWork | None:
        """Worker: own the staged evidence and its fence slot."""
        with self._lock:
            event, self._staged = self._staged, None
            if event is None:
                return None
            self._fence.begin()
            return FinalApproachDockedEvidenceWork(event, self._record, self._fence)

    def take_staged(self) -> DockContactEvent | None:
        """Reset: take evidence the worker never took (record after drain)."""
        with self._lock:
            event, self._staged = self._staged, None
            return event

    def record_now(self, event: DockContactEvent) -> None:
        self._record(event)


__all__ = ["FinalApproachDockedEvidence", "FinalApproachDockedEvidenceWork"]
