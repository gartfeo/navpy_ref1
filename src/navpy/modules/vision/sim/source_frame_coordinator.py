"""Frame-transaction coordination across focused simulator state owners."""

from __future__ import annotations

import threading
from contextlib import contextmanager
from dataclasses import replace
from typing import Iterator, Optional, Sequence, TypeVar

from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.models.detection_publication import DetectionPublication
from navpy.modules.vision.sim.detection_publication_store import (
    DetectionPublicationStore,
    PublicationSlot,
)
from navpy.modules.vision.target_priority import select_most_centered_target
from navpy.modules.vision.sim.frame_generation_gate import (
    FrameGeneration,
    FrameGenerationGate,
)
from navpy.modules.vision.sim.pose_inbox import PoseInbox
from navpy.modules.vision.sim.sim_coordinator_ports import PoseBuilder
from navpy.modules.vision.sim.sim_runtime_ports import (
    CapacityWarningSink,
    FrameOutcomeSink,
)


PoseT = TypeVar("PoseT")


class SourceFrameCoordinator:
    """Coordinate admission and transactions without owning their state."""

    def __init__(
        self,
        *,
        gate: FrameGenerationGate,
        inbox: PoseInbox,
        publications: DetectionPublicationStore,
        record_outcome: FrameOutcomeSink,
        overload_warning: CapacityWarningSink,
    ) -> None:
        self._gate = gate
        self._inbox = inbox
        self._publications = publications
        self._record_outcome = record_outcome
        self._overload_warning = overload_warning
        self._stop_event = threading.Event()
        self._reset_lock = threading.RLock()
        self._overload_active = False

    @property
    def token(self) -> FrameGeneration:
        return self._gate.token

    @property
    def stop_event(self) -> threading.Event:
        return self._stop_event

    @property
    def source_driven(self) -> bool:
        return self._publications.source_driven

    @contextmanager
    def admission(
        self,
        token: FrameGeneration,
    ) -> Iterator[Optional[FrameGeneration]]:
        """Fence pre-render sensor and source-clock mutations."""
        with self._gate.admission(token) as active:
            if active is None or self._stop_event.is_set():
                yield None
                return
            yield active

    def admit_pose(
        self,
        token: FrameGeneration,
        build_pose: PoseBuilder[PoseT],
    ) -> bool:
        dropped_poses: list[PoseT] = []
        dropped_events = []
        with self._gate.admission(token) as active:
            if active is None or self._stop_event.is_set():
                return False
            pose = build_pose(active)
            if pose is None or not self._gate.is_current(active):
                return False
            overloaded = self._inbox.is_full()
            if overloaded:
                active = self._gate.supersede()
                dropped_poses = self._inbox.clear()
                dropped_events = self._publications.clear(
                    mark_discontinuity=True,
                )
                pose = replace(pose, source_discontinuity=True)
                if not self._overload_active:
                    self._overload_warning(self._inbox.capacity)
                self._overload_active = True
            else:
                self._overload_active = False
            pose = replace(pose, generation=active)
            self._inbox.put(pose)
        self._record_dropped(dropped_poses, dropped_events, "overload_dropped")
        self._record_outcome(getattr(pose, "timestamp_s", None), "accepted", None)
        return True

    def wait_and_pop_pose(self) -> Optional[PoseT]:
        pose, superseded = self._inbox.wait_and_pop_latest(
            self._stop_event,
            _is_source_discontinuity,
        )
        for stale_pose in superseded:
            self._record_outcome(
                getattr(stale_pose, "timestamp_s", None),
                "source_superseded",
                None,
            )
        return pose

    def reserve_publication(
        self,
        token: FrameGeneration,
        source_timestamp_s: float,
    ) -> Optional[PublicationSlot]:
        slot = self._publications.reserve(
            token.invalidated,
            self._stop_event,
        )
        if slot is None and self._publications.source_driven:
            outcome = (
                "source_stopped_dropped"
                if self._stop_event.is_set()
                else "source_invalidated_dropped"
            )
            self._record_outcome(source_timestamp_s, outcome, None)
        return slot

    @contextmanager
    def commit(self, token: FrameGeneration) -> Iterator[bool]:
        with self._gate.commit(token) as accepted:
            yield accepted and not self._stop_event.is_set()

    def abandon_publication(self, slot: PublicationSlot) -> None:
        self._publications.abandon(slot)

    def reject_publication(
        self,
        slot: PublicationSlot,
        source_timestamp_s: float,
    ) -> None:
        self._publications.abandon(slot)
        if self._publications.source_driven:
            outcome = (
                "source_stopped_dropped"
                if self._stop_event.is_set()
                else "source_invalidated_dropped"
            )
            self._record_outcome(source_timestamp_s, outcome, None)

    def publish_detection(
        self,
        slot: PublicationSlot,
        targets: Sequence[DetectedObject],
        *,
        source_timestamp_s: float,
        source_receipt_timestamp_s: Optional[float],
        source_name: Optional[str],
        source_discontinuity: Optional[bool],
    ) -> bool:
        return self._publications.publish(
            slot,
            targets,
            primary_target=select_most_centered_target(targets),
            source_timestamp_s=source_timestamp_s,
            source_receipt_timestamp_s=source_receipt_timestamp_s,
            source_name=source_name,
            source_discontinuity=source_discontinuity,
        )

    @contextmanager
    def reset(self) -> Iterator[bool]:
        dropped_poses = []
        dropped_events = []
        with self._reset_lock:
            ticket = self._gate.begin_reset()
            if ticket is None:
                yield False
                return
            dropped_poses = self._inbox.clear()
            dropped_events = self._publications.clear(
                mark_discontinuity=True,
            )
            try:
                yield True
            except Exception:
                raise
            else:
                self._gate.finish_reset(ticket)
            finally:
                self._record_dropped(
                    dropped_poses,
                    dropped_events,
                    "source_reset_dropped",
                )

    def stop(self) -> None:
        self._stop_event.set()
        self._gate.stop()
        self._inbox.wake()
        self._publications.wake()
        with self._reset_lock:
            poses = self._inbox.clear()
            events = self._publications.stop()
        self._record_dropped(poses, events, "source_stopped_dropped")

    def _record_dropped(
        self,
        poses: Sequence[PoseT],
        publications: Sequence[DetectionPublication],
        outcome: str,
    ) -> None:
        for pose in poses:
            self._record_outcome(getattr(pose, "timestamp_s", None), outcome, None)
        for publication in publications:
            self._record_outcome(publication.source_timestamp_s, outcome, None)


def _is_source_discontinuity(pose: object) -> bool:
    return bool(getattr(pose, "source_discontinuity", False))


__all__ = ["SourceFrameCoordinator"]
