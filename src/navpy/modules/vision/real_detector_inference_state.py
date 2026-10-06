"""Atomic inference inboxes and generation fencing."""

from __future__ import annotations

import threading
from collections.abc import Iterable
from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from navpy.modules.vision.real_frame_association import RealFrameAssociation
    from navpy.modules.vision.yolo_detector import Detection


@dataclass(frozen=True)
class DetectionBatch:
    is_new: bool
    detections: tuple[Detection, ...]
    association: RealFrameAssociation | None
    generation: int | None


class DetectionBatchInbox:
    """Single-slot detection/association exchange with sequence semantics."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._detections: list[Detection] = []
        self._association: RealFrameAssociation | None = None
        self._generation: int | None = None
        self._published_sequence = 0
        self._consumed_sequence = -1

    def publish(
        self,
        detections: Iterable[Detection],
        association: RealFrameAssociation,
        generation: int,
    ) -> None:
        with self._lock:
            self._detections = list(detections)
            self._association = association
            self._generation = int(generation)
            self._published_sequence += 1

    def take_next(self) -> DetectionBatch:
        with self._lock:
            is_new = self._published_sequence != self._consumed_sequence
            detections = tuple(self._detections) if is_new else ()
            association = self._association if is_new else None
            generation = self._generation if is_new else None
            self._consumed_sequence = self._published_sequence
        return DetectionBatch(is_new, detections, association, generation)

    def clear(self) -> None:
        with self._lock:
            self._detections = []
            self._association = None
            self._generation = None
            self._consumed_sequence = self._published_sequence


class DeepSearchInbox:
    """Frame-keyed one-shot exchange for deep-search detections."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._detections: list[Detection] = []
        self._published_at_s = 0.0
        self._frame_sequence: int | None = None

    def publish(
        self,
        detections: Iterable[Detection],
        published_at_s: float,
        frame_sequence: int | None,
    ) -> None:
        with self._lock:
            self._detections = list(detections)
            self._published_at_s = float(published_at_s)
            self._frame_sequence = frame_sequence

    def take(
        self,
        *,
        expected_frame_sequence: int | None,
        stale_seconds: float,
        now_s: float,
    ) -> list[Detection]:
        with self._lock:
            if float(now_s) - self._published_at_s > float(stale_seconds):
                self._clear_locked()
                return []
            if (
                expected_frame_sequence is not None
                and self._frame_sequence != expected_frame_sequence
            ):
                return []
            detections = list(self._detections)
            self._clear_locked()
        return detections

    def clear(self) -> None:
        with self._lock:
            self._clear_locked()

    def _clear_locked(self) -> None:
        self._detections = []
        self._published_at_s = 0.0
        self._frame_sequence = None


@dataclass(frozen=True)
class InferenceReservation:
    generation: int


class InferenceGeneration:
    """Reject publications from inference that crossed a refresh boundary."""

    def __init__(
        self,
        detections: DetectionBatchInbox,
        deep_detections: DeepSearchInbox,
    ) -> None:
        self._lock = threading.Lock()
        self._generation = 0
        self._detections = detections
        self._deep_detections = deep_detections

    def reserve(self) -> InferenceReservation:
        with self._lock:
            return InferenceReservation(self._generation)

    def publish_detection(
        self,
        reservation: InferenceReservation,
        detections: Iterable[Detection],
        association: RealFrameAssociation,
    ) -> bool:
        with self._lock:
            if reservation.generation != self._generation:
                return False
            self._detections.publish(detections, association, self._generation)
            return True

    def is_current(self, generation: int | None) -> bool:
        with self._lock:
            return generation is not None and generation == self._generation

    def publish_deep_search(
        self,
        reservation: InferenceReservation,
        detections: Iterable[Detection],
        published_at_s: float,
        frame_sequence: int | None,
    ) -> bool:
        with self._lock:
            if reservation.generation != self._generation:
                return False
            self._deep_detections.publish(
                detections,
                published_at_s,
                frame_sequence,
            )
            return True

    def take_deep_search(
        self,
        *,
        expected_frame_sequence: int | None,
        stale_seconds: float,
        now_s: float,
    ) -> list[Detection]:
        with self._lock:
            return self._deep_detections.take(
                expected_frame_sequence=expected_frame_sequence,
                stale_seconds=stale_seconds,
                now_s=now_s,
            )

    def clear_deep_search(self) -> None:
        with self._lock:
            self._deep_detections.clear()

    def invalidate(self) -> None:
        with self._lock:
            self._generation += 1
            self._detections.clear()
            self._deep_detections.clear()


__all__ = [
    "DeepSearchInbox",
    "DetectionBatch",
    "DetectionBatchInbox",
    "InferenceGeneration",
    "InferenceReservation",
]
