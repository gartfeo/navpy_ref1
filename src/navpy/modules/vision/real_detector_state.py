"""Result, overlay, confirmation-frame, and freshness state."""

from __future__ import annotations

import threading
import time
from collections.abc import Sequence

import numpy as np

from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.multi_object_tracker import TrackedObject
from navpy.modules.vision.real_detector_inference_state import (
    DeepSearchInbox,
    DetectionBatch,
    DetectionBatchInbox,
    InferenceGeneration,
    InferenceReservation,
)
from navpy.modules.vision.real_detector_ports import TimestampedVisionItem
from navpy.modules.vision.real_detector_runtime_state import (
    DetectorRunState,
    LoopTiming,
    PipelineMutationGate,
    RuntimeMetrics,
    next_loop_deadline,
)


ConfirmationFrame = tuple[
    np.ndarray,
    tuple[float, float, float, float],
    float,
]


class DetectionResultStore:
    """Atomic owner of the published target list and primary target."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._targets: list[DetectedObject] = []
        self._primary: DetectedObject | None = None

    def publish(
        self,
        targets: Sequence[DetectedObject],
        primary: DetectedObject | None,
    ) -> None:
        with self._lock:
            self._targets = list(targets)
            self._primary = primary

    def snapshot(self) -> tuple[list[DetectedObject], DetectedObject | None]:
        with self._lock:
            return list(self._targets), self._primary

    def clear(self) -> None:
        self.publish([], None)


class OverlayStore:
    """Atomic owner of tracks and the corresponding selected track."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._tracks: list[TrackedObject] = []
        self._locked: TrackedObject | None = None

    def publish(
        self,
        tracks: Sequence[TrackedObject],
        locked: TrackedObject | None,
    ) -> None:
        with self._lock:
            self._tracks = list(tracks)
            self._locked = locked

    def snapshot(self) -> tuple[list[TrackedObject], TrackedObject | None]:
        with self._lock:
            return list(self._tracks), self._locked


class ConfirmationFrameStore:
    """Own paired confirmation images/bboxes and bounded eviction."""

    def __init__(self, maximum_frames: int = 32) -> None:
        self._lock = threading.Lock()
        self._frames: dict[int, ConfirmationFrame] = {}
        self._maximum_frames = int(maximum_frames)

    def get(self, obj_id: int) -> ConfirmationFrame | None:
        with self._lock:
            return self._frames.get(obj_id)

    def put(
        self,
        obj_id: int,
        frame: np.ndarray,
        bbox: tuple[float, float, float, float],
        center_score: float,
    ) -> None:
        stored = (frame.copy(), bbox, center_score)
        with self._lock:
            self._frames[obj_id] = stored

    def evict_except(self, live_ids: set[int]) -> None:
        with self._lock:
            for obj_id in [item for item in self._frames if item not in live_ids]:
                del self._frames[obj_id]
            while len(self._frames) > self._maximum_frames:
                del self._frames[next(iter(self._frames))]

    def clear(self) -> None:
        with self._lock:
            self._frames.clear()


class FreshnessPolicy:
    """Single cached-target freshness rule shared by all readers."""

    def __init__(self, track_period_s: float) -> None:
        self._maximum_age_s = max(0.25, 3.0 * float(track_period_s))

    @property
    def maximum_age_s(self) -> float:
        return self._maximum_age_s

    def is_fresh(
        self,
        target: TimestampedVisionItem,
        now_s: float | None = None,
    ) -> bool:
        timestamp = (
            target.timing.detection_timestamp_s
            if isinstance(target, DetectedObject)
            else target.timestamp
        )
        if timestamp is None:
            return True
        current = time.time() if now_s is None else float(now_s)
        return current - float(timestamp) <= self._maximum_age_s


__all__ = [
    "ConfirmationFrameStore",
    "DeepSearchInbox",
    "DetectionBatch",
    "DetectionBatchInbox",
    "DetectionResultStore",
    "DetectorRunState",
    "FreshnessPolicy",
    "InferenceGeneration",
    "InferenceReservation",
    "LoopTiming",
    "OverlayStore",
    "PipelineMutationGate",
    "RuntimeMetrics",
    "next_loop_deadline",
]
