"""Tracking-loop, visual recovery, and gimbal measurement delivery."""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np

from navpy.logger.cache_logger import ILogger
from navpy.modules.vision.appearance import AsyncAppearanceEmbedder
from navpy.modules.vision.geometry import cxcywh_to_xyxy
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.multi_object_tracker import TrackedObject
from navpy.modules.vision.lost_target_bridge import LostTargetBridge
from navpy.modules.vision.real_detector_state import (
    DetectionBatchInbox,
    DetectionResultStore,
    DetectorRunState,
    LoopTiming,
    RuntimeMetrics,
    FreshnessPolicy,
    next_loop_deadline,
)
from navpy.modules.vision.real_detector_ports import GimbalMeasurementPort
from navpy.modules.vision.real_tracking_batch import (
    TrackingBatchProcessor,
    TrackingModels,
    TrackingPublications,
)
from navpy.modules.vision.target_lock import TargetLock
from navpy.modules.vision.target_priority import find_target_by_id
from navpy.modules.vision.track_identity import TrackIdentityResolver


class TrackingNavigationSink:
    """Deliver each fresh pixel measurement once, otherwise explicitly coast."""

    def __init__(
        self,
        navigation: GimbalMeasurementPort | None,
        freshness: FreshnessPolicy,
    ) -> None:
        self._navigation = navigation
        self._freshness = freshness
        self._last_measurement_timestamp: float | None = None

    def update(self, targets: Sequence[DetectedObject]) -> None:
        if self._navigation is None:
            return
        tracking_id = self._navigation.tracking_obj_id
        if tracking_id is None:
            return
        tracked = find_target_by_id(targets, tracking_id)
        if tracked is not None and not self._freshness.is_fresh(tracked):
            tracked = None
        timestamp = (
            None
            if tracked is None
            else tracked.timing.detection_timestamp_s
        )
        if timestamp is not None and timestamp == self._last_measurement_timestamp:
            return
        self._navigation.update(tracked)
        if timestamp is not None:
            self._last_measurement_timestamp = timestamp


class TrackingRecovery:
    """Visual re-identification and short-gap recovery for the selected target."""

    def __init__(
            self,
            bridge: LostTargetBridge,
            appearance: AsyncAppearanceEmbedder | None,
            identity: TrackIdentityResolver,
            target_lock: TargetLock,
            use_target_lock: bool,
            metrics: RuntimeMetrics,
    ) -> None:
        self._bridge = bridge
        self._appearance = appearance
        self._identity = identity
        self._target_lock = target_lock
        self._use_target_lock = bool(use_target_lock)
        self._metrics = metrics

    def bridge_locked_target(
            self,
            raw_tracks: Sequence[TrackedObject],
            frame: np.ndarray,
            frame_width: int,
            frame_height: int,
            now_s: float,
    ) -> None:
        if not self._use_target_lock:
            return
        locked_id = self._target_lock.locked_id
        if locked_id is None:
            self._bridge.reset()
            return
        present = next(
            (
                track for track in raw_tracks
                if self._identity.stable_of(int(track.id)) == locked_id
            ),
            None,
        )
        if present is not None:
            self._bridge.observe(
                frame, (present.cx, present.cy, present.w, present.h), now_s,
            )
            return
        hit = self._bridge.search(frame, now_s)
        if hit is not None:
            self._identity.hint_position(
                locked_id,
                hit.cx / max(frame_width, 1),
                hit.cy / max(frame_height, 1),
                now_s,
            )
            self._metrics.bump("bridge_hits")

    def compute_embeddings(
            self,
            tracks: Sequence[TrackedObject],
            frame: np.ndarray | None,
    ) -> dict[int, np.ndarray] | None:
        if self._appearance is None or frame is None or not tracks:
            return None
        boxes = [
            cxcywh_to_xyxy((track.cx, track.cy, track.w, track.h))
            for track in tracks
        ]
        keys = [int(track.id) for track in tracks]
        self._appearance.submit(frame, boxes, keys)
        latest = self._appearance.latest(dict(zip(keys, boxes)))
        return latest or None


class TrackingLoop:
    """Periodic consumer that distinguishes coasting from an empty inference."""

    def __init__(
            self,
            run_state: DetectorRunState,
            period_s: float,
            inbox: DetectionBatchInbox,
            results: DetectionResultStore,
            processor: TrackingBatchProcessor,
            navigation: TrackingNavigationSink,
            metrics: RuntimeMetrics,
            logger: ILogger,
            timing: LoopTiming = LoopTiming(),
    ) -> None:
        self._run_state = run_state
        self._period_s = float(period_s)
        self._inbox = inbox
        self._results = results
        self._processor = processor
        self._navigation = navigation
        self._metrics = metrics
        self._logger = logger
        self._timing = timing

    def run(self) -> None:
        next_at = self._timing.monotonic()
        while self._run_state.is_running:
            now_s = self._timing.monotonic()
            sleep_s = next_at - now_s
            if sleep_s > 0.0:
                self._timing.sleep(sleep_s)
                continue
            next_at = next_loop_deadline(next_at, self._period_s, now_s)
            batch = self._inbox.take_next()
            self._metrics.bump("track_ticks")
            if not batch.is_new:
                self._coast()
                self._metrics.update_fps()
                continue
            if batch.association is None:
                self._metrics.update_fps()
                continue
            try:
                self._processor.process(batch, now_s)
                self._metrics.update_fps()
            except Exception as error:
                self._logger.error(f"Track/update error: {error}")

    def _coast(self) -> None:
        self._metrics.bump("coast_ticks")
        targets, _ = self._results.snapshot()
        self._navigation.update(targets)


__all__ = [
    "TrackingBatchProcessor",
    "TrackingNavigationSink",
    "TrackingLoop",
    "TrackingModels",
    "TrackingPublications",
    "TrackingRecovery",
]
