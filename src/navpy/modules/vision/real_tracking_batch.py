"""Frame association and result publication for the real tracking pipeline."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from typing import Protocol

import numpy as np

from navpy.modules.vision.deep_search import merge_deep_search_detections
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.multi_object_tracker import TrackedObject
from navpy.modules.vision.real_detector_state import (
    DetectionBatch,
    DetectionResultStore,
    InferenceGeneration,
    OverlayStore,
    PipelineMutationGate,
    RuntimeMetrics,
)
from navpy.modules.vision.real_frame_association import RealFrameAssociation
from navpy.modules.vision.real_inference import DeepSearchChannel
from navpy.modules.vision.real_detection_mapper import DetectedObjectMapper
from navpy.modules.vision.target_lock import TargetLock
from navpy.modules.vision.target_priority import find_target_by_id
from navpy.modules.vision.track_identity import TrackIdentityResolver
from navpy.modules.vision.tracker_backends import TrackerBackend


class TrackingRecoveryPort(Protocol):
    def compute_embeddings(
        self,
        tracks: Sequence[TrackedObject],
        frame: np.ndarray | None,
    ) -> dict[int, np.ndarray] | None: ...

    def bridge_locked_target(
        self,
        tracks: Sequence[TrackedObject],
        frame: np.ndarray,
        frame_width: int,
        frame_height: int,
        now_s: float,
    ) -> None: ...


class TrackingNavigationPort(Protocol):
    def update(self, targets: Sequence[DetectedObject]) -> None: ...


@dataclass(frozen=True)
class TrackingModels:
    tracker: TrackerBackend
    identity: TrackIdentityResolver
    target_lock: TargetLock


@dataclass(frozen=True)
class TrackingPublications:
    results: DetectionResultStore
    overlays: OverlayStore


@dataclass(frozen=True)
class TrackingAssociationResult:
    tracks: tuple[TrackedObject, ...]
    locked: TrackedObject | None
    frame: RealFrameAssociation


class TrackingAssociationProcessor:
    """Admit one frame and resolve stable track identities and target lock."""

    def __init__(
        self,
        models: TrackingModels,
        recovery: TrackingRecoveryPort,
        deep_search: DeepSearchChannel,
        metrics: RuntimeMetrics,
        inference_generation: InferenceGeneration,
        use_target_lock: bool,
    ) -> None:
        self._models = models
        self._recovery = recovery
        self._deep_search = deep_search
        self._metrics = metrics
        self._inference_generation = inference_generation
        self._use_target_lock = bool(use_target_lock)

    def process(
        self,
        batch: DetectionBatch,
        now_s: float,
    ) -> TrackingAssociationResult | None:
        if not self._inference_generation.is_current(batch.generation):
            return None
        association = batch.association
        if association is None:
            return None
        frame = association.frame
        frame_width = association.frame_width
        frame_height = association.frame_height
        if frame is None or frame_width <= 0 or frame_height <= 0:
            return None
        detections = list(batch.detections)
        deep_detections = self._deep_search.take_for_tracking(
            association.frame_sequence,
        )
        if deep_detections:
            detections = merge_deep_search_detections(
                detections,
                deep_detections,
            )
        raw_tracks = self._models.tracker.update(
            detections,
            frame_width,
            frame_height,
            frame=frame,
        )
        embeddings = self._recovery.compute_embeddings(raw_tracks, frame)
        self._recovery.bridge_locked_target(
            raw_tracks,
            frame,
            frame_width,
            frame_height,
            now_s,
        )
        tracks = self._models.identity.update(
            raw_tracks,
            frame_width,
            frame_height,
            now=now_s,
            embeddings=embeddings,
        )
        locked = (
            self._models.target_lock.select(tracks, frame_width, frame_height)
            if self._use_target_lock
            else None
        )
        pinned = (
            self._models.target_lock.locked_id
            if self._use_target_lock
            else None
        )
        self._models.identity.pin(pinned)
        self._metrics.bump("track_updates")
        return TrackingAssociationResult(tuple(tracks), locked, association)


class TrackingResultPublisher:
    """Map accepted tracks and atomically publish all consumer views."""

    def __init__(
        self,
        mapper: DetectedObjectMapper,
        publications: TrackingPublications,
        navigation: TrackingNavigationPort,
    ) -> None:
        self._mapper = mapper
        self._publications = publications
        self._navigation = navigation

    def publish(self, result: TrackingAssociationResult) -> None:
        targets = self._mapper.convert(
            list(result.tracks),
            result.locked,
            result.frame,
        )
        primary = find_target_by_id(
            targets,
            result.locked.id if result.locked is not None else None,
        )
        self._publications.results.publish(targets, primary)
        self._publications.overlays.publish(result.tracks, result.locked)
        self._navigation.update(targets)


class TrackingBatchProcessor:
    """Serialize mutation while delegating association and publication."""

    def __init__(
        self,
        mutation_gate: PipelineMutationGate,
        association: TrackingAssociationProcessor,
        publisher: TrackingResultPublisher,
    ) -> None:
        self._mutation_gate = mutation_gate
        self._association = association
        self._publisher = publisher

    def process(self, batch: DetectionBatch, now_s: float) -> None:
        with self._mutation_gate:
            result = self._association.process(batch, now_s)
            if result is not None:
                self._publisher.publish(result)


__all__ = [
    "TrackingAssociationProcessor",
    "TrackingAssociationResult",
    "TrackingBatchProcessor",
    "TrackingModels",
    "TrackingPublications",
    "TrackingResultPublisher",
]
