"""Polling and source-publication aggregation for detector fleets."""

from __future__ import annotations

import math
from collections.abc import Callable
from numbers import Real
from typing import TYPE_CHECKING, Protocol, Sequence

from navpy.modules.vision.detection_identity_registry import (
    DetectionIdentityRegistry,
)
from navpy.modules.vision.models.detection_publication import DetectionPublication
from navpy.modules.vision.models.detection_event_lease import (
    DetectionEventLease,
    DetectionLeaseDispatch,
)
from navpy.modules.vision.models.detect_response import DetectResponse
from navpy.modules.vision.target_priority import select_most_centered_target

if TYPE_CHECKING:
    from navpy.modules.vision.detector_ports import (
        DetectionEventPort,
        DetectionSnapshotPort,
    )
    from navpy.modules.vision.models.detect_data import DetectedObject
    from navpy.modules.vision.models.detect_request import DetectRequest


class DetectionMember(Protocol):
    """Child capability intersection used only by this aggregation owner."""

    @property
    def has_source_driven_detection_events(self) -> bool: ...

    def get_detect_data(self, request: "DetectRequest") -> DetectResponse: ...

    def drain_detection_events(
        self,
        request: "DetectRequest",
    ) -> list[DetectionPublication]: ...

    def open_detection_event_lease(
        self,
        request: "DetectRequest",
        reset_handler: Callable[[], None] | None = None,
    ) -> DetectionEventLease | None: ...


class NormalizedDetectionEventLease:
    """Apply fleet identity normalization to one exclusive child stream."""

    def __init__(
        self,
        lease: DetectionEventLease,
        aggregator: "DetectionAggregator",
    ) -> None:
        self._lease = lease
        self._aggregator = aggregator

    @property
    def closed(self) -> bool:
        return self._lease.closed

    def wait_and_dispatch(
        self,
        handler: Callable[[DetectionPublication], bool],
    ) -> bool | None:
        return self._lease.wait_and_dispatch(
            lambda publication: handler(
                self._aggregator.normalize_publication(publication)
            )
        )

    def dispatch_available(
        self,
        handler: Callable[[DetectionPublication], bool],
    ) -> DetectionLeaseDispatch:
        return self._lease.dispatch_available(
            lambda publication: handler(
                self._aggregator.normalize_publication(publication)
            )
        )

    def close(self) -> None:
        self._lease.close()


class DetectionAggregator:
    """Normalize child snapshots/publications without owning command routing."""

    def __init__(
            self,
            detectors: Sequence["DetectionMember"],
            registry: DetectionIdentityRegistry,
    ) -> None:
        self._detectors = tuple(detectors)
        self._registry = registry

    @property
    def has_source_driven_detection_events(self) -> bool:
        return any(
            detector.has_source_driven_detection_events is True
            for detector in self._detectors
        )

    def target_uses_source_driven_events(
            self,
            target: "DetectedObject",
    ) -> bool | None:
        return self._registry.target_uses_source_driven_events(target)

    def get_detect_data(self, request: "DetectRequest") -> DetectResponse:
        targets: list["DetectedObject"] = []
        primary_candidates: list["DetectedObject"] = []
        for detector in self._detectors:
            response = detector.get_detect_data(request)
            self._registry.remember_publication_mode(
                response.detected_targets,
                source_driven=detector.has_source_driven_detection_events,
            )
            targets.extend(response.detected_targets)
            if response.primary_target is not None:
                primary_candidates.append(response.primary_target)
        return self.build_response(targets, primary_candidates)

    def drain_detection_events(
            self,
            request: "DetectRequest",
    ) -> list[DetectionPublication]:
        child_events = []
        source_children = [
            detector for detector in self._detectors
            if detector.has_source_driven_detection_events is True
        ]
        for child_index, detector in enumerate(source_children):
            for event_index, publication in enumerate(
                    detector.drain_detection_events(request)
            ):
                if not isinstance(publication, DetectionPublication):
                    continue
                self._registry.remember_publication_mode(
                    publication.detected_targets,
                    source_driven=True,
                )
                child_events.append((
                    self.event_timestamp(publication),
                    child_index,
                    event_index,
                    publication,
                ))
        if len(source_children) > 1:
            child_events.sort(key=lambda event: (
                event[0] is None,
                event[0] if event[0] is not None else 0.0,
                event[1],
                event[2],
            ))
        return [
            self.normalize_publication(publication)
            for _, _, _, publication in child_events
        ]

    def open_detection_event_lease(
        self,
        request: "DetectRequest",
        reset_handler: Callable[[], None] | None = None,
    ) -> DetectionEventLease | None:
        source_children = [
            detector
            for detector in self._detectors
            if detector.has_source_driven_detection_events is True
        ]
        if not source_children:
            return None
        if len(source_children) != 1:
            raise RuntimeError(
                "terminal source lease requires exactly one source-driven detector"
            )
        lease = source_children[0].open_detection_event_lease(
            request,
            reset_handler,
        )
        return (
            None
            if lease is None
            else NormalizedDetectionEventLease(lease, self)
        )

    def normalize_publication(
        self,
        publication: DetectionPublication,
    ) -> DetectionPublication:
        self._registry.remember_publication_mode(
            publication.detected_targets,
            source_driven=True,
        )
        return publication.with_targets(self.normalize_targets(
            list(publication.detected_targets),
            [publication.primary_target]
            if publication.primary_target is not None else [],
        ))

    def build_response(
            self,
            targets: list["DetectedObject"],
            primary_candidates: list["DetectedObject"],
    ) -> DetectResponse:
        ordered_targets = self.normalize_targets(targets, primary_candidates)
        return DetectResponse(
            ordered_targets,
            primary_target=ordered_targets[0] if ordered_targets else None,
        )

    def normalize_targets(
            self,
            targets: list["DetectedObject"],
            primary_candidates: list["DetectedObject"],
    ) -> list["DetectedObject"]:
        return self._registry.normalize_targets(
            targets,
            primary_candidates,
            select_primary=select_most_centered_target,
        )

    @staticmethod
    def event_timestamp(publication: DetectionPublication) -> float | None:
        source_timestamp_s = publication.source_timestamp_s
        if (
                isinstance(source_timestamp_s, Real)
                and math.isfinite(float(source_timestamp_s))
        ):
            return float(source_timestamp_s)
        timestamps = [
            float(target.timing.detection_timestamp_s)
            for target in publication.detected_targets
            if isinstance(target.timing.detection_timestamp_s, Real)
            and math.isfinite(float(target.timing.detection_timestamp_s))
        ]
        return min(timestamps) if timestamps else None
