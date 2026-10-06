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
from navpy.modules.vision.poi_priority import select_most_centered_poi

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

    def poi_uses_source_driven_events(
            self,
            poi: "DetectedObject",
    ) -> bool | None:
        return self._registry.poi_uses_source_driven_events(poi)

    def get_detect_data(self, request: "DetectRequest") -> DetectResponse:
        pois: list["DetectedObject"] = []
        primary_candidates: list["DetectedObject"] = []
        for detector in self._detectors:
            response = detector.get_detect_data(request)
            self._registry.remember_publication_mode(
                response.detected_pois,
                source_driven=detector.has_source_driven_detection_events,
            )
            pois.extend(response.detected_pois)
            if response.primary_poi is not None:
                primary_candidates.append(response.primary_poi)
        return self.build_response(pois, primary_candidates)

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
                    publication.detected_pois,
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
                "final-approach source lease requires exactly one source-driven detector"
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
            publication.detected_pois,
            source_driven=True,
        )
        return publication.with_pois(self.normalize_pois(
            list(publication.detected_pois),
            [publication.primary_poi]
            if publication.primary_poi is not None else [],
        ))

    def build_response(
            self,
            pois: list["DetectedObject"],
            primary_candidates: list["DetectedObject"],
    ) -> DetectResponse:
        ordered_pois = self.normalize_pois(pois, primary_candidates)
        return DetectResponse(
            ordered_pois,
            primary_poi=ordered_pois[0] if ordered_pois else None,
        )

    def normalize_pois(
            self,
            pois: list["DetectedObject"],
            primary_candidates: list["DetectedObject"],
    ) -> list["DetectedObject"]:
        return self._registry.normalize_pois(
            pois,
            primary_candidates,
            select_primary=select_most_centered_poi,
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
            float(poi.timing.detection_timestamp_s)
            for poi in publication.detected_pois
            if isinstance(poi.timing.detection_timestamp_s, Real)
            and math.isfinite(float(poi.timing.detection_timestamp_s))
        ]
        return min(timestamps) if timestamps else None
