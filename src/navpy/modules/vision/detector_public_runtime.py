"""Stateless runtime, event, query, and simulation facets for Detector."""

from __future__ import annotations

from collections.abc import Callable
from typing import TYPE_CHECKING

import numpy as np

from navpy.modules.vision.models.detect_request import DetectRequest
from navpy.modules.vision.models.detect_response import DetectResponse
from navpy.modules.vision.real_detector_public_ports import (
    RealEventParts,
    RealLifecycleParts,
    RealQueryParts,
    RealSimulationParts,
)

if TYPE_CHECKING:
    from navpy.modules.common.models.location import Location
    from navpy.modules.vision.models.detect_data import DetectedObject
    from navpy.modules.vision.models.detection_publication import (
        DetectionPublication,
    )
    from navpy.modules.vision.models.detection_event_lease import DetectionEventLease


class DetectorLifecycleFacet:
    _parts: RealLifecycleParts

    def start(self) -> None:
        self._parts.lifecycle.start()

    def stop(self) -> bool:
        return self._parts.lifecycle.stop()

    @property
    def is_quiescent(self) -> bool:
        return self._parts.lifecycle.is_quiescent

    def refresh(self) -> None:
        self._parts.reset.refresh()

    def raise_if_failed(self) -> None:
        self._parts.lifecycle.raise_if_failed()

    def wall_period_for_scheduler_period(
        self,
        scheduler_period_s: float,
    ) -> float:
        return self._parts.cadence.wall_period_for_scheduler_period(
            scheduler_period_s
        )


class DetectorEventsFacet:
    _parts: RealEventParts

    @property
    def has_source_driven_detection_events(self) -> bool:
        return self._parts.events.has_source_driven_detection_events

    def poi_uses_source_driven_events(
        self,
        poi: DetectedObject,
    ) -> bool | None:
        return self._parts.events.poi_uses_source_driven_events(poi)

    def drain_detection_events(
        self,
        request: DetectRequest,
    ) -> list[DetectionPublication]:
        return list(self._parts.events.drain_detection_events(request))

    def open_detection_event_lease(
        self,
        request: DetectRequest,
        reset_handler: Callable[[], None] | None = None,
    ) -> DetectionEventLease | None:
        return self._parts.events.open_detection_event_lease(
            request,
            reset_handler,
        )


class DetectorQueryFacet:
    _parts: RealQueryParts

    def push_frame(self, frame: np.ndarray | None) -> None:
        self._parts.lifecycle.push_frame(frame)

    def get_detect_data(self, request: DetectRequest) -> DetectResponse:
        return self._parts.query.get_detect_data(request)


class DetectorSimulationFacet:
    _parts: RealSimulationParts

    @property
    def is_simulation(self) -> bool:
        return self._parts.simulation.is_simulation

    def set_sim_poi(
        self,
        command_index: int,
        location: Location,
        location_type: str | None = None,
    ) -> None:
        self._parts.simulation.set_sim_poi(
            command_index,
            location,
            location_type=location_type,
        )


__all__ = [
    "DetectorEventsFacet",
    "DetectorLifecycleFacet",
    "DetectorQueryFacet",
    "DetectorSimulationFacet",
]
