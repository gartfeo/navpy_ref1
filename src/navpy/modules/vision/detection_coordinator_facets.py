"""Stateless public capability facets for DetectionCoordinator."""

from __future__ import annotations

from collections.abc import Callable
from typing import TYPE_CHECKING

from navpy.modules.vision.detection_coordinator_public_ports import (
    CoordinatorEventParts,
    CoordinatorGeoParts,
    CoordinatorIdentityParts,
    CoordinatorLifecycleParts,
    CoordinatorSimulationParts,
    CoordinatorTrackingParts,
    CoordinatorZoomParts,
)
from navpy.modules.vision.detection_coordination import DetectionCoordination
from navpy.modules.vision.detector_ports import DetectorFleetMember

if TYPE_CHECKING:
    from navpy.modules.common.models.attitude import Attitude
    from navpy.modules.common.models.location import Location
    from navpy.modules.navigation.geo.geo_ref_calc import GeoRefCalc
    from navpy.modules.vision.camera_mount import CameraMount
    from navpy.modules.vision.models.detect_data import DetectedObject
    from navpy.modules.vision.models.detection_event_lease import DetectionEventLease
    from navpy.modules.vision.models.detect_request import DetectRequest
    from navpy.modules.vision.models.detect_response import DetectResponse
    from navpy.modules.vision.models.detection_publication import (
        DetectionPublication,
    )
    from navpy.modules.vision.poi_zoom_types import ZoomTrackResult


class CoordinatorIdentityFacet:
    _parts: CoordinatorIdentityParts

    @property
    def detectors(self) -> list[DetectorFleetMember]:
        return list(self._parts.lifecycle.members)

    @property
    def mounts(self) -> list[CameraMount]:
        return list(self._parts.mounts.mounts)


class CoordinatorCompositionFacet:
    """Application-root access to the complete Nav composition bundle."""

    _parts: DetectionCoordination

    @property
    def coordination(self) -> DetectionCoordination:
        return self._parts


class CoordinatorEventsFacet:
    _parts: CoordinatorEventParts

    def get_detect_data(self, request: DetectRequest) -> DetectResponse:
        return self._parts.snapshot.get_detect_data(request)

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


class CoordinatorLifecycleFacet:
    _parts: CoordinatorLifecycleParts

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


class CoordinatorSimulationFacet:
    _parts: CoordinatorSimulationParts

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


class CoordinatorTrackingFacet:
    _parts: CoordinatorTrackingParts

    def start_tracking(self, obj_id: int) -> None:
        self._parts.tracking_commands.start_tracking(obj_id)

    def stop_tracking(self, to_neutral: bool = True) -> None:
        self._parts.tracking_commands.stop_tracking(to_neutral)

    @property
    def is_detection_armed(self) -> bool:
        return self._parts.tracking_status.is_detection_armed

    @property
    def loss_hold_sec(self) -> float:
        return self._parts.tracking_status.loss_hold_sec

    def rebind_task_id(self, task_id: int, poi: DetectedObject) -> bool:
        return self._parts.poi_identity.rebind_task_id(task_id, poi)


class CoordinatorGeoFacet:
    _parts: CoordinatorGeoParts

    def start_geo_tracking(
        self,
        poi_loc: Location,
        geo_ref: GeoRefCalc,
    ) -> None:
        self._parts.geo_pointing.start_geo_tracking(poi_loc, geo_ref)

    def update_geo(self, uav_loc: Location, uav_att: Attitude) -> None:
        self._parts.geo_pointing.update_geo(uav_loc, uav_att)

    def prepare_geo_acquisition(
        self,
        uav_loc: Location,
        uav_att: Attitude,
        class_id: int,
        min_pixels: float,
    ) -> bool:
        return self._parts.geo_pointing.prepare_geo_acquisition(
            uav_loc,
            uav_att,
            class_id,
            min_pixels,
        )

    def stop_geo_tracking(self) -> None:
        self._parts.geo_pointing.stop_geo_tracking()

    @property
    def is_geo_armed(self) -> bool:
        return self._parts.geo_pointing.is_geo_armed


class CoordinatorZoomFacet:
    _parts: CoordinatorZoomParts

    @property
    def is_zoom_stable(self) -> bool:
        return self._parts.zoom.is_zoom_stable

    def get_zoom_result(
        self,
        obj_id: int | None = None,
    ) -> ZoomTrackResult | None:
        return self._parts.zoom.get_zoom_result(obj_id)

    def set_zoom_size_demand(self, enabled: bool) -> None:
        self._parts.zoom.set_zoom_size_demand(enabled)

    def freeze_final_approach_zoom_at_min(self) -> bool:
        return self._parts.zoom.freeze_final_approach_zoom_at_min()


__all__ = [
    "CoordinatorCompositionFacet",
    "CoordinatorEventsFacet",
    "CoordinatorGeoFacet",
    "CoordinatorIdentityFacet",
    "CoordinatorLifecycleFacet",
    "CoordinatorSimulationFacet",
    "CoordinatorTrackingFacet",
    "CoordinatorZoomFacet",
]
