"""A detector fleet member that forwards every call to the instance it holds now.

The stable face of a detector restarted as a new instance (app/perception/restarting_detector.py): the coordinator,
the fleet and NAV keep this object; the instance behind it changes. Each call reads the instance once, so a call
in flight during a swap finishes on the instance it started on.

What an instance returns is stamped with its generation (DetectionIdentity.source_generation): a restarted
detector's local object ids start again, and the coordinator must not take a new object for an old one.
Generation 0, the first instance, is never stamped: its objects already say 0.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable, Sequence
from typing import TYPE_CHECKING, Protocol

from navpy.core.perception.detector_ports import DetectorFleetMember

if TYPE_CHECKING:
    from navpy.core.kernel.attitude import Attitude
    from navpy.core.kernel.location import Location
    from navpy.core.perception.camera.camera_mount import CameraMount
    from navpy.core.perception.geo.geo_ref_calc import GeoRefCalc
    from navpy.core.perception.zoom.poi_zoom_types import ZoomTrackResult
    from navpy.core.ports.detect_data import DetectedObject
    from navpy.core.ports.detect_request import DetectRequest
    from navpy.core.ports.detect_response import DetectResponse
    from navpy.core.ports.detection_event_lease import DetectionEventLease, DetectionLeaseDispatch
    from navpy.core.ports.detection_publication import DetectionPublication


class DetectorInstanceHolder(Protocol):
    @property
    def instance(self) -> DetectorFleetMember: ...

    @property
    def current(self) -> tuple[DetectorFleetMember, int]:
        """The instance and its generation, read together."""
        ...


def _stamp(pois: Iterable["DetectedObject | None"], generation: int) -> None:
    if generation:
        for poi in pois:
            if poi is not None:
                poi.mark_source_generation(generation)


class GenerationLease:
    """An instance's event lease whose publications carry that instance's generation."""

    def __init__(self, lease: "DetectionEventLease", generation: int) -> None:
        self._lease = lease
        self._generation = generation

    @property
    def closed(self) -> bool:
        return self._lease.closed

    def wait_and_dispatch(self, handler: Callable[["DetectionPublication"], bool]) -> bool | None:
        return self._lease.wait_and_dispatch(self._stamped(handler))

    def dispatch_available(self, handler: Callable[["DetectionPublication"], bool]) -> "DetectionLeaseDispatch":
        return self._lease.dispatch_available(self._stamped(handler))

    def close(self) -> None:
        self._lease.close()

    def _stamped(
        self, handler: Callable[["DetectionPublication"], bool],
    ) -> Callable[["DetectionPublication"], bool]:
        def stamped(publication: "DetectionPublication") -> bool:
            _stamp(publication.detected_pois, self._generation)
            return handler(publication)
        return stamped


class DetectorQueryDelegateFacet:
    """Detections and source events of the instance held now."""

    _parts: DetectorInstanceHolder

    def get_detect_data(self, request: "DetectRequest") -> "DetectResponse":
        instance, generation = self._parts.current
        response = instance.get_detect_data(request)
        _stamp((*response.detected_pois, response.primary_poi), generation)
        return response

    @property
    def has_source_driven_detection_events(self) -> bool:
        return self._parts.instance.has_source_driven_detection_events

    def drain_detection_events(self, request: "DetectRequest") -> Sequence["DetectionPublication"]:
        instance, generation = self._parts.current
        publications = instance.drain_detection_events(request)
        for publication in publications:
            _stamp(publication.detected_pois, generation)
        return publications

    def open_detection_event_lease(
        self,
        request: "DetectRequest",
        reset_handler: Callable[[], None] | None = None,
    ) -> "DetectionEventLease | None":
        # A lease belongs to the instance that opened it: a restart closes it, and its reader sees it closed.
        instance, generation = self._parts.current
        lease = instance.open_detection_event_lease(request, reset_handler)
        return lease if lease is None or not generation else GenerationLease(lease, generation)

    def poi_uses_source_driven_events(self, poi: "DetectedObject") -> bool | None:
        return self._parts.instance.poi_uses_source_driven_events(poi)



class DetectorControlDelegateFacet:
    """Tracking, geo pointing and zoom commands to the instance held now."""

    _parts: DetectorInstanceHolder

    def start_tracking(self, obj_id: int) -> None:
        self._parts.instance.start_tracking(obj_id)

    def stop_tracking(self, to_neutral: bool = True) -> None:
        self._parts.instance.stop_tracking(to_neutral=to_neutral)

    @property
    def is_detection_armed(self) -> bool:
        return self._parts.instance.is_detection_armed

    @property
    def loss_hold_sec(self) -> float | None:
        return self._parts.instance.loss_hold_sec

    def start_geo_tracking(self, poi_loc: "Location", geo_ref: "GeoRefCalc") -> None:
        self._parts.instance.start_geo_tracking(poi_loc, geo_ref)

    def update_geo(self, uav_loc: "Location", uav_att: "Attitude") -> None:
        self._parts.instance.update_geo(uav_loc, uav_att)

    def prepare_geo_acquisition(
        self,
        uav_loc: "Location",
        uav_att: "Attitude",
        class_id: int,
        min_pixels: float,
    ) -> bool:
        return self._parts.instance.prepare_geo_acquisition(uav_loc, uav_att, class_id, min_pixels)

    def stop_geo_tracking(self) -> None:
        self._parts.instance.stop_geo_tracking()

    @property
    def is_geo_armed(self) -> bool:
        return self._parts.instance.is_geo_armed

    @property
    def is_zoom_stable(self) -> bool:
        return self._parts.instance.is_zoom_stable

    def get_zoom_result(self, obj_id: int | None = None) -> "ZoomTrackResult | None":
        return self._parts.instance.get_zoom_result(obj_id)

    def set_zoom_size_demand(self, enabled: bool) -> None:
        self._parts.instance.set_zoom_size_demand(enabled)

    def freeze_final_approach_zoom_at_min(self) -> bool:
        return self._parts.instance.freeze_final_approach_zoom_at_min()



class DetectorPlatformDelegateFacet:
    """Mounts, simulation, reset, cadence and identity of the instance held now."""

    _parts: DetectorInstanceHolder

    @property
    def mounts(self) -> Sequence["CameraMount"]:
        return self._parts.instance.mounts

    @property
    def is_simulation(self) -> bool:
        return self._parts.instance.is_simulation

    def set_sim_poi(
        self,
        command_index: int,
        location: "Location",
        location_type: str | None = None,
    ) -> None:
        self._parts.instance.set_sim_poi(command_index, location, location_type=location_type)

    def refresh(self) -> None:
        self._parts.instance.refresh()

    def wall_period_for_scheduler_period(self, scheduler_period_s: float) -> float:
        return self._parts.instance.wall_period_for_scheduler_period(scheduler_period_s)

    @property
    def source_name(self) -> str:
        return self._parts.instance.source_name

    @property
    def source_generation(self) -> int:
        """The instance serving now; the coordinator routes an object's commands only to its own instance."""
        return self._parts.current[1]


__all__ = [
    "DetectorControlDelegateFacet",
    "GenerationLease",
    "DetectorInstanceHolder",
    "DetectorPlatformDelegateFacet",
    "DetectorQueryDelegateFacet",
]
