"""Stateless runtime and projection facets for DetectorSim."""

from __future__ import annotations

from collections.abc import Callable

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.models.detect_request import DetectRequest
from navpy.modules.vision.models.detect_response import DetectResponse
from navpy.modules.vision.models.detection_publication import DetectionPublication
from navpy.modules.vision.models.detection_event_lease import DetectionEventLease
from navpy.modules.vision.sim.frame_generation_gate import FrameGeneration
from navpy.modules.vision.sim.sim_detector_public_ports import (
    SimControlParts,
    SimDetectionParts,
    SimLifecycleParts,
    SimProjectionParts,
)
from navpy.modules.vision.simulation_object import SimulationObject


class SimDetectorLifecycleFacet:
    _parts: SimLifecycleParts

    def start(self) -> None:
        self._parts.lifecycle.start()

    def stop(self) -> bool:
        return self._parts.lifecycle.stop()

    @property
    def is_quiescent(self) -> bool:
        return self._parts.lifecycle.is_quiescent

    def refresh(self) -> None:
        self._parts.lifecycle.refresh()

    def raise_if_failed(self) -> None:
        self._parts.lifecycle.raise_if_failed()

    def wall_period_for_scheduler_period(
        self,
        scheduler_period_s: float,
    ) -> float:
        return self._parts.cadence.wall_period_for_scheduler_period(
            scheduler_period_s,
        )


class SimDetectorDetectionFacet:
    _parts: SimDetectionParts

    @property
    def has_source_driven_detection_events(self) -> bool:
        return self._parts.detection.source_driven

    def target_uses_source_driven_events(
        self,
        target: DetectedObject,
    ) -> bool:
        del target
        return self._parts.detection.source_driven

    def drain_detection_events(
        self,
        request: DetectRequest,
    ) -> list[DetectionPublication]:
        return self._parts.detection.drain_detection_events(request)

    def open_detection_event_lease(
        self,
        request: DetectRequest,
        reset_handler: Callable[[], None] | None = None,
    ) -> DetectionEventLease | None:
        if not self._parts.detection.source_driven:
            return None
        return self._parts.detection.open_detection_event_lease(
            request,
            reset_handler,
        )

    def get_detect_data(self, request: DetectRequest) -> DetectResponse:
        return self._parts.detection.get_detect_data(request)

    def get_latest_detections(self) -> list[DetectedObject]:
        return self._parts.detection.get_latest_detections()


class SimDetectorSimulationFacet:
    _parts: SimControlParts

    @property
    def is_simulation(self) -> bool:
        return self._parts.identity.is_simulation

    def set_sim_target(
        self,
        command_index: int,
        location: Location,
        location_type: str | None = None,
    ) -> None:
        self._parts.simulation.set_sim_target(
            command_index,
            location,
            location_type=location_type,
        )


class SimDetectorProjectionFacet:
    _parts: SimProjectionParts

    def update(
        self,
        camera_location: Location,
        target: SimulationObject,
        uas_attitude: Attitude,
        *,
        timestamp_s: float | None = None,
        uas_body_rates_rad_s: tuple[float, float, float] | None = None,
        navigation_attitude: Attitude | None = None,
    ) -> DetectedObject | None:
        return self._parts.projector.update(
            camera_location,
            target,
            uas_attitude,
            timestamp_s=timestamp_s,
            uas_body_rates_rad_s=uas_body_rates_rad_s,
            navigation_attitude=navigation_attitude,
        )

    def detect_targets(
        self,
        camera_location: Location,
        uas_attitude: Attitude,
        attitude_time_boot_s: float | None = None,
        uas_body_rates_rad_s: tuple[float, float, float] | None = None,
        *,
        frame_timestamp_s: float | None = None,
        frame_receipt_timestamp_s: float | None = None,
        frame_air_speed_mps: float | None = None,
        frame_navigation_attitude: Attitude | None = None,
        frame_epoch: int | None = None,
        frame_generation: FrameGeneration | None = None,
        frame_source_discontinuity: bool | None = None,
    ) -> bool:
        return self._parts.renderer.detect_targets(
            camera_location,
            uas_attitude,
            attitude_time_boot_s,
            uas_body_rates_rad_s,
            frame_timestamp_s=frame_timestamp_s,
            frame_receipt_timestamp_s=frame_receipt_timestamp_s,
            frame_air_speed_mps=frame_air_speed_mps,
            frame_navigation_attitude=frame_navigation_attitude,
            frame_epoch=frame_epoch,
            frame_generation=frame_generation,
            frame_source_discontinuity=frame_source_discontinuity,
        )


__all__ = [
    "SimDetectorDetectionFacet",
    "SimDetectorLifecycleFacet",
    "SimDetectorProjectionFacet",
    "SimDetectorSimulationFacet",
]
