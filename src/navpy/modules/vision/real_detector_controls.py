"""Public command, query, and reset owners for real detection."""

from __future__ import annotations

import time

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.geo.geo_ref_calc import GeoRefCalc
from navpy.modules.vision.gimbal_rate_types import GimbalTrackResult
from navpy.modules.vision.lost_target_bridge import LostTargetBridge
from navpy.modules.vision.models.detect_request import DetectRequest
from navpy.modules.vision.models.detect_response import DetectResponse
from navpy.modules.vision.real_detector_diagnostics import (
    DetectorDebugConfig,
    DetectorDiagnostics,
)
from navpy.modules.vision.real_detector_ports import (
    GimbalGeoCommandPort,
    GimbalTrackingCommandPort,
)
from navpy.modules.vision.real_detector_state import (
    ConfirmationFrameStore,
    DetectionResultStore,
    DetectorRunState,
    FreshnessPolicy,
    InferenceGeneration,
    PipelineMutationGate,
)
from navpy.modules.vision.target_lock import TargetLock
from navpy.modules.vision.target_priority import prioritize_targets
from navpy.modules.vision.target_zoom_types import ZoomTrackResult
from navpy.modules.vision.track_identity import TrackIdentityResolver
from navpy.modules.vision.tracker_backends import TrackerBackend


class DetectorTrackingControl:
    """Own target-lock and gimbal detection/zoom command forwarding."""

    def __init__(
        self,
        navigation: GimbalTrackingCommandPort | None,
        target_lock: TargetLock,
    ) -> None:
        self._navigation = navigation
        self._target_lock = target_lock

    @property
    def rate_result(self) -> GimbalTrackResult | None:
        return self._navigation.rate_result if self._navigation else None

    @property
    def is_zoom_stable(self) -> bool:
        return self._navigation.is_zoom_stable if self._navigation else True

    def get_zoom_result(self, obj_id: int | None = None) -> ZoomTrackResult | None:
        if self._navigation is None:
            return None
        if obj_id is not None and self._navigation.tracking_obj_id != obj_id:
            return None
        return self._navigation.zoom_result

    def get_zoom_target_pixels(self, class_id: object) -> float | None:
        if self._navigation is None:
            return None
        return self._navigation.zoom_target_pixels(class_id)

    def set_zoom_size_demand(self, enabled: bool) -> None:
        if self._navigation is not None:
            self._navigation.set_zoom_size_demand(enabled)

    def freeze_terminal_zoom_at_min(self) -> bool:
        if self._navigation is None:
            return True
        return self._navigation.freeze_terminal_zoom_at_min()

    def start_tracking(self, obj_id: int) -> None:
        if obj_id < 0:
            raise ValueError(
                f"Detector.start_tracking requires obj_id >= 0, got {obj_id}"
            )
        self._target_lock.force_lock(obj_id)
        if self._navigation is None:
            return
        try:
            self._navigation.start_tracking(obj_id)
        except Exception:
            self._target_lock.reset()
            raise

    def stop_tracking(self, to_neutral: bool = True) -> None:
        self._target_lock.reset()
        if self._navigation is not None:
            self._navigation.stop_tracking(to_neutral=to_neutral)


class DetectorGeoControl:
    """Own geo-pointing command forwarding and status reads."""

    def __init__(self, navigation: GimbalGeoCommandPort | None) -> None:
        self._navigation = navigation

    def start_geo_tracking(
        self,
        target_loc: Location,
        geo_ref: GeoRefCalc,
    ) -> None:
        if self._navigation is not None:
            self._navigation.start_geo_tracking(target_loc, geo_ref)

    def update_geo(self, uav_loc: Location, uav_att: Attitude) -> None:
        if self._navigation is not None:
            self._navigation.update_geo(uav_loc, uav_att)

    def prepare_geo_acquisition(
        self,
        uav_loc: Location,
        uav_att: Attitude,
        class_id: int,
        min_pixels: float,
    ) -> bool:
        if self._navigation is None:
            return False
        return self._navigation.prepare_geo_acquisition(
            uav_loc,
            uav_att,
            class_id,
            min_pixels,
        )

    def stop_geo_tracking(self) -> None:
        if self._navigation is not None:
            self._navigation.stop_geo_tracking()

    @property
    def is_geo_armed(self) -> bool:
        return self._navigation.is_geo_armed if self._navigation is not None else False

    @property
    def is_detection_armed(self) -> bool:
        return (
            self._navigation.is_detection_armed
            if self._navigation is not None
            else False
        )

    @property
    def loss_hold_sec(self) -> float | None:
        return self._navigation.loss_hold_sec if self._navigation is not None else None


class DetectionQuery:
    """Apply lock requests and return a freshness-filtered result snapshot."""

    def __init__(
        self,
        results: DetectionResultStore,
        freshness: FreshnessPolicy,
        target_lock: TargetLock,
        use_target_lock: bool,
    ) -> None:
        self._results = results
        self._freshness = freshness
        self._target_lock = target_lock
        self._use_target_lock = bool(use_target_lock)

    def get_detect_data(self, request: DetectRequest) -> DetectResponse:
        if request.force_lock_id is not None and self._use_target_lock:
            self._target_lock.force_lock(request.force_lock_id)
        if request.force_lock_bbox_cxcywh is not None and self._use_target_lock:
            self._target_lock.force_lock_bbox(request.force_lock_bbox_cxcywh)
        targets, primary = self._results.snapshot()
        now_s = time.time()
        targets = [
            target
            for target in targets
            if self._freshness.is_fresh(target, now_s)
        ]
        if primary is not None and not self._freshness.is_fresh(primary, now_s):
            primary = None
        ordered = prioritize_targets(targets, primary)
        return DetectResponse(ordered, primary_target=primary)


class DetectorResetController:
    """Reset every stateful pipeline owner in the established order."""

    def __init__(
        self,
        run_state: DetectorRunState,
        results: DetectionResultStore,
        inference_generation: InferenceGeneration,
        confirmation_frames: ConfirmationFrameStore,
        tracker: TrackerBackend,
        identity: TrackIdentityResolver,
        target_lock: TargetLock,
        bridge: LostTargetBridge,
        mutation_gate: PipelineMutationGate,
    ) -> None:
        self._run_state = run_state
        self._results = results
        self._inference_generation = inference_generation
        self._confirmation_frames = confirmation_frames
        self._tracker = tracker
        self._identity = identity
        self._target_lock = target_lock
        self._bridge = bridge
        self._mutation_gate = mutation_gate

    def refresh(self) -> None:
        if self._run_state.is_stopped:
            return
        with self._mutation_gate:
            if self._run_state.is_stopped:
                return
            self._results.clear()
            self._confirmation_frames.clear()
            self._tracker.reset()
            self._identity.reset()
            self._target_lock.reset()
            self._bridge.reset()
            self._inference_generation.invalidate()


__all__ = [
    "DetectionQuery",
    "DetectorDebugConfig",
    "DetectorDiagnostics",
    "DetectorGeoControl",
    "DetectorResetController",
    "DetectorTrackingControl",
]
