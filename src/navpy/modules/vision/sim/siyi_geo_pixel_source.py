"""Finite SIYI pixels from one known geo POI for navigation isolation."""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from types import SimpleNamespace

import numpy as np

from navpy.modules.common.models.location import Location
from navpy.modules.common.scheduler_cadence import SchedulerCadence
from navpy.modules.navigation.gimbal_navigation import GimbalNavigation
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from navpy.modules.vehicle.pose_streams import pose_frame_association_max_skew_s
from navpy.modules.vehicle.vehicle_interface import IVehicle
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.sim.finite_poi_projector import FinitePoiProjector
from navpy.modules.vision.sim.pose_associator import PoseAssociator
from navpy.modules.vision.sim.pose_stream_link import PoseStreamLink
from navpy.modules.vision.sim.sim_camera_ports import FrameSize, ProjectionCameraPort
from navpy.modules.vision.sim.siyi_pixel_diagnostics import (
    SiyiPixelSourceMetrics,
    SiyiSightRecorder,
    center_error_px,
    projected_poi_pixels,
)
from navpy.modules.vision.simulation_object import SimulationObject


# This source renders from TELEMETRY pose on purpose: its associated pose also
# feeds tracker.update_geo() and reaches a physical gimbal.set_att(), so
# idealizing it would hide the behaviour this harness exists to exercise. Its
# truth event is therefore the estimated position, and the tuple is its OWN --
# borrowing the direct source's named SIM_STATE, which has no default Plane
# stream and which this source never requests, so the required event pair could
# never complete and acquisition timed out.
SIYI_GEO_POSE_MESSAGE_TYPES = ("ATTITUDE", "GLOBAL_POSITION_INT")


class _SiyiAcquisitionState:
    """Mutable frame/acquisition state, always touched under the source lock."""

    def __init__(self) -> None:
        self.frame = None
        self.latest: DetectedObject | None = None
        self.observed: DetectedObject | None = None
        self.active = False
        self.ready = False
        self.acquisition_zoom_key: str | None = None
        self.source_now_s = 0.0


class SiyiGeoPixelSource:
    """Use geo only for SIYI pointing; publish only finite-camera pixels."""

    def __init__(
        self,
        vehicle: IVehicle,
        poi: Location,
        mount: object,
        tracker: GimbalNavigation,
        geo_ref: object,
        cadence: SchedulerCadence,
        *,
        min_pixels: float,
        deliver: Callable[[DetectedObject], bool],
        wall_now_s: Callable[[], float] = time.time,
    ) -> None:
        self._poi = SimulationObject(1, poi, 2)
        self._mount = mount
        self._tracker = tracker
        self._min_pixels = float(min_pixels)
        self._deliver = deliver
        self._wall_now_s = wall_now_s
        self._lock = threading.RLock()
        self._state = _SiyiAcquisitionState()
        self._sight = SiyiSightRecorder(geo_ref, self._poi.g_loc)
        self._stream = PoseStreamLink(vehicle)
        scheduler_period_s = pose_frame_association_max_skew_s(
            self._stream.rate_hz
        )
        self._associator = PoseAssociator(
            attitude_sample=lambda: vehicle.attitude_sample,
            truth_pose=lambda: _render_pose(vehicle),
            air_speed=lambda: vehicle.air_speed,
            maximum_receipt_skew_s=lambda: cadence.wall_period_for_scheduler_period(
                scheduler_period_s
            ),
            require_event_pair=True,
        )
        frame_size = FrameSize.with_defaults(mount.image_width, mount.image_height)
        self._projector = FinitePoiProjector(
            ProjectionCameraPort(
                self._read_matrix,
                self._read_gimbal,
                frame_size,
                mount.is_valid,
            ),
            geo_ref.calc_uv,
            lambda: (self._poi,),
            self.source_now,
        )

    def start(self) -> None:
        self._stream.start(self._on_message, SIYI_GEO_POSE_MESSAGE_TYPES)

    @property
    def ready(self) -> bool:
        with self._lock:
            return self._state.ready

    def activate(self) -> None:
        with self._lock:
            if not self._state.ready:
                raise RuntimeError("SIYI pixel source activated before acquisition")
            self._state.active = True

    @property
    def latest_detection(self) -> DetectedObject | None:
        with self._lock:
            return self._state.observed

    def dispatch_available(self) -> bool:
        with self._lock:
            poi, self._state.latest = self._state.latest, None
        if poi is None:
            return False
        delivered = bool(self._deliver(poi))
        with self._lock:
            self._sight.record_delivery(delivered)
        return delivered

    @property
    def metrics(self) -> SiyiPixelSourceMetrics:
        with self._lock:
            return self._sight.snapshot()

    def source_now(self) -> float:
        with self._lock:
            return self._state.source_now_s

    def close(self) -> None:
        with self._lock:
            self._state.active = False
            self._state.latest = None
        self._stream.close()

    def _on_message(self, message: object) -> None:
        try:
            self._associator.note_event(str(message.get_type()))
        except Exception:
            return
        associated = self._associator.associate()
        if associated is None:
            return
        location = associated.truth_pose.location
        attitude = associated.truth_pose.attitude
        self._tracker.update_geo(location, attitude)
        frame = self._mount.capture_frame_state(
            int(self._mount.image_width), int(self._mount.image_height)
        )
        with self._lock:
            if frame is not None:
                self._sight.record_frame_pair(
                    frame.gimbal_data,
                    attitude,
                    self._wall_now_s,
                )
            self._state.source_now_s = associated.attitude_timestamp_s
            self._state.frame = frame
            result = None if frame is None else self._projector.detect(
                location,
                self._poi,
                attitude,
                timestamp_s=associated.attitude_timestamp_s,
                uas_body_rates_rad_s=associated.attitude_sample.body_rates_rad_s,
            )
            detection = None if result is None else result.poi
            if detection is None:
                if self._state.active:
                    self._sight.record_sight_loss(location)
                return
            self._sight.reset_sight_loss_run()
            self._sight.record_visual_truth_error(detection, location, attitude)
            projected_px = projected_poi_pixels(
                detection, location, self._poi.g_loc
            )
            centered = center_error_px(detection) < 5.0
            commanded_zoom = self._tracker.status.geo.zoom_key
            acquired_now = (
                centered
                and projected_px >= self._min_pixels
                and commanded_zoom is not None
                and _zoom_matches(self._mount, commanded_zoom)
            )
            if acquired_now and self._state.acquisition_zoom_key is None:
                self._state.acquisition_zoom_key = commanded_zoom
                self._state.ready = True
            # Scoped to the navigation task, like the loss counter above.
            # scripts/siyi_pixel_pn_child.py publishes this as
            # projected_frames, and eval_observation_freshness divides
            # delivered_frames by it. Deliveries only happen from
            # dispatch_available() once active, so a frame rendered while the
            # source was still acquiring can never reach the numerator; adding
            # it to the denominator alone reports an accurate run as host
            # contention.
            if self._state.active:
                self._sight.record_visible()
            receipt_s = max(
                associated.attitude_receipt_s,
                associated.truth_receipt_s,
            )
            detection.record_receipt(
                receipt_s,
                self._wall_now_s,
                associated.air_speed_mps,
            )
            self._associator.commit(associated)
            self._state.observed = detection
            if self._state.active:
                self._state.latest = detection
        self._tracker.prepare_geo_acquisition(
            location, attitude, 0, self._min_pixels
        )

    def _read_matrix(self) -> np.ndarray:
        if self._state.frame is None:
            raise RuntimeError("SIYI frame unavailable")
        return self._state.frame.k

    def _read_gimbal(self) -> GimbalData:
        if self._state.frame is None:
            raise RuntimeError("SIYI frame unavailable")
        return self._state.frame.gimbal_data


def _render_pose(vehicle: IVehicle) -> object | None:
    location = vehicle.location(False)
    attitude = vehicle.attitude_sample
    if location is None or attitude is None:
        return None
    return SimpleNamespace(
        location=location,
        attitude=attitude.attitude,
        receipt_time_s=attitude.receipt_time_s,
    )


def _zoom_matches(mount: object, command: str) -> bool:
    try:
        return abs(float(command) - float(mount.gimbal.get_zoom_level())) < 0.1
    except (AttributeError, TypeError, ValueError):
        return False


__all__ = [
    "SIYI_GEO_POSE_MESSAGE_TYPES",
    "SiyiGeoPixelSource",
    "SiyiPixelSourceMetrics",
]
