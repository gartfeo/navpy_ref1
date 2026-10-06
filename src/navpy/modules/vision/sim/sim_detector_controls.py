"""Focused command and status owners for a simulated detector."""

from __future__ import annotations

from typing import Optional

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.geo.geo_ref_calc import GeoRefCalc
from navpy.modules.vision.camera_mount import CameraMount
from navpy.modules.vision.detector_ports import source_name_from_gimbal
from navpy.modules.vision.gimbal_rate_types import GimbalTrackResult
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.sim.sim_control_ports import (
    GeoNavigationPort,
    TargetProviderMutationPort,
    TrackingNavigationPort,
    ZoomNavigationPort,
)
from navpy.modules.vision.sim.sim_detector_state import SimCaptureState
from navpy.modules.vision.target_zoom_types import ZoomTrackResult


class SimDetectorIdentity:
    """Direct public compatibility adapter; never injected into a leaf."""

    def __init__(self, mount: CameraMount) -> None:
        self._mount = mount

    @property
    def mount(self) -> CameraMount:
        return self._mount

    @property
    def mounts(self) -> list[CameraMount]:
        return [self._mount]

    @property
    def source_name(self) -> str:
        return source_name_from_gimbal(self._mount.get_gimbal_data)

    @property
    def is_simulation(self) -> bool:
        return True


class SimTrackingControls:
    def __init__(
        self,
        navigation: Optional[TrackingNavigationPort],
        capture: SimCaptureState,
    ) -> None:
        self._navigation = navigation
        self._capture = capture

    @property
    def rate_result(self) -> Optional[GimbalTrackResult]:
        return self._navigation.rate_result if self._navigation else None

    @property
    def tracking_obj_id(self) -> Optional[int]:
        return self._navigation.tracking_obj_id if self._navigation else None

    def apply_detection_update(
        self,
        target: Optional[DetectedObject],
        timestamp_s: float,
    ) -> None:
        if self._navigation is not None:
            self._navigation.update(target, now=timestamp_s)

    def start_tracking(self, obj_id: int) -> None:
        capture_was_enabled = self._capture.enabled
        self._capture.enabled = True
        try:
            if self._navigation is not None:
                self._navigation.start_tracking(obj_id)
        except Exception:
            self._capture.enabled = capture_was_enabled
            raise

    def stop_tracking(self, to_neutral: bool = True) -> None:
        if self._navigation is not None:
            self._navigation.stop_tracking(to_neutral=to_neutral)

    @property
    def is_detection_armed(self) -> bool:
        return bool(self._navigation and self._navigation.is_detection_armed)

    @property
    def loss_hold_sec(self) -> Optional[float]:
        return self._navigation.loss_hold_sec if self._navigation else None

    def reset(self) -> None:
        if self._navigation is None:
            return
        self._navigation.stop_geo_tracking()
        self._navigation.stop_tracking(to_neutral=True)


class SimZoomControls:
    def __init__(
        self,
        navigation: Optional[ZoomNavigationPort],
        tracking: SimTrackingControls,
        capture: SimCaptureState,
    ) -> None:
        self._navigation = navigation
        self._tracking = tracking
        self._capture = capture

    @property
    def is_zoom_stable(self) -> bool:
        return self._navigation.is_zoom_stable if self._navigation else True

    def get_zoom_result(
        self,
        obj_id: Optional[int] = None,
    ) -> Optional[ZoomTrackResult]:
        if self._navigation is None:
            return None
        if obj_id is not None and self._tracking.tracking_obj_id != obj_id:
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
        self._capture.enabled = False
        if self._navigation is None:
            return True
        return self._navigation.freeze_terminal_zoom_at_min()


class SimGeoControls:
    def __init__(self, navigation: Optional[GeoNavigationPort]) -> None:
        self._navigation = navigation

    def start_geo_tracking(self, target_loc: Location, geo_ref: GeoRefCalc) -> None:
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
        return bool(self._navigation and self._navigation.is_geo_armed)


class SimTargetControls:
    """Direct public provider adapter; never injected into a leaf."""

    def __init__(self, target_provider: TargetProviderMutationPort) -> None:
        self._target_provider = target_provider

    def set_sim_target(
        self,
        command_index: int,
        location: Location,
        location_type: Optional[str] = None,
    ) -> None:
        self._target_provider.set_sim_target(
            command_index,
            location,
            location_type=location_type,
        )


__all__ = [
    "SimDetectorIdentity",
    "SimGeoControls",
    "SimTargetControls",
    "SimTrackingControls",
    "SimZoomControls",
]
