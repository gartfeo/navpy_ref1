"""Segregated public capabilities for the gimbal-navigation facade."""

from __future__ import annotations

from typing import Protocol

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.navigation.gimbal_navigation_state import GimbalNavigationStatus
from navpy.modules.navigation.gimbal_tracking_constants import ARMED_ANY_OBJ_ID
from navpy.modules.navigation.gimbal_zoom_control import GimbalZoomController
from navpy.modules.vision.gimbal_rate_types import GimbalTrackResult
from navpy.modules.vision.target_zoom_types import ZoomTrackResult


class GimbalStatusParts(Protocol):
    status: GimbalNavigationStatus


class DetectionOwner(Protocol):
    def start(self, obj_id: int) -> None: ...
    def stop(self, to_neutral: bool = True) -> None: ...


class GimbalDetectionParts(Protocol):
    detection: DetectionOwner


class GeoOwner(Protocol):
    def start(self, target_loc: object, geo_ref: object) -> None: ...
    def stop(self) -> None: ...
    def update(self, uav_loc: object, uav_att: Attitude) -> None: ...
    def prepare_acquisition(
        self,
        uav_loc: object,
        uav_att: Attitude,
        class_id: int,
        min_pixels: float,
    ) -> bool: ...


class GimbalGeoParts(Protocol):
    geo: GeoOwner


class VisualOwner(Protocol):
    def update(
        self,
        target: object,
        now: float | None = None,
        principal_point: tuple[float, float] | None = None,
    ) -> None: ...


class GimbalVisualParts(Protocol):
    visual: VisualOwner


class GimbalZoomParts(Protocol):
    zoom: GimbalZoomController


class GimbalNavigationStatusFacet:
    _parts: GimbalStatusParts

    @property
    def status(self) -> GimbalNavigationStatus:
        return self._parts.status

    @property
    def tracking_obj_id(self) -> int | None:
        return self.status.detection.tracking_obj_id

    @property
    def is_detection_armed(self) -> bool:
        return self.tracking_obj_id is not None

    @property
    def is_geo_armed(self) -> bool:
        return self.status.geo.target is not None

    @property
    def rate_result(self) -> GimbalTrackResult | None:
        return self.status.rate_result

    @property
    def zoom_result(self) -> ZoomTrackResult | None:
        return self.status.zoom_result

    @property
    def is_zoom_stable(self) -> bool:
        return self.status.zoom_is_stable

    def zoom_target_pixels(self, class_id: object) -> float | None:
        return self.status.zoom_target_pixels(class_id)

    @property
    def loss_hold_sec(self) -> float:
        return self.status.loss_hold_sec


class GimbalDetectionFacet:
    _parts: GimbalDetectionParts

    def start_tracking(self, obj_id: int) -> None:
        self._parts.detection.start(obj_id)

    def arm(self) -> None:
        self.start_tracking(ARMED_ANY_OBJ_ID)

    def stop_tracking(self, to_neutral: bool = True) -> None:
        self._parts.detection.stop(to_neutral)


class GimbalGeoFacet:
    _parts: GimbalGeoParts

    def start_geo_tracking(self, target_loc: object, geo_ref: object) -> None:
        self._parts.geo.start(target_loc, geo_ref)

    def stop_geo_tracking(self) -> None:
        self._parts.geo.stop()

    def update_geo(self, uav_loc: object, uav_att: Attitude) -> None:
        self._parts.geo.update(uav_loc, uav_att)

    def prepare_geo_acquisition(
        self,
        uav_loc: object,
        uav_att: Attitude,
        class_id: int,
        min_pixels: float,
    ) -> bool:
        return self._parts.geo.prepare_acquisition(
            uav_loc,
            uav_att,
            class_id,
            min_pixels,
        )


class GimbalVisualFacet:
    _parts: GimbalVisualParts

    def update(
        self,
        target: object,
        now: float | None = None,
        principal_point: tuple[float, float] | None = None,
    ) -> None:
        self._parts.visual.update(target, now, principal_point)


class GimbalZoomFacet:
    _parts: GimbalZoomParts

    def set_zoom_size_demand(self, enabled: bool) -> None:
        self._parts.zoom.set_size_demand(enabled)

    def freeze_terminal_zoom_at_min(self) -> bool:
        return self._parts.zoom.freeze_at_min()


__all__ = [
    "GimbalDetectionFacet",
    "GimbalGeoFacet",
    "GimbalNavigationStatusFacet",
    "GimbalVisualFacet",
    "GimbalZoomFacet",
]
