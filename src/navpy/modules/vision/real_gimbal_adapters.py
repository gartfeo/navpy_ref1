"""Narrow adapters over the broad gimbal-navigation compatibility object."""

from __future__ import annotations

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.geo.geo_ref_calc import GeoRefCalc
from navpy.modules.navigation.gimbal_navigation import GimbalNavigation
from navpy.modules.vision.gimbal_rate_types import GimbalTrackResult
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.target_zoom_types import ZoomTrackResult


class GimbalTrackingAdapter:
    def __init__(self, navigation: GimbalNavigation) -> None:
        self._navigation = navigation

    @property
    def tracking_obj_id(self) -> int | None:
        return self._navigation.tracking_obj_id

    @property
    def rate_result(self) -> GimbalTrackResult | None:
        return self._navigation.rate_result

    @property
    def is_zoom_stable(self) -> bool:
        return self._navigation.is_zoom_stable

    @property
    def zoom_result(self) -> ZoomTrackResult | None:
        return self._navigation.zoom_result

    def zoom_target_pixels(self, class_id: object) -> float | None:
        return self._navigation.zoom_target_pixels(class_id)

    def set_zoom_size_demand(self, enabled: bool) -> None:
        self._navigation.set_zoom_size_demand(enabled)

    def freeze_terminal_zoom_at_min(self) -> bool:
        return self._navigation.freeze_terminal_zoom_at_min()

    def start_tracking(self, obj_id: int) -> None:
        self._navigation.start_tracking(obj_id)

    def stop_tracking(self, to_neutral: bool = True) -> None:
        self._navigation.stop_tracking(to_neutral=to_neutral)


class GimbalGeoAdapter:
    def __init__(self, navigation: GimbalNavigation) -> None:
        self._navigation = navigation

    def start_geo_tracking(
        self,
        target_loc: Location,
        geo_ref: GeoRefCalc,
    ) -> None:
        self._navigation.start_geo_tracking(target_loc, geo_ref)

    def update_geo(self, uav_loc: Location, uav_att: Attitude) -> None:
        self._navigation.update_geo(uav_loc, uav_att)

    def prepare_geo_acquisition(
        self,
        uav_loc: Location,
        uav_att: Attitude,
        class_id: int,
        min_pixels: float,
    ) -> bool:
        return self._navigation.prepare_geo_acquisition(
            uav_loc,
            uav_att,
            class_id,
            min_pixels,
        )

    def stop_geo_tracking(self) -> None:
        self._navigation.stop_geo_tracking()

    @property
    def is_geo_armed(self) -> bool:
        return self._navigation.is_geo_armed

    @property
    def is_detection_armed(self) -> bool:
        return self._navigation.is_detection_armed

    @property
    def loss_hold_sec(self) -> float | None:
        return self._navigation.loss_hold_sec


class GimbalMeasurementAdapter:
    def __init__(self, navigation: GimbalNavigation) -> None:
        self._navigation = navigation

    @property
    def tracking_obj_id(self) -> int | None:
        return self._navigation.tracking_obj_id

    def update(self, target: DetectedObject | None) -> None:
        self._navigation.update(target)


__all__ = [
    "GimbalGeoAdapter",
    "GimbalMeasurementAdapter",
    "GimbalTrackingAdapter",
]
