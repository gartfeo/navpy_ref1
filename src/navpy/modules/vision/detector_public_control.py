"""Stateless public tracking and geo capability facets for Detector."""

from __future__ import annotations

from typing import TYPE_CHECKING

from navpy.modules.vision.real_detector_public_ports import (
    RealGeoParts,
    RealIdentityParts,
    RealTrackingParts,
)

if TYPE_CHECKING:
    from navpy.modules.common.models.attitude import Attitude
    from navpy.modules.common.models.location import Location
    from navpy.modules.navigation.geo.geo_ref_calc import GeoRefCalc
    from navpy.modules.navigation.gimbal_navigation import GimbalNavigation
    from navpy.modules.vision.camera_mount import CameraMount
    from navpy.modules.vision.gimbal_rate_types import GimbalTrackResult
    from navpy.modules.vision.poi_zoom_types import ZoomTrackResult


class DetectorIdentityFacet:
    _parts: RealIdentityParts

    @property
    def mount(self) -> CameraMount:
        return self._parts.mount

    @property
    def mounts(self) -> list[CameraMount]:
        return [self._parts.mount]

    @property
    def source_name(self) -> str:
        return self._parts.identity.source_name

    @property
    def navigation(self) -> GimbalNavigation | None:
        return self._parts.compatibility_navigation


class DetectorTrackingFacet:
    _parts: RealTrackingParts

    @property
    def rate_result(self) -> GimbalTrackResult | None:
        return self._parts.tracking.rate_result

    @property
    def is_zoom_stable(self) -> bool:
        return self._parts.tracking.is_zoom_stable

    def get_zoom_result(
        self,
        obj_id: int | None = None,
    ) -> ZoomTrackResult | None:
        return self._parts.tracking.get_zoom_result(obj_id)

    def get_zoom_target_pixels(self, class_id: object) -> float | None:
        return self._parts.tracking.get_zoom_target_pixels(class_id)

    def set_zoom_size_demand(self, enabled: bool) -> None:
        self._parts.tracking.set_zoom_size_demand(enabled)

    def freeze_final_approach_zoom_at_min(self) -> bool:
        return self._parts.tracking.freeze_final_approach_zoom_at_min()

    def start_tracking(self, obj_id: int) -> None:
        self._parts.tracking.start_tracking(obj_id)

    def stop_tracking(self, to_neutral: bool = True) -> None:
        self._parts.tracking.stop_tracking(to_neutral)


class DetectorGeoFacet:
    _parts: RealGeoParts

    def start_geo_tracking(
        self,
        poi_loc: Location,
        geo_ref: GeoRefCalc,
    ) -> None:
        self._parts.geo.start_geo_tracking(poi_loc, geo_ref)

    def update_geo(self, uav_loc: Location, uav_att: Attitude) -> None:
        self._parts.geo.update_geo(uav_loc, uav_att)

    def prepare_geo_acquisition(
        self,
        uav_loc: Location,
        uav_att: Attitude,
        class_id: int,
        min_pixels: float,
    ) -> bool:
        return self._parts.geo.prepare_geo_acquisition(
            uav_loc,
            uav_att,
            class_id,
            min_pixels,
        )

    def stop_geo_tracking(self) -> None:
        self._parts.geo.stop_geo_tracking()

    @property
    def is_geo_armed(self) -> bool:
        return self._parts.geo.is_geo_armed

    @property
    def is_detection_armed(self) -> bool:
        return self._parts.geo.is_detection_armed

    @property
    def loss_hold_sec(self) -> float | None:
        return self._parts.geo.loss_hold_sec


__all__ = [
    "DetectorGeoFacet",
    "DetectorIdentityFacet",
    "DetectorTrackingFacet",
]
