"""Known-geolocation gimbal capability facade."""

from __future__ import annotations

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.geo.geo_ref_calc import GeoRefCalc
from navpy.modules.navigation.gimbal_geo_acquisition import GeoAcquisitionZoom
from navpy.modules.navigation.gimbal_geo_commander import GimbalGeoCommander
from navpy.modules.navigation.gimbal_geo_lifecycle import GimbalGeoLifecycle


class GimbalGeoTracking:
    """Expose lifecycle, pointing, and acquisition as one narrow capability."""

    def __init__(
        self,
        lifecycle: GimbalGeoLifecycle,
        commander: GimbalGeoCommander,
        acquisition_zoom: GeoAcquisitionZoom,
    ) -> None:
        self._lifecycle = lifecycle
        self._commander = commander
        self._acquisition_zoom = acquisition_zoom

    def start(
        self,
        target_loc: Location | None,
        geo_ref: GeoRefCalc | None,
    ) -> None:
        self._lifecycle.start(target_loc, geo_ref)

    def stop(self) -> None:
        self._lifecycle.stop()

    def update(
        self,
        uav_loc: Location | None,
        uav_att: Attitude | None,
    ) -> None:
        self._commander.update(uav_loc, uav_att)

    def prepare_acquisition(
        self,
        uav_loc: Location | None,
        uav_att: Attitude | None,
        class_id: int,
        min_pixels: float,
    ) -> bool:
        return self._acquisition_zoom.prepare(
            uav_loc,
            uav_att,
            class_id,
            min_pixels,
        )


__all__ = ["GimbalGeoTracking"]
