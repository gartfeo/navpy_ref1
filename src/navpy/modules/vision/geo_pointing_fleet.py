"""Fleet-wide geo-pointing transactions."""

from __future__ import annotations

from typing import TYPE_CHECKING, Protocol, Sequence

from navpy.modules.vision.tracking_command_router import TrackingLogger

if TYPE_CHECKING:
    from navpy.modules.common.models.attitude import Attitude
    from navpy.modules.common.models.location import Location
    from navpy.modules.navigation.geo.geo_ref_calc import GeoRefCalc


class GeoPointingMember(Protocol):
    def start_geo_tracking(self, poi_loc: "Location", geo_ref: "GeoRefCalc") -> None: ...

    def update_geo(self, uav_loc: "Location", uav_att: "Attitude") -> None: ...

    def prepare_geo_acquisition(
        self,
        uav_loc: "Location",
        uav_att: "Attitude",
        class_id: int,
        min_pixels: float,
    ) -> bool: ...

    def stop_geo_tracking(self) -> None: ...

    @property
    def is_geo_armed(self) -> bool: ...


class GeoPointingFleet:
    def __init__(
        self,
        members: Sequence[GeoPointingMember],
        logger: TrackingLogger,
    ) -> None:
        self._members = tuple(members)
        self._logger = logger

    def start_geo_tracking(self, poi_loc: "Location", geo_ref: "GeoRefCalc") -> None:
        started: list[GeoPointingMember] = []
        try:
            for member in self._members:
                member.start_geo_tracking(poi_loc, geo_ref)
                started.append(member)
        except Exception:
            self._rollback(started)
            raise
        self._logger.info(
            f"DetectionCoordinator: start_geo_tracking poi={poi_loc} "
            f"started={len(started)}/{len(self._members)}"
        )

    def update_geo(self, uav_loc: "Location", uav_att: "Attitude") -> None:
        for member in self._members:
            member.update_geo(uav_loc, uav_att)

    def prepare_geo_acquisition(
        self,
        uav_loc: "Location",
        uav_att: "Attitude",
        class_id: int,
        min_pixels: float,
    ) -> bool:
        prepared = False
        for member in self._members:
            prepared = member.prepare_geo_acquisition(
                uav_loc, uav_att, class_id, min_pixels,
            ) or prepared
        return prepared

    def stop_geo_tracking(self) -> None:
        for member in self._members:
            member.stop_geo_tracking()
        self._logger.info("DetectionCoordinator: stop_geo_tracking")

    @property
    def is_geo_armed(self) -> bool:
        return any(member.is_geo_armed for member in self._members)

    def _rollback(self, started: list[GeoPointingMember]) -> None:
        for member in started:
            try:
                member.stop_geo_tracking()
            except Exception as error:
                self._logger.warning(
                    "DetectionCoordinator: rollback "
                    f"stop_geo_tracking failed: {error}"
                )


__all__ = ["GeoPointingFleet"]
