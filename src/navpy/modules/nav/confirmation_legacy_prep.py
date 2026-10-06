"""Legacy geo localization and navigation hold before operator review."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from navpy.modules.common.models.location import Location
from navpy.modules.navigation.approach_strategy import ApproachKind
from navpy.modules.navigation.peer_offset import OFFSET_LOITER_RADIUS_M
from navpy.modules.nav.nav_state import NavigationTaskState
from navpy.modules.nav.vehicle_navigation import LoiterRadiusLease
from navpy.modules.vision.models.detect_data import DetectedObject


@dataclass(frozen=True)
class LegacyReviewPorts:
    ground_location: Callable[..., Location | None]
    current_location: Callable[[], Location | None]
    goto_poi: Callable[[Location], None]
    goto_loiter: Callable[[Location, float, float | None], None]
    absolute_location: Callable[[Location | None], Location | None]


class LegacyReviewPreparation:
    """Prepare only the legacy geo-driven review path."""

    def __init__(
        self,
        ports: LegacyReviewPorts,
        navigation_task: NavigationTaskState,
        loiter_radius: LoiterRadiusLease,
        approach_kind: ApproachKind,
    ) -> None:
        self._ports = ports
        self._navigation_task = navigation_task
        self._loiter_radius = loiter_radius
        self._approach_kind = approach_kind

    def prepare(self, poi: DetectedObject) -> None:
        if self._navigation_task.orbit_radius_m <= 0:
            self._hold_current_location()
        poi_geo = poi.geo.projected_poi_location
        if poi_geo is None:
            poi_geo = self._ports.ground_location(poi)
            if poi_geo is not None:
                poi.set_p_t_g_loc(poi_geo)
        if (
            self._navigation_task.navigation_poi_location is None
            and poi_geo is not None
        ):
            self._navigation_task.navigation_poi_location = (
                self._ports.absolute_location(poi_geo)
            )

    def _hold_current_location(self) -> None:
        current = self._ports.current_location()
        if current is None:
            return
        if self._approach_kind == ApproachKind.OFFSET:
            if self._loiter_radius.acquire():
                self._ports.goto_loiter(
                    current,
                    OFFSET_LOITER_RADIUS_M,
                    None,
                )
            else:
                self._ports.goto_poi(current)
        else:
            self._ports.goto_poi(current)


__all__ = ["LegacyReviewPorts", "LegacyReviewPreparation"]
