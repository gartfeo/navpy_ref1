"""Segregated navigation and target-control contracts for simulator adapters."""

from __future__ import annotations

from typing import Optional, Protocol, Sequence

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.geo.geo_ref_calc import GeoRefCalc
from navpy.modules.vision.gimbal_rate_types import GimbalTrackResult
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.simulation_object import SimulationObject
from navpy.modules.vision.target_zoom_types import ZoomTrackResult


class TrackingNavigationPort(Protocol):
    @property
    def rate_result(self) -> Optional[GimbalTrackResult]: ...

    @property
    def tracking_obj_id(self) -> Optional[int]: ...

    @property
    def is_detection_armed(self) -> bool: ...

    @property
    def loss_hold_sec(self) -> float: ...

    def start_tracking(self, obj_id: int) -> None: ...

    def stop_tracking(self, to_neutral: bool = True) -> None: ...

    def stop_geo_tracking(self) -> None: ...

    def update(
        self,
        target: Optional[DetectedObject],
        now: Optional[float] = None,
    ) -> None: ...


class ZoomNavigationPort(Protocol):
    @property
    def zoom_result(self) -> Optional[ZoomTrackResult]: ...

    @property
    def is_zoom_stable(self) -> bool: ...

    def zoom_target_pixels(self, class_id: object) -> float | None: ...

    def set_zoom_size_demand(self, enabled: bool) -> None: ...

    def freeze_terminal_zoom_at_min(self) -> bool: ...


class GeoNavigationPort(Protocol):
    @property
    def is_geo_armed(self) -> bool: ...

    def start_geo_tracking(self, target_loc: Location, geo_ref: GeoRefCalc) -> None: ...

    def update_geo(self, uav_loc: Location, uav_att: Attitude) -> None: ...

    def prepare_geo_acquisition(
        self,
        uav_loc: Location,
        uav_att: Attitude,
        class_id: int,
        min_pixels: float,
    ) -> bool: ...

    def stop_geo_tracking(self) -> None: ...


class TargetProviderMutationPort(Protocol):
    @property
    def targets(self) -> Sequence[SimulationObject]: ...

    def refresh(self) -> None: ...

    def set_sim_target(
        self,
        command_index: int,
        location: Location,
        location_type: Optional[str] = None,
    ) -> None: ...


__all__ = [
    "GeoNavigationPort",
    "TargetProviderMutationPort",
    "TrackingNavigationPort",
    "ZoomNavigationPort",
]
