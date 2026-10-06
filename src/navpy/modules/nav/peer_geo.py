"""Peer-target geo pointing and acquisition diagnostics."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Callable, Optional

import numpy as np
import pymap3d

from navpy.args.logger_args import LogStatusDest
from navpy.logger.cache_logger import ILogger
from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.geo.geo_ref_calc import GeoRefCalc
from navpy.modules.nav.nav_state import NavigationTaskState, GeoHoldState
from navpy.modules.vision.detector_ports import GeoPointingPort, MountCatalogPort
from navpy.modules.vision.vision_class_profile import (
    get_class_detect_size,
    get_min_pixels_for_class,
)

if TYPE_CHECKING:
    from navpy.modules.vision.camera_mount import CameraMount


@dataclass(frozen=True)
class PeerGeoAcquisitionState:
    class_id: int
    range_h_m: float
    range_v_m: float
    slant_m: float
    uv: tuple[Optional[float], Optional[float]]
    projected_px: float
    min_pixels: float
    reason: str
    in_frame: bool


class PeerGeoTracker:
    """Arm, prime, and stop the detector's geo-follow capability."""

    def __init__(
        self,
        geo_pointing: GeoPointingPort,
        geo_hold: GeoHoldState,
        geo_ref: GeoRefCalc,
        current_location: Callable[[], Optional[Location]],
        current_attitude: Callable[[], Attitude | None],
        logger: ILogger,
    ) -> None:
        self._geo_pointing = geo_pointing
        self._geo_hold = geo_hold
        self._geo_ref = geo_ref
        self._current_location = current_location
        self._current_attitude = current_attitude
        self._logger = logger

    def start(self, target: Location) -> bool:
        self._geo_hold.target_location = target
        try:
            self._geo_pointing.start_geo_tracking(target, self._geo_ref)
        except Exception as error:  # noqa: BLE001 - graceful degradation
            self._logger.warning(
                f"Peer-nav geo arming failed for target={target}: {error}",
                key="nav",
            )
            self._geo_hold.target_location = None
            self._geo_hold.acquisition_log_bucket = None
            return False
        return True

    def prime(self) -> bool:
        target = self._geo_hold.target_location
        if target is None:
            return False
        location = self._current_location()
        attitude = self._current_attitude()
        if location is None or attitude is None:
            return True
        try:
            self._geo_pointing.update_geo(location, attitude)
        except Exception as error:  # noqa: BLE001 - dispatch still proceeds
            self._logger.warning(
                f"Peer-nav geo priming update failed for target={target}: {error}",
                key="nav",
            )
            self.stop()
            return False
        self._logger.info(
            f"Peer-nav geo priming update issued for target={target}",
            key="nav",
        )
        return True

    def stop(self) -> None:
        target = self._geo_hold.target_location
        try:
            self._geo_pointing.stop_geo_tracking()
        except Exception as error:  # noqa: BLE001 - best effort teardown
            self._logger.warning(
                f"Peer-nav geo teardown failed for target={target}: {error}",
                key="nav",
            )
        self._geo_hold.target_location = None
        self._geo_hold.acquisition_log_bucket = None


class PeerGeoAcquisition:
    """Compute and report one diagnostic geo-projection snapshot."""

    def __init__(
        self,
        mounts: MountCatalogPort,
        geo_ref: GeoRefCalc,
        vision_profile: dict,
        geo_hold: GeoHoldState,
        navigation_task: NavigationTaskState,
        selected_target: Callable[[], object],
        logger: ILogger,
    ) -> None:
        self._mounts = mounts
        self._geo_ref = geo_ref
        self._vision_profile = vision_profile
        self._geo_hold = geo_hold
        self._navigation_task = navigation_task
        self._selected_target = selected_target
        self._logger = logger

    def snapshot(
        self,
        uav_location: Location,
        uav_attitude: Attitude,
        target_location: Optional[Location] = None,
        class_id: Optional[int] = None,
    ) -> Optional[PeerGeoAcquisitionState]:
        target = target_location or self._geo_hold.target_location
        if target is None or uav_location is None or uav_attitude is None:
            return None
        if not self._mounts.mounts:
            return None
        mount = self._mounts.mounts[0]
        try:
            k = mount.get_k()
            gimbal = mount.get_gimbal_data()
            ned = pymap3d.geodetic2ned(
                target.lat,
                target.lng,
                target.alt,
                uav_location.lat,
                uav_location.lng,
                uav_location.alt,
            )
            range_h_m = float(np.hypot(ned[0], ned[1]))
            range_v_m = float(abs(ned[2]))
            slant_m = float(np.linalg.norm(ned))
            uv = self._geo_ref.calc_uv(ned, k, gimbal, uav_attitude)
            resolved_class_id = self._class_id(class_id)
            min_pixels = get_min_pixels_for_class(
                self._vision_profile,
                resolved_class_id,
            )
            projected_px = (
                float(k[1][1])
                * get_class_detect_size(resolved_class_id)
                / max(slant_m, 1.0)
            )
            reason, in_frame = self._frame_state(mount, uv, projected_px, min_pixels)
            return PeerGeoAcquisitionState(
                resolved_class_id,
                range_h_m,
                range_v_m,
                slant_m,
                uv,
                projected_px,
                float(min_pixels),
                reason,
                in_frame,
            )
        except Exception as error:  # noqa: BLE001 - diagnostic only
            self._logger.warning(
                f"Peer geo acquisition diagnostic failed for target={target}: {error}",
                key="peer_geo_acq_warn",
            )
            return None

    def log(self, state: PeerGeoAcquisitionState) -> None:
        bucket = (state.reason,)
        if self._geo_hold.acquisition_log_bucket == bucket:
            return
        self._geo_hold.acquisition_log_bucket = bucket
        label = "PEER_GEO_ACQ_OK" if state.reason == "acquired" else "PEER_GEO_ACQ"
        self._logger.info(
            f"{label}: target={self._geo_hold.target_location} "
            f"range_h={state.range_h_m:.0f}m range_v={state.range_v_m:.0f}m "
            f"slant={state.slant_m:.0f}m "
            f"orbit_r={self._navigation_task.orbit_radius_m:.0f}m "
            f"uv={self.format_uv(state.uv)} proj={state.projected_px:.1f}px "
            f"min={state.min_pixels:.1f}px reason={state.reason}",
            key="peer_geo_acq",
            dest=LogStatusDest.DRONE,
        )

    @staticmethod
    def format_uv(uv: tuple[Optional[float], Optional[float]]) -> str:
        if uv[0] is None or uv[1] is None:
            return "(None,None)"
        return f"({uv[0]:.0f},{uv[1]:.0f})"

    def _class_id(self, explicit: Optional[int]) -> int:
        if explicit is not None:
            return explicit
        selected = self._selected_target()
        return getattr(selected, "class_id", 0) if selected is not None else 0

    @staticmethod
    def _frame_state(
        mount: CameraMount,
        uv: tuple[float | None, float | None],
        projected_px: float,
        min_pixels: float,
    ) -> tuple[str, bool]:
        if uv[0] is None or uv[1] is None:
            return "behind_cam", False
        if not mount.is_valid(uv[0], uv[1]):
            return "out_of_fov", False
        if projected_px < min_pixels:
            return "too_small", True
        return "acquired", True


__all__ = [
    "PeerGeoAcquisition",
    "PeerGeoAcquisitionState",
    "PeerGeoTracker",
]
