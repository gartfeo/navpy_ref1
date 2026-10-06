"""Legacy geo-assisted target resolution outside the pure-vision path."""

from __future__ import annotations

import threading
from typing import Optional, Protocol

import numpy as np
import pymap3d

from navpy.args.navigation_args import NavigationArgs
from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.geo.geo_ref_calc import GeoRefCalc
from navpy.modules.navigation.geo.rotation_utils import normalize
from navpy.modules.navigation.geo.zc_util import ZcUtil
from navpy.modules.vision.models.detect_data import DetectedObject


class LegacyPoseReader(Protocol):
    @property
    def attitude(self) -> Attitude: ...

    def location(self, is_relative: bool) -> Location | None: ...


def target_ground_location(
    *,
    zc_util: Optional[ZcUtil],
    geo_ref: GeoRefCalc,
    detect_data: DetectedObject,
    target_ned: Optional[np.ndarray],
    allow_fallback: bool,
) -> Optional[Location]:
    """Back-project a legacy detection ray to terrain."""
    if zc_util is None:
        return None
    if target_ned is None:
        target_ned = geo_ref.calc_ned(
            u=detect_data.pixel.u_px,
            v=detect_data.pixel.v_px,
            k=detect_data.optics.camera_matrix(),
            g_data=detect_data.pose.gimbal_data,
            uas_att=detect_data.pose.aircraft_attitude,
        )
    return zc_util.ray_to_terrain_ned(
        detect_data.geo.camera_location,
        target_ned,
        delta_alt=detect_data.geo.reference_height_m,
        allow_fallback=allow_fallback,
    )


def geodetic_target_ned(
    current: Location,
    target: Location,
) -> np.ndarray:
    """Return the normalized geodetic NED direction to a locked target."""
    return normalize(
        np.array(
            pymap3d.geodetic2ned(
                target.lat,
                target.lng,
                target.alt,
                current.lat,
                current.lng,
                current.alt,
            )
        )
    )


def navigation_target_location(
    current: Optional[Location],
    target_ned: Optional[np.ndarray],
    distance: Optional[float],
    fallback: Optional[Location],
) -> Optional[Location]:
    """Project a finite legacy navigation target along a NED direction."""
    if current is None or target_ned is None or distance is None or distance <= 0:
        return fallback if fallback is not None else current
    direction = normalize(target_ned)
    if np.allclose(direction, 0):
        return fallback if fallback is not None else current
    ned_vector = direction * distance
    try:
        latitude, longitude, altitude = pymap3d.ned2geodetic(
            ned_vector[0],
            ned_vector[1],
            ned_vector[2],
            current.lat,
            current.lng,
            current.alt,
        )
    except Exception:
        return fallback if fallback is not None else current
    return Location(latitude, longitude, altitude, is_absolute=True)


class LegacyNavigationState:
    """Own the target lock and one-shot mission adjustment for legacy laws."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._locked_target: Optional[Location] = None
        self._adjusted = False
        self._adjustment_available = True

    def reset(self) -> None:
        with self._lock:
            self._locked_target = None
            self._adjusted = False
            self._adjustment_available = True

    def locked_target(self) -> Optional[Location]:
        with self._lock:
            return self._locked_target

    def set_locked_target(self, target: Optional[Location]) -> None:
        with self._lock:
            self._locked_target = target

    def lock_target_if_empty(self, target: Optional[Location]) -> None:
        if target is None:
            return
        with self._lock:
            if self._locked_target is None:
                self._locked_target = target

    def begin_adjustment(self) -> Optional[Location]:
        with self._lock:
            if not self._adjustment_available or self._locked_target is None:
                return None
            self._adjustment_available = False
            return self._locked_target

    def finish_adjustment(self, adjusted: bool) -> None:
        with self._lock:
            self._adjusted = bool(adjusted)

    def is_adjusted(self) -> bool:
        with self._lock:
            return self._adjusted


class LegacyDestinationResolver:
    """Resolve and retain geo-assisted targets outside pure-vision navigation."""

    def __init__(
        self,
        vehicle: LegacyPoseReader,
        args: NavigationArgs,
        geo_ref: GeoRefCalc,
        zc_util: Optional[ZcUtil],
        state: LegacyNavigationState,
    ) -> None:
        self._vehicle = vehicle
        self._args = args
        self._geo_ref = geo_ref
        self._zc_util = zc_util
        self._state = state

    def resolve(
        self,
        detect_data: Optional[DetectedObject],
    ) -> tuple[Optional[np.ndarray], Optional[Attitude]]:
        if detect_data is None:
            locked_target = self._state.locked_target()
            if locked_target is None:
                return None, None
            current = self._vehicle.location(False)
            return geodetic_target_ned(current, locked_target), self._vehicle.attitude

        if (
            self._args.use_direct_target
            and detect_data.geo.is_simulation
            and detect_data.geo.truth_target_location is not None
        ):
            target = detect_data.geo.truth_target_location
            self._state.set_locked_target(target)
            current = self._vehicle.location(False)
            return geodetic_target_ned(current, target), detect_data.pose.aircraft_attitude

        target_ned = self._geo_ref.calc_ned(
            u=detect_data.pixel.u_px,
            v=detect_data.pixel.v_px,
            k=detect_data.optics.camera_matrix(),
            g_data=detect_data.pose.gimbal_data,
            uas_att=detect_data.pose.aircraft_attitude,
        )
        if self._state.locked_target() is None:
            self._state.lock_target_if_empty(
                self.ground_location(detect_data, target_ned=target_ned)
            )
        return target_ned, detect_data.pose.aircraft_attitude

    def ground_location(
        self,
        detect_data: DetectedObject,
        target_ned: Optional[np.ndarray] = None,
        allow_fallback: bool = True,
    ) -> Optional[Location]:
        return target_ground_location(
            zc_util=self._zc_util,
            geo_ref=self._geo_ref,
            detect_data=detect_data,
            target_ned=target_ned,
            allow_fallback=allow_fallback,
        )

    def locked_distance(self) -> Optional[float]:
        locked_target = self._state.locked_target()
        if locked_target is None:
            return None
        return GeoRefCalc.calculate_distance(
            self._vehicle.location(False),
            locked_target,
        )

    @property
    def geo_ref(self) -> GeoRefCalc:
        return self._geo_ref

    @property
    def locked_target(self) -> Optional[Location]:
        return self._state.locked_target()

    def set_locked_target(self, target: Optional[Location]) -> None:
        self._state.set_locked_target(target)


__all__ = [
    "LegacyNavigationState",
    "LegacyPoseReader",
    "LegacyDestinationResolver",
    "geodetic_target_ned",
    "navigation_target_location",
    "target_ground_location",
]
