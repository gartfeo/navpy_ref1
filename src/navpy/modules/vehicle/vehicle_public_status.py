"""Stateless status and position facets for :class:`VehicleMav`."""

from __future__ import annotations

from _thread import RLock as RLockType

from navpy.modules.common.models.location import Location
from navpy.modules.vehicle.flight_mode import FlightMode
from navpy.modules.vehicle.preflight_health import PrearmCheckState
from navpy.modules.vehicle.vehicle_public_ports import (
    HealthParts,
    LimitsModeParts,
    PositionMissionParts,
)


class VehicleLimitsModeFacet:
    """Expose configured attitude limits and current flight mode."""

    _parts: LimitsModeParts

    @property
    def max_pitch(self) -> float:
        return self._parts.parameters.max_pitch

    @property
    def min_pitch(self) -> float:
        return self._parts.parameters.min_pitch

    @property
    def lim_roll(self) -> float:
        return self._parts.parameters.lim_roll

    @property
    def get_mode(self) -> FlightMode | None:
        return self._parts.mode.current


class VehicleHealthFacet:
    """Expose link, pre-arm, sensor, EKF, RC, and arm status."""

    _parts: HealthParts

    @property
    def link_ok(self) -> bool:
        return self._parts.health.link_ok

    @property
    def link_quality(self) -> int:
        return self._parts.health.link_quality

    @property
    def prearm_check_state(self) -> PrearmCheckState:
        return self._parts.health.prearm_check_state

    @property
    def prearm_ok(self) -> bool | None:
        return self._parts.health.prearm_ok

    @property
    def sensor_health(self) -> dict[str, bool | None]:
        return self._parts.health.sensor_health

    @property
    def airspeed_present(self) -> bool | None:
        return self._parts.health.airspeed_present

    @property
    def airspeed_healthy(self) -> bool | None:
        return self._parts.health.airspeed_healthy

    @property
    def ekf_status(self) -> dict[str, int | float] | None:
        return self._parts.health.ekf_status

    @property
    def rc3_raw(self) -> int | None:
        return self._parts.health.rc3_raw

    @property
    def is_armed(self) -> bool:
        return self._parts.health.is_armed


class VehiclePositionMissionViewFacet:
    """Expose position reads and the mission collection view."""

    _parts: PositionMissionParts

    @property
    def home_location(self) -> Location | None:
        return self._parts.position.home_location

    @property
    def mission_items_count(self) -> int:
        return self._parts.mission.count

    @property
    def mission_items_next(self) -> int | None:
        return self._parts.mission.next_seq

    @property
    def mission_lock(self) -> RLockType:
        return self._parts.mission.lock

    def location(self, is_relative: bool) -> Location | None:
        return self._parts.position.location(is_relative)

    def terrain_height_at(
        self,
        lat: float,
        lng: float,
        *,
        timeout: float = 1.0,
    ) -> float | None:
        return self._parts.position.terrain_height_at(lat, lng, timeout=timeout)
