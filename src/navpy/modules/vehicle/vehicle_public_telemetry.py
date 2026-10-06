"""Stateless public telemetry facets for :class:`VehicleMav`."""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.wind import Wind
from navpy.modules.vehicle.vehicle_public_ports import (
    IdentityParts,
    MotionParts,
    PoseParts,
    PowerGpsParts,
)


class VehicleIdentityFacet:
    """Expose link identities without owning link state."""

    _parts: IdentityParts

    @property
    def target_system(self) -> int:
        return self._parts.identity.target_system

    @property
    def source_system(self) -> int:
        return self._parts.identity.source_system

    @property
    def transport_source_system(self) -> int | None:
        return self._parts.transport_identity.source_system

    @property
    def transport_source_component(self) -> int | None:
        return self._parts.transport_identity.source_component


class VehiclePowerGpsFacet:
    """Expose power and GPS observations."""

    _parts: PowerGpsParts

    @property
    def battery_level(self) -> int | None:
        return self._parts.power_gps.battery_level

    @property
    def battery_voltage(self) -> float | None:
        return self._parts.power_gps.battery_voltage

    @property
    def battery_current(self) -> float | None:
        return self._parts.power_gps.battery_current

    @property
    def gps_fix_type(self) -> int | None:
        return self._parts.power_gps.gps_fix_type

    @property
    def gps_satellites(self) -> int | None:
        return self._parts.power_gps.gps_satellites

    @property
    def gps_hacc(self) -> float | None:
        return self._parts.power_gps.gps_hacc


class VehicleMotionFacet:
    """Expose vehicle motion observations."""

    _parts: MotionParts

    @property
    def velocity(self) -> tuple[float, float, float] | None:
        return self._parts.flight.velocity

    @property
    def ground_speed(self) -> float | None:
        return self._parts.flight.ground_speed

    @property
    def ground_speed_ned(self) -> np.ndarray | None:
        return self._parts.flight.ground_speed_ned

    @property
    def air_speed(self) -> float | None:
        return self._parts.flight.air_speed

    @property
    def climb_rate(self) -> float | None:
        return self._parts.flight.climb_rate

    @property
    def throttle_pct(self) -> float | None:
        return self._parts.flight.throttle_pct

    @property
    def wind(self) -> Wind | None:
        return self._parts.flight.wind

    @property
    def heading(self) -> float | None:
        return self._parts.flight.heading


class VehiclePoseFacet:
    """Expose attitude, simulation pose, and command diagnostics."""

    _parts: PoseParts

    @property
    def attitude(self) -> Attitude | None:
        return self._parts.pose.attitude

    @property
    def attitude_sample(self) -> SimpleNamespace | None:
        return self._parts.pose.attitude_sample

    def attitude_history(self, count: int) -> tuple[SimpleNamespace, ...]:
        return self._parts.pose.attitude_history(count)

    @property
    def link_identity(self) -> str | None:
        return self._parts.pose.link_identity

    @property
    def simulator_truth_pose(self) -> SimpleNamespace | None:
        return self._parts.pose.simulator_truth_pose

    @property
    def attitude_target_debug(self) -> SimpleNamespace | None:
        return self._parts.pose.attitude_target_debug

    @property
    def nav_controller_output_debug(self) -> SimpleNamespace | None:
        return self._parts.pose.nav_controller_output_debug
