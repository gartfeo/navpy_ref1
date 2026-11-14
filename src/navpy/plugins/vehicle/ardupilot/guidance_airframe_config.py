"""Narrow autopilot-parameter adapter for guidance-law limits."""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass

from navpy.core.kernel.config.navigation_args import NavigationArgs
from navpy.core.ports.law_config import GuidanceLawConfig
from navpy.core.ports.law_config import GuidanceLawConfigProvider
from navpy.core.ports.vehicle_contract import IVehicle


@dataclass(frozen=True)
class GuidanceParameterPort:
    read: Callable[[str], object]


class GuidanceAirframeConfigProvider(GuidanceLawConfigProvider):
    """Refresh only the AP parameters needed by the pure guidance law."""

    def __init__(
        self,
        parameters: GuidanceParameterPort,
        configured_throttle: Callable[[], object],
    ) -> None:
        self._parameters = parameters
        self._configured_throttle = configured_throttle

    def read(self) -> GuidanceLawConfig | None:
        try:
            pitch_min_deg = _finite_or_none(
                self._parameters.read("PTCH_LIM_MIN_DEG")
            )
            pitch_max_deg = _finite_or_none(
                self._parameters.read("PTCH_LIM_MAX_DEG")
            )
            roll_limit_deg = _finite_or_none(
                self._parameters.read("ROLL_LIMIT_DEG")
            )
            pitch_time_constant_s = _positive_or_none(
                self._parameters.read("PTCH2SRV_TCONST")
            )
            throttle = self._read_throttle()
        except Exception:  # noqa: BLE001 - external parameter boundary
            return None
        if (
            pitch_min_deg is None
            or pitch_max_deg is None
            or roll_limit_deg is None
            or pitch_min_deg >= pitch_max_deg
            or roll_limit_deg <= 0.0
        ):
            return None
        return GuidanceLawConfig(
            pitch_min_deg=pitch_min_deg,
            pitch_max_deg=pitch_max_deg,
            roll_limit_deg=roll_limit_deg,
            pitch_time_constant_s=pitch_time_constant_s,
            throttle=throttle,
        )

    def _read_throttle(self) -> float | None:
        configured = _finite_or_none(self._configured_throttle())
        percent = (
            configured
            if configured is not None
            else _finite_or_none(self._parameters.read("TRIM_THROTTLE"))
        )
        if percent is None or percent < 0.0:
            return None
        return min(1.0, percent / 100.0)


class VehicleGuidanceLawConfigProvider(GuidanceAirframeConfigProvider):
    """Compatibility adapter for callers that still own a whole vehicle."""

    def __init__(self, vehicle: IVehicle, args: NavigationArgs) -> None:
        super().__init__(
            GuidanceParameterPort(
                lambda name: vehicle.get_parameter(name, quiet=True)
            ),
            lambda: args.guidance_throttle,
        )


def _positive_or_none(value: object) -> float | None:
    number = _finite_or_none(value)
    return number if number is not None and number > 0.0 else None


def _finite_or_none(value: object) -> float | None:
    if isinstance(value, bool) or value is None:
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


__all__ = [
    "GuidanceAirframeConfigProvider",
    "GuidanceParameterPort",
    "VehicleGuidanceLawConfigProvider",
]
