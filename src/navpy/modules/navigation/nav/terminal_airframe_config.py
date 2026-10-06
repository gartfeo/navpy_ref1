"""Narrow autopilot-parameter adapter for terminal-law limits."""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass

from navpy.args.navigation_args import NavigationArgs
from navpy.modules.navigation.nav.vision_nav.law import (
    TerminalLawConfig,
    TerminalLawConfigProvider,
)
from navpy.modules.vehicle.vehicle_interface import IVehicle


@dataclass(frozen=True)
class TerminalParameterPort:
    read: Callable[[str], object]


class TerminalAirframeConfigProvider(TerminalLawConfigProvider):
    """Refresh only the AP parameters needed by the pure terminal law."""

    def __init__(
        self,
        parameters: TerminalParameterPort,
        configured_throttle: Callable[[], object],
    ) -> None:
        self._parameters = parameters
        self._configured_throttle = configured_throttle

    def read(self) -> TerminalLawConfig | None:
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
        return TerminalLawConfig(
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


class VehicleTerminalLawConfigProvider(TerminalAirframeConfigProvider):
    """Compatibility adapter for callers that still own a whole vehicle."""

    def __init__(self, vehicle: IVehicle, args: NavigationArgs) -> None:
        super().__init__(
            TerminalParameterPort(
                lambda name: vehicle.get_parameter(name, quiet=True)
            ),
            lambda: args.termination_throttle,
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
    "TerminalAirframeConfigProvider",
    "TerminalParameterPort",
    "VehicleTerminalLawConfigProvider",
]
