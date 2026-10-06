"""Stateless simulation and parameter facets for :class:`VehicleMav`."""

from __future__ import annotations

from collections.abc import Callable

from navpy.modules.vehicle.full_param_snapshot import FullParamSnapshot
from navpy.modules.vehicle.vehicle_public_ports import (
    ParameterParts,
    SimulationParts,
)


class VehicleSimulationFacet:
    """Expose autopilot simulation capability without owning state."""

    _parts: SimulationParts

    def sim_speedup(self) -> float:
        return self._parts.simulation.speedup()

    def is_simulated_autopilot(self) -> bool:
        return self._parts.simulation.is_simulated()


class VehicleParameterFacet:
    """Delegate parameter reads, writes, and snapshot acquisition."""

    _parts: ParameterParts

    def get_parameter(
        self,
        name: str,
        timeout: float = 2.0,
        retries: int = 1,
        quiet: bool = False,
    ) -> float | None:
        return self._parts.parameters.get(name, timeout, retries, quiet)

    def get_parameter_fresh(
        self,
        name: str,
        timeout: float = 2.0,
        retries: int = 1,
        quiet: bool = False,
    ) -> float | None:
        return self._parts.parameters.get_fresh(name, timeout, retries, quiet)

    def get_param_or_default(self, name: str, default: float) -> float:
        return self._parts.parameters.get_or_default(name, default)

    def set_parameter(
        self,
        name: str,
        value: int | float,
        *,
        mav_param_type: int | None = None,
        timeout: float = 2.0,
    ) -> bool:
        return self._parts.parameters.set(
            name,
            value,
            mav_param_type=mav_param_type,
            timeout=timeout,
        )

    def send_parameter_unverified(
        self,
        name: str,
        value: int | float,
    ) -> bool:
        return self._parts.parameters.send_unverified(name, value)

    def fetch_full_param_snapshot(
        self,
        **kwargs: bool
        | float
        | Callable[[dict[str, object] | None], None]
        | None,
    ) -> FullParamSnapshot:
        return self._parts.parameters.fetch_snapshot(**kwargs)
