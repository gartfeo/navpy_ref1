"""Public parameter capability composed from focused protocol owners."""

from __future__ import annotations

from collections.abc import Callable

from navpy.core.kernel import runtime_time as rt
from navpy.core.ports.parameter_exchange import ExchangePolicy, ParameterExchange
from navpy.core.ports.vehicle_control_interfaces import PARAM_PROTOCOL_ATTEMPTS
from navpy.plugins.vehicle.ardupilot.full_param_snapshot import FullParamSnapshot
from navpy.plugins.vehicle.ardupilot.parameter_reader import (
    MISSING_PARAMETER_RETRY_S,
    ParameterReader,
)
from navpy.plugins.vehicle.ardupilot.parameter_snapshot import ParameterSnapshotClient
from navpy.plugins.vehicle.ardupilot.parameter_writer import ParameterWriter


class ParameterClient:
    def __init__(
        self,
        reader: ParameterReader,
        writer: ParameterWriter,
        snapshot_client: ParameterSnapshotClient | None = None,
    ) -> None:
        self._reader = reader
        self._writer = writer
        self._snapshot = snapshot_client

    def get(
        self,
        name: str,
        timeout: float = 2.0,
        retries: int = PARAM_PROTOCOL_ATTEMPTS,
        quiet: bool = False,
    ) -> float | None:
        return self._reader.get(name, timeout, retries, quiet)

    def get_or_default(self, name: str, default: float) -> float:
        value = self.get(name)
        return default if value is None else value

    def get_fresh(
        self,
        name: str,
        timeout: float = 2.0,
        retries: int = PARAM_PROTOCOL_ATTEMPTS,
        quiet: bool = False,
    ) -> float | None:
        return self._reader.get_fresh(name, timeout, retries, quiet)

    def set(
        self,
        name: str,
        value: int | float,
        *,
        mav_param_type: int | None = None,
        timeout: float = 2.0,
    ) -> bool:
        return self._writer.set(
            name,
            value,
            mav_param_type=mav_param_type,
            timeout=timeout,
        )

    def read(self, name: str, policy: ExchangePolicy, timeout: float = 2.0) -> ParameterExchange:
        return self._reader.read(name, policy, timeout)

    def write(
        self,
        name: str,
        value: int | float,
        policy: ExchangePolicy,
        timeout: float = 2.0,
    ) -> ParameterExchange:
        return self._writer.write(name, value, policy, timeout=timeout)

    def send_unverified(self, name: str, value: int | float) -> bool:
        return self._writer.send_unverified(name, value)

    @property
    def max_pitch(self) -> float:
        return self.get_or_default("PTCH_LIM_MAX_DEG", 20.0)

    @property
    def min_pitch(self) -> float:
        return self.get_or_default("PTCH_LIM_MIN_DEG", -40.0)

    @property
    def lim_roll(self) -> float:
        return self.get_or_default("ROLL_LIMIT_DEG", 45.0)

    def fetch_snapshot(
        self,
        **kwargs: bool
        | float
        | Callable[[dict[str, object] | None], None]
        | None,
    ) -> FullParamSnapshot:
        if self._snapshot is None:
            raise RuntimeError("MAVFTP parameter snapshot is not configured")
        return self._snapshot.fetch(**kwargs)


__all__ = [
    "MISSING_PARAMETER_RETRY_S",
    "ParameterClient",
    "ParameterReader",
    "ParameterWriter",
]
