"""Simulator detection and scheduler-cadence observation."""

from __future__ import annotations

import math
import threading
import time
from typing import TYPE_CHECKING

from navpy.modules.vehicle.mav_transport import MavTransport
from navpy.modules.vehicle.parameter_repository import ParameterRepository
from navpy.modules.vehicle.vehicle_identity import VehicleIdentity

if TYPE_CHECKING:
    from navpy.modules.vehicle.parameter_reader import ParameterReader


class SimAutopilotState:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._value = 1.0
        self._next_probe_s = 0.0
        self._probes = 0
        self._seen = False

    def observe(self, value: int | float | None) -> None:
        if (
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isfinite(float(value))
            and float(value) > 0.0
        ):
            with self._lock:
                self._value = float(value)
                self._seen = True

    def snapshot(self) -> tuple[float, float, int, bool]:
        with self._lock:
            return self._value, self._next_probe_s, self._probes, self._seen

    def reserve_probe(
        self,
        now_s: float,
        interval_s: float,
        limit: int,
    ) -> bool:
        with self._lock:
            if now_s < self._next_probe_s:
                return False
            self._next_probe_s = now_s + interval_s
            if not self._seen and self._probes >= limit:
                return False
            self._probes += 1
            return True


class SimAutopilotCapability:
    def __init__(
        self,
        identity: VehicleIdentity,
        transport: MavTransport,
        repository: ParameterRepository,
        reader: "ParameterReader",
        state: SimAutopilotState,
    ) -> None:
        self._identity = identity
        self._transport = transport
        self._repository = repository
        self._reader = reader
        self._state = state

    def speedup(self) -> float:
        cached = self._repository.get("SIM_SPEEDUP")
        self._state.observe(cached)
        now_s = time.monotonic()
        value, _next_probe, _probes, seen = self._state.snapshot()
        if cached is None and self._state.reserve_probe(now_s, 1.0, 10):
            try:
                self._transport.call(
                    lambda connection: connection.mav.param_request_read_send(
                        self._identity.target_system,
                        0,
                        b"SIM_SPEEDUP",
                        -1,
                    )
                )
            except Exception:
                pass
        value, _next_probe, _probes, _seen = self._state.snapshot()
        return value

    def is_simulated(self) -> bool:
        _value, _next_probe, _probes, seen = self._state.snapshot()
        if seen:
            return True
        value = self._reader.get("SIM_SPEEDUP", timeout=1.0, quiet=True)
        self._state.observe(value)
        return self._state.snapshot()[3]


__all__ = ["SimAutopilotCapability", "SimAutopilotState"]
