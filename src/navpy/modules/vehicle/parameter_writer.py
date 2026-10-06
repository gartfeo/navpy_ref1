"""Validated and opportunistic MAVLink parameter writes."""

from __future__ import annotations

import math
import threading
import time

from pymavlink import mavutil
from pymavlink.dialects.v20.ardupilotmega import (
    MAV_PARAM_TYPE_REAL32,
    MAVLink_message,
)

from navpy.modules.vehicle.logger_ref import LoggerRef
from navpy.modules.vehicle.mav_transport import MavTransport
from navpy.modules.vehicle.message_store import MessageStore
from navpy.modules.vehicle.parameter_repository import ParameterRepository
from navpy.modules.vehicle.parameter_spec import (
    ParameterWriteSpec,
    build_parameter_write_spec,
)
from navpy.modules.vehicle.sim_autopilot import SimAutopilotState
from navpy.modules.vehicle.vehicle_identity import VehicleIdentity


class ParameterWriter:
    def __init__(
        self,
        identity: VehicleIdentity,
        transport: MavTransport,
        repository: ParameterRepository,
        messages: MessageStore,
        logger_ref: LoggerRef,
        sim_state: SimAutopilotState,
    ) -> None:
        self._identity = identity
        self._transport = transport
        self._repository = repository
        self._messages = messages
        self._logger_ref = logger_ref
        self._sim_state = sim_state
        self._write_lock = threading.Lock()

    def set(
        self,
        name: str,
        value: int | float,
        *,
        mav_param_type: int | None = None,
        timeout: float = 2.0,
    ) -> bool:
        spec = build_parameter_write_spec(
            name.upper(),
            value,
            mav_param_type,
            self._logger_ref.value,
        )
        if spec is None:
            return False
        with self._write_lock:
            self._repository.invalidate(spec.name)
            cursor = self._messages.cursor()
            self._transport.call(
                lambda connection: connection.mav.param_set_send(
                    self._identity.target_system,
                    0,
                    spec.name.encode(),
                    spec.value,
                    spec.wire_type,
                )
            )
            sample = self._messages.wait_after(
                "PARAM_VALUE",
                cursor,
                lambda message: self._matches(message, spec),
                deadline=time.monotonic() + timeout,
            )
        if sample is None:
            return False
        if spec.name == "SIM_SPEEDUP":
            self._sim_state.observe(sample.message.param_value)
        return True

    def send_unverified(self, name: str, value: int | float) -> bool:
        normalized = name.upper()
        try:
            wire_value = float(value)
        except (TypeError, ValueError):
            self._logger_ref.value.warning(
                f"_send_param_set_unverified {normalized}: "
                f"non-numeric value {value!r}.",
                "param",
            )
            return False
        if not math.isfinite(wire_value):
            self._logger_ref.value.warning(
                f"_send_param_set_unverified {normalized}: "
                f"non-finite value {value!r}.",
                "param",
            )
            return False
        if not self._write_lock.acquire(blocking=False):
            self._logger_ref.value.debug(
                f"_send_param_set_unverified {normalized}: "
                "param-write busy; skipped (value carried elsewhere)."
            )
            return False
        try:
            def _send(connection: mavutil.mavfile) -> bool:
                self._repository.invalidate(normalized)
                connection.mav.param_set_send(
                    self._identity.target_system,
                    0,
                    normalized.encode(),
                    wire_value,
                    MAV_PARAM_TYPE_REAL32,
                )
                return True

            sent = self._transport.try_call(_send, blocking=False)
            if sent is not True:
                self._logger_ref.value.debug(
                    f"_send_param_set_unverified {normalized}: "
                    "transport busy; skipped."
                )
                return False
        finally:
            self._write_lock.release()
        return True

    def _matches(
        self,
        message: MAVLink_message,
        spec: ParameterWriteSpec,
    ) -> bool:
        try:
            if message.get_srcSystem() != self._identity.target_system:
                return False
            name = message.param_id.rstrip("\x00").upper()
            echoed_value = float(message.param_value)
        except Exception:
            return False
        if name != spec.name:
            return False
        if (
            spec.strict_type_check
            and getattr(message, "param_type", None) != spec.requested_type
        ):
            return False
        if spec.expected_int is not None:
            return int(round(echoed_value)) == spec.expected_int
        tolerance = 1e-5 * max(abs(spec.value), 1.0)
        return abs(echoed_value - spec.value) <= tolerance


__all__ = ["ParameterWriter"]
