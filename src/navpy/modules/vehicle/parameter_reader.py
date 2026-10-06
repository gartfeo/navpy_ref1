"""Fresh-response MAVLink parameter reads."""

from __future__ import annotations

import time

from pymavlink.dialects.v20.ardupilotmega import MAVLink_message

from navpy.modules.vehicle.logger_ref import LoggerRef
from navpy.modules.vehicle.mav_transport import MavTransport
from navpy.modules.vehicle.message_store import MessageStore
from navpy.modules.vehicle.parameter_repository import ParameterRepository
from navpy.modules.vehicle.vehicle_identity import VehicleIdentity


MISSING_PARAMETER_RETRY_S = 10.0


class ParameterReader:
    def __init__(
        self,
        identity: VehicleIdentity,
        transport: MavTransport,
        repository: ParameterRepository,
        messages: MessageStore,
        logger_ref: LoggerRef,
    ) -> None:
        self._identity = identity
        self._transport = transport
        self._repository = repository
        self._messages = messages
        self._logger_ref = logger_ref

    def get(
        self,
        name: str,
        timeout: float = 2.0,
        retries: int = 1,
        quiet: bool = False,
    ) -> float | None:
        normalized = name.upper()
        if self._repository.contains(normalized):
            return self._repository.get(normalized)
        if self._repository.missing_is_fresh(normalized):
            return None
        return self.get_fresh(normalized, timeout, retries, quiet)

    def get_fresh(
        self,
        name: str,
        timeout: float = 2.0,
        retries: int = 1,
        quiet: bool = False,
    ) -> float | None:
        normalized = name.upper()
        for attempt in range(1, retries + 1):
            cursor = self._messages.cursor()
            self._transport.call(
                lambda connection: connection.mav.param_request_read_send(
                    self._identity.target_system,
                    0,
                    normalized.encode(),
                    -1,
                )
            )
            sample = self._messages.wait_after(
                "PARAM_VALUE",
                cursor,
                lambda message: self._matches(message, normalized),
                deadline=time.monotonic() + timeout,
            )
            if sample is not None:
                return sample.message.param_value
            if not quiet:
                self._logger_ref.value.info(
                    f"PARAM_VALUE {normalized} not received "
                    f"(attempt {attempt}/{retries})",
                    "param",
                )
        if not quiet:
            self._logger_ref.value.info(
                f"Parameter {normalized} not found after {retries} retries.",
                "param",
            )
        self._repository.mark_missing(normalized, MISSING_PARAMETER_RETRY_S)
        return None

    def _matches(self, message: MAVLink_message, name: str) -> bool:
        try:
            return (
                message.get_srcSystem() == self._identity.target_system
                and message.param_id.rstrip("\x00").upper() == name
            )
        except Exception:
            return False


__all__ = ["MISSING_PARAMETER_RETRY_S", "ParameterReader"]
