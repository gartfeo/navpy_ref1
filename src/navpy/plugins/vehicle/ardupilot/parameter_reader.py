"""Fresh-response MAVLink parameter reads."""

from __future__ import annotations


from pymavlink.dialects.v20.ardupilotmega import MAVLink_message

from navpy.core.ports.parameter_exchange import ExchangePolicy, ParameterExchange
from navpy.core.ports.vehicle_control_interfaces import PARAM_PROTOCOL_ATTEMPTS
from navpy.plugins.vehicle.ardupilot.link_state import HeartbeatState
from navpy.plugins.vehicle.ardupilot.logger_ref import LoggerRef
from navpy.plugins.vehicle.ardupilot.mav_transport import MavTransport
from navpy.plugins.vehicle.ardupilot.message_store import MessageStore
from navpy.plugins.vehicle.ardupilot.parameter_exchange_loop import run_exchange
from navpy.plugins.vehicle.ardupilot.parameter_repository import ParameterRepository
from navpy.plugins.vehicle.ardupilot.vehicle_identity import VehicleIdentity


MISSING_PARAMETER_RETRY_S = 10.0


class ParameterReader:
    def __init__(
        self,
        identity: VehicleIdentity,
        transport: MavTransport,
        repository: ParameterRepository,
        messages: MessageStore,
        logger_ref: LoggerRef,
        link: HeartbeatState,
    ) -> None:
        self._identity = identity
        self._transport = transport
        self._repository = repository
        self._messages = messages
        self._logger_ref = logger_ref
        self._link = link

    def get(
        self,
        name: str,
        timeout: float = 2.0,
        retries: int = PARAM_PROTOCOL_ATTEMPTS,
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
        retries: int = PARAM_PROTOCOL_ATTEMPTS,
        quiet: bool = False,
    ) -> float | None:
        normalized = name.upper()
        exchange = self.read(
            normalized, ExchangePolicy(attempts=retries), timeout, quiet=quiet,
        )
        return exchange.value if exchange.confirmed else None

    def read(
        self,
        name: str,
        policy: ExchangePolicy,
        timeout: float = 2.0,
        *,
        quiet: bool = False,
    ) -> ParameterExchange:
        """One fresh read under ``policy``; a parameter that never answers is marked
        missing."""
        normalized = name.upper()
        # An answer to an earlier request still counts.
        cursor = self._messages.cursor()

        def miss(attempt: int) -> None:
            if not quiet:
                self._logger_ref.value.info(
                    f"PARAM_VALUE {normalized} not received "
                    f"(attempt {attempt}/{policy.attempts})",
                    "param",
                )

        exchange, _sample = run_exchange(
            policy,
            self._link,
            lambda: self._transport.call(
                lambda connection: connection.mav.param_request_read_send(
                    self._identity.target_system,
                    0,
                    normalized.encode(),
                    -1,
                )
            ),
            lambda deadline: self._messages.wait_after(
                "PARAM_VALUE", cursor, lambda message: self._matches(message, normalized),
                deadline=deadline,
            ),
            timeout,
            on_miss=miss,
        )
        if not quiet and (exchange.attempts > 1 or not exchange.confirmed):
            self._logger_ref.value.info(
                f"PARAM_REQUEST_READ {normalized}: {exchange.outcome.value}"
                f"{'' if exchange.reason is None else ' (' + exchange.reason.value + ')'}"
                f" after {exchange.attempts}/{policy.attempts} sends",
                "param",
            )
        if exchange.attempts == policy.attempts and not exchange.confirmed:
            self._repository.mark_missing(normalized, MISSING_PARAMETER_RETRY_S)
        return exchange

    def _matches(self, message: MAVLink_message, name: str) -> bool:
        try:
            return (
                message.get_srcSystem() == self._identity.target_system
                and message.param_id.rstrip("\x00").upper() == name
            )
        except Exception:
            return False


__all__ = ["MISSING_PARAMETER_RETRY_S", "ParameterReader"]
