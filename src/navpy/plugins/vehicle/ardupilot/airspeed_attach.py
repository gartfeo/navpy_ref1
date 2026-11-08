"""Attach the eligible airspeed source to a vehicle (hitl_guidance.md section 9, step 20): subscribe to the own
autopilot's AIRSPEED, SYS_STATUS, PARAM_VALUE and COMMAND_ACK, claim AIRSPEED at 20 Hz, wait for the claim's ack,
then read the pinned parameters back from the board."""
from __future__ import annotations

from collections.abc import Callable
from contextlib import AbstractContextManager
from typing import Any, Protocol

from navpy.core.kernel import runtime_time as rt
from navpy.plugins.vehicle.ardupilot.airspeed_eligibility import AirspeedEligibility, AirspeedEvent
from navpy.plugins.vehicle.ardupilot.airspeed_profile import (
    AIRSPEED_INTERVAL_US,
    AIRSPEED_MSG_ID,
    AUTOPILOT_COMPONENT,
    CLAIM_ACK_TIMEOUT_S,
    PARAM_READ_TIMEOUT_S,
    PENDING,
    SET_MESSAGE_INTERVAL,
    AirspeedProfile,
)


class AirspeedVehicle(Protocol):
    target_system: int

    def on_message(self, message_name: str, callback: Callable[[object], None]) -> object: ...

    def send_command_long(self, command: int, **parameters: object) -> object: ...

    def get_parameter_fresh(self, name: str, timeout: float = 2.0) -> float | None: ...

    def exclusive_send(self) -> AbstractContextManager[None]: ...


def _own(vehicle: AirspeedVehicle, message: object) -> bool:
    try:
        return (message.get_srcSystem() == vehicle.target_system
                and message.get_srcComponent() == AUTOPILOT_COMPONENT)
    except Exception:  # noqa: BLE001 - an unknown source is not the own autopilot
        return False


def _subscribe(vehicle: AirspeedVehicle, source: AirspeedEligibility, acked: Any) -> None:
    def on_airspeed(message: object) -> None:
        if _own(vehicle, message):
            source.observe_airspeed(int(message.id), message.airspeed, int(message.flags), rt.perf_counter())

    def on_status(message: object) -> None:
        if _own(vehicle, message):
            source.observe_status(int(message.onboard_control_sensors_present),
                                  int(message.onboard_control_sensors_enabled),
                                  int(message.onboard_control_sensors_health), rt.perf_counter())

    def on_param(message: object) -> None:
        if _own(vehicle, message):
            source.observe_param(str(message.param_id).rstrip("\x00"), float(message.param_value), rt.perf_counter())

    def on_ack(message: object) -> None:
        if _own(vehicle, message) and int(message.command) == SET_MESSAGE_INTERVAL:
            source.observe_ack(int(message.result), rt.perf_counter())
            acked.set()

    for name, callback in (("AIRSPEED", on_airspeed), ("SYS_STATUS", on_status), ("PARAM_VALUE", on_param),
                           ("COMMAND_ACK", on_ack)):
        vehicle.on_message(name, callback)


def attach(vehicle: AirspeedVehicle, profile: AirspeedProfile, events: AirspeedEvent) -> AirspeedEligibility:
    """Returns the source: pending, or already revoked when the claim or a pin failed."""
    source = AirspeedEligibility(profile, events)
    acked = rt.Event()
    _subscribe(vehicle, source, acked)
    source.claim_sent(rt.perf_counter())
    vehicle.send_command_long(SET_MESSAGE_INTERVAL, p1=AIRSPEED_MSG_ID, p2=AIRSPEED_INTERVAL_US)
    # The first ack ends the wait: accepted, or a refusal that already revoked.
    acked.wait(CLAIM_ACK_TIMEOUT_S)
    source.ack_deadline(rt.perf_counter())
    if source.state == PENDING:
        values: dict[str, float | None] = {}
        for name in profile.pins():
            try:
                values[name] = vehicle.get_parameter_fresh(name, timeout=PARAM_READ_TIMEOUT_S)
            except Exception:  # noqa: BLE001 - unreadable is a revoke, not a crash
                values[name] = None
        source.pins_read(values, rt.perf_counter())
    return source


__all__ = ["AirspeedVehicle", "attach"]
