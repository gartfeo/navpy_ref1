"""Narrow structural ports used by the task-confirmation backend."""

from __future__ import annotations

from collections.abc import Callable
from typing import Protocol

from pymavlink.dialects.v20.ardupilotmega import MAVLink_message


class MavlinkMessageSender(Protocol):
    def send_mavlink_message(self, message: MAVLink_message) -> None: ...


class MessageCallbackRegistrar(Protocol):
    def on_message(self, name: str, callback: Callable) -> None: ...


class TaskConfirmVehiclePort(
    MavlinkMessageSender,
    MessageCallbackRegistrar,
    Protocol,
):
    pass


__all__ = [
    "MavlinkMessageSender",
    "MessageCallbackRegistrar",
    "TaskConfirmVehiclePort",
]
