"""How the backend reaches its ESP32 launch controller: one seam, two ports.

The wall port builds today's objects: the loopback HTTP simulator
(``esp32_simulator.Esp32Simulator``) and the HTTP trigger client
(``esp32_trigger.Esp32TriggerClient``). The lockstep backend participant
installs an in-process port instead (``gcs.backend.lockstep.esp32_port``):
no ``HTTPServer``, no port, the same answers.
"""
from __future__ import annotations

from typing import Any, Protocol


class Esp32Port(Protocol):
    def simulator(self, port: int) -> Any:
        """A simulator with ``start``, ``stop``, ``port`` and ``set_trigger_callback``."""

    def client(self, host: str, port: int | None, timeout: float = 5.0) -> Any:
        """A trigger client: ``health_check``, ``trigger_channel``, ``get_status``, ``reset_all``."""


class WallEsp32Port:
    def simulator(self, port: int) -> Any:
        from gcs.backend.esp32_simulator import Esp32Simulator

        return Esp32Simulator(port=port)

    def client(self, host: str, port: int | None, timeout: float = 5.0) -> Any:
        from gcs.backend.esp32_trigger import Esp32TriggerClient

        return Esp32TriggerClient(host, port, timeout=timeout)


class Esp32Access:
    def __init__(self) -> None:
        self.port: Esp32Port = WallEsp32Port()

    def simulator(self, port: int) -> Any:
        return self.port.simulator(port)

    def client(self, host: str, port: int | None, timeout: float = 5.0) -> Any:
        return self.port.client(host, port, timeout)


esp32 = Esp32Access()


__all__ = ["Esp32Access", "Esp32Port", "WallEsp32Port", "esp32"]
