"""Read-only identity of the MAVLink encoder used by this vehicle link."""
from __future__ import annotations

from navpy.modules.vehicle.mav_transport import MavTransport


class TransportIdentity:
    def __init__(self, transport: MavTransport) -> None:
        self._transport = transport

    @property
    def source_system(self) -> int | None:
        return self._transport.source_system

    @property
    def source_component(self) -> int | None:
        return self._transport.source_component
