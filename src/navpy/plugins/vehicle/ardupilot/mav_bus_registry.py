"""Identity-preserving device-to-bus registry."""
from __future__ import annotations

import threading
from collections.abc import Callable
from dataclasses import dataclass
from typing import Generic, TypeVar


EntryT = TypeVar("EntryT")


@dataclass(frozen=True)
class MavBusRegistryPorts(Generic[EntryT]):
    identity: Callable[[EntryT], object]
    return_if_open: Callable[[EntryT, Callable[[], bool]], bool]
    wait_closed: Callable[[EntryT, float | None], bool]


class MavBusRegistry(Generic[EntryT]):
    def __init__(self, ports: MavBusRegistryPorts[EntryT]) -> None:
        self._ports = ports
        self._lock = threading.Lock()
        self._entries: dict[str, EntryT] = {}

    @property
    def entries(self) -> dict[str, EntryT]:
        return self._entries

    def get_or_create(
        self,
        device: str,
        create: Callable[[], EntryT],
    ) -> EntryT:
        while True:
            with self._lock:
                bus = self._entries.get(device)
                if bus is None:
                    bus = create()
                    self._entries[device] = bus
                    return bus
            if self._ports.return_if_open(
                bus,
                lambda: self._is_current(device, bus)
            ):
                return bus
            self._ports.wait_closed(bus, 2.5)

    def remove(
        self,
        device: str,
        registry_identity: object,
    ) -> None:
        with self._lock:
            bus = self._entries.get(device)
            if (
                bus is not None
                and self._ports.identity(bus) is registry_identity
            ):
                self._entries.pop(device, None)

    def _is_current(self, device: str, bus: EntryT) -> bool:
        with self._lock:
            return self._entries.get(device) is bus


__all__ = ["MavBusRegistry", "MavBusRegistryPorts"]
