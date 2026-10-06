"""Target membership and construction reservations for a shared MAVLink bus."""
from __future__ import annotations

import threading
from typing import Protocol

from pymavlink.dialects.v20.ardupilotmega import MAVLink_message


class MavMessageReceiver(Protocol):
    def feed_message(self, message: MAVLink_message) -> None: ...


class DuplicateMavSystemError(RuntimeError):
    """Raised when two consumers claim the same system id on one bus."""


class MavBusMembership:
    """Owns target routing and the reservations that protect bus construction."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._targets: dict[int, MavMessageReceiver] = {}
        self._reservations: set[int] = set()

    def reserve(self, system_id: int) -> None:
        with self._lock:
            self._assert_available(system_id)
            self._reservations.add(system_id)

    def attach(self, system_id: int, receiver: MavMessageReceiver) -> None:
        with self._lock:
            self._assert_available(system_id)
            self._targets[system_id] = receiver

    def activate(self, system_id: int, receiver: MavMessageReceiver) -> None:
        with self._lock:
            if system_id not in self._reservations:
                raise RuntimeError(f"system {system_id} has no bus reservation")
            self._reservations.remove(system_id)
            self._targets[system_id] = receiver

    def release(self, system_id: int, attached: bool) -> None:
        with self._lock:
            if attached:
                self._targets.pop(system_id, None)
            else:
                self._reservations.discard(system_id)

    def receivers_for(
        self,
        source_system: int,
    ) -> tuple[MavMessageReceiver, ...]:
        with self._lock:
            receiver = self._targets.get(source_system)
            if receiver is not None:
                return (receiver,)
            return tuple(self._targets.values())

    @property
    def has_targets(self) -> bool:
        with self._lock:
            return bool(self._targets)

    @property
    def is_empty(self) -> bool:
        with self._lock:
            return not self._targets and not self._reservations

    def _assert_available(self, system_id: int) -> None:
        if system_id in self._targets or system_id in self._reservations:
            raise DuplicateMavSystemError(
                f"system {system_id} is already attached or reserved"
            )
