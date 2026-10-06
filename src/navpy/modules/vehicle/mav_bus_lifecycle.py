"""Close-state serialization for a shared MAVLink connection."""
from __future__ import annotations

import logging
import threading
from collections.abc import Callable
from typing import TYPE_CHECKING, TypeVar

from pymavlink import mavutil

from navpy.modules.vehicle.mav_bus_membership import MavBusMembership

if TYPE_CHECKING:
    from navpy.modules.vehicle.mav_bus_reader import MavBusReader

ResultT = TypeVar("ResultT")

log = logging.getLogger(__name__)


class MavBusClosedError(RuntimeError):
    """Raised when work is submitted to a bus that is closing."""


class MavBusLifecycle:
    def __init__(
        self,
        connection: mavutil.mavfile,
        device: str,
        membership: MavBusMembership,
        unregister: Callable[[], None],
    ) -> None:
        self._connection = connection
        self._device = device
        self._membership = membership
        self._unregister = unregister
        self._stop = threading.Event()
        self._closed = threading.Event()
        self._lock = threading.Lock()
        self._reader = None

    @property
    def stop_event(self) -> threading.Event:
        return self._stop

    @property
    def is_closed(self) -> bool:
        return self._closed.is_set()

    @property
    def is_closing(self) -> bool:
        return self._stop.is_set() and not self._closed.is_set()

    def bind_reader(self, reader: "MavBusReader") -> None:
        self._reader = reader

    def change(self, action: Callable[[], ResultT]) -> ResultT:
        with self._lock:
            self._require_open()
            return action()

    def release(self, action: Callable[[], None]) -> None:
        with self._lock:
            action()
            if self._membership.is_empty and not self._closed.is_set():
                self._close_locked()

    def return_if_open(self, action: Callable[[], bool]) -> bool:
        with self._lock:
            if self._stop.is_set():
                return False
            return bool(action())

    def wait_closed(self, timeout_s: float | None = None) -> bool:
        return self._closed.wait(timeout_s)

    def close(self) -> None:
        with self._lock:
            self._close_locked()

    def _require_open(self) -> None:
        if self._stop.is_set():
            raise MavBusClosedError(f"MavBus {self._device} is closing")

    def _close_locked(self) -> None:
        if self._closed.is_set():
            return
        self._stop.set()
        try:
            if self._reader is not None:
                self._reader.join(timeout_s=2.0)
            self._connection.close()
        finally:
            self._unregister()
            self._closed.set()
            log.info("MavBus closed for %s", self._device)
