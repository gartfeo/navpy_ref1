"""Transactional bus attachment and idempotent resource cleanup."""
from __future__ import annotations

from navpy.exception_groups import BaseExceptionGroup, ExceptionGroup

import threading
from collections.abc import Callable, Iterable
from typing import NoReturn, Protocol

from navpy.modules.vehicle.heartbeat_runtime import HeartbeatRuntime
from navpy.modules.vehicle.link_state import HeartbeatState
from navpy.modules.vehicle.logger_ref import LoggerRef
from navpy.modules.vehicle.mav_bus import MavBus, MavBusLease
from navpy.modules.vehicle.mav_bus_membership import MavMessageReceiver


class Closeable(Protocol):
    def close(self) -> None: ...


def _raise_cleanup_errors(errors: list[BaseException]) -> NoReturn:
    if len(errors) == 1:
        raise errors[0]
    if all(isinstance(error, Exception) for error in errors):
        raise ExceptionGroup("vehicle cleanup failed", errors)
    raise BaseExceptionGroup("vehicle cleanup failed", errors)


class VehicleLifecycle:
    def __init__(
        self,
        bus: MavBus,
        bus_lease: MavBusLease,
        target_system: int,
        heartbeat_runtime: HeartbeatRuntime,
        heartbeat_state: HeartbeatState,
        logger_ref: LoggerRef,
        closers: Iterable[Closeable] = (),
    ) -> None:
        self._bus = bus
        self._bus_lease = bus_lease
        self._target = target_system
        self._heartbeat_runtime = heartbeat_runtime
        self._heartbeat_state = heartbeat_state
        self._logger_ref = logger_ref
        self._closers = tuple(closers)
        self._condition = threading.Condition()
        self._status = "open"
        self._cleanup_done: set[str] = set()

    def attach(self, receiver: MavMessageReceiver) -> None:
        with self._condition:
            if self._status != "open":
                raise RuntimeError("cannot attach a closing vehicle lifecycle")
            self._bus_lease.attach(receiver)

    def wait_heartbeat_from(self, target_system: int, timeout: float = 30.0) -> bool:
        success = self._bus.wait_heartbeat(target_system, timeout)
        if success:
            self._heartbeat_state.touch()
            self._logger_ref.value.info(
                f"Heartbeat from system {target_system} received."
            )
            return True
        others = [system for system in self._bus.heartbeats if system != target_system]
        if others:
            self._logger_ref.value.warning(
                f"No heartbeat from system {target_system}; "
                f"only from {sorted(others)}."
            )
        else:
            self._logger_ref.value.warning("No heartbeats received at all.")
        return False

    def start_heartbeat(self) -> None:
        with self._condition:
            if self._status != "open":
                raise RuntimeError("cannot start heartbeat on closing lifecycle")
            self._heartbeat_runtime.start()

    def close(self) -> None:
        with self._condition:
            while self._status == "closing":
                self._condition.wait()
            if self._status == "closed":
                return
            self._status = "closing"
        errors: list[BaseException] = []
        self._attempt("heartbeat", self._heartbeat_runtime.close, errors)
        if "heartbeat" in self._cleanup_done:
            for index, closer in enumerate(self._closers):
                self._attempt(f"closer:{index}", closer.close, errors)
            self._attempt("bus_lease", self._bus_lease.release, errors)
        with self._condition:
            self._status = "close_failed" if errors else "closed"
            self._condition.notify_all()
        if errors:
            _raise_cleanup_errors(errors)

    def _attempt(
        self,
        key: str,
        action: Callable[[], None],
        errors: list[BaseException],
    ) -> None:
        if key in self._cleanup_done:
            return
        try:
            action()
        except BaseException as exc:
            errors.append(exc)
        else:
            self._cleanup_done.add(key)
