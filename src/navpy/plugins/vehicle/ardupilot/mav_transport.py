"""The sole serialized access path to a pymavlink encoder/connection."""
from __future__ import annotations

from _thread import RLock as RLockType
from collections.abc import Callable, Iterator
from contextlib import contextmanager
from typing import TypeVar

from pymavlink import mavutil
from pymavlink.dialects.v20.ardupilotmega import MAVLink_message

ResultT = TypeVar("ResultT")


class MavTransport:
    def __init__(
        self,
        connection: mavutil.mavfile,
        send_lock: RLockType,
    ) -> None:
        self._connection = connection
        self._send_lock = send_lock

    def call(
        self,
        action: Callable[[mavutil.mavfile], ResultT],
    ) -> ResultT:
        with self._send_lock:
            return action(self._connection)

    def try_call(
        self,
        action: Callable[[mavutil.mavfile], ResultT],
        *,
        blocking: bool = False,
    ) -> ResultT | None:
        if not self._send_lock.acquire(blocking=blocking):
            return None
        try:
            return action(self._connection)
        finally:
            self._send_lock.release()

    @contextmanager
    def transaction(self) -> Iterator[mavutil.mavfile]:
        with self._send_lock:
            yield self._connection

    def send(
        self,
        message: MAVLink_message,
        *,
        source_component: int | None = None,
    ) -> None:
        def _send(connection: mavutil.mavfile) -> None:
            encoder = connection.mav
            if source_component is None:
                encoder.send(message)
                return
            had_component = hasattr(encoder, "srcComponent")
            previous_component = getattr(encoder, "srcComponent", None)
            encoder.srcComponent = int(source_component)
            try:
                encoder.send(message)
            finally:
                if had_component:
                    encoder.srcComponent = previous_component
                else:
                    delattr(encoder, "srcComponent")

        self.call(_send)

    @property
    def source_system(self) -> int | None:
        return self.call(
            lambda connection: getattr(connection.mav, "srcSystem", None)
        )

    @property
    def source_component(self) -> int | None:
        return self.call(
            lambda connection: getattr(connection.mav, "srcComponent", None)
        )
