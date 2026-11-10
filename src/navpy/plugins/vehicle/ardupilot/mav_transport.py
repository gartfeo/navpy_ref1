"""The sole serialized access path to a pymavlink encoder/connection."""
from __future__ import annotations

from collections.abc import Callable, Iterator
from contextlib import contextmanager
from typing import TypeVar

from pymavlink import mavutil
from pymavlink.dialects.v20.ardupilotmega import MAVLink_message

from navpy.core.kernel import runtime_time as rt
from navpy.plugins.vehicle.ardupilot.mav_command_fence import MavCommandFence

ResultT = TypeVar("ResultT")


class MavTransport:
    def __init__(
        self,
        connection: mavutil.mavfile,
        send_lock: rt.LockPort,
        fence: MavCommandFence | None = None,
    ) -> None:
        self._connection = connection
        self._send_lock = send_lock
        self.fence = MavCommandFence() if fence is None else fence

    def call(
        self,
        action: Callable[[mavutil.mavfile], ResultT],
    ) -> ResultT:
        with self._send_lock:
            return action(self._connection)

    def uplink_bytes(self) -> int | None:
        """Bytes this connection's encoder has written (pymavlink's
        ``total_bytes_sent``), ``None`` when it keeps no count."""
        written = getattr(getattr(self._connection, "mav", None), "total_bytes_sent", None)
        return written if isinstance(written, int) else None

    def call_by(
        self,
        deadline_s: float | None,
        action: Callable[[mavutil.mavfile], object],
    ) -> bool:
        """Run ``action`` under the send lock unless ``deadline_s`` passes first.

        ``deadline_s`` is a ``runtime_time.monotonic()`` instant: the lock wait
        ends there, and a lock won at or after it is given back unused. Without
        a deadline it is :meth:`call`. True when ``action`` ran.
        """
        if deadline_s is None:
            self.call(action)
            return True
        if not self._send_lock.acquire(timeout=max(0.0, deadline_s - rt.monotonic())):
            return False
        try:
            if rt.monotonic() >= deadline_s:
                return False
            action(self._connection)
            return True
        finally:
            self._send_lock.release()

    def try_call(
        self,
        action: Callable[[mavutil.mavfile], ResultT],
        *,
        blocking: bool = False,
        timeout: float = -1,
    ) -> ResultT | None:
        """Run ``action`` under the send lock; ``None`` when it was not acquired.

        ``timeout`` (seconds, blocking only) bounds the wait; -1 waits forever.
        """
        if not self._send_lock.acquire(blocking=blocking, timeout=timeout):
            return None
        try:
            return action(self._connection)
        finally:
            self._send_lock.release()

    @contextmanager
    def fenced(self) -> Iterator[mavutil.mavfile | None]:
        """A navigation command's writes: the connection when the fence admits
        them, else ``None`` (dropped and counted). Admission is decided under
        the send lock, which is held for the writes (mav_command_fence.py)."""
        with self._send_lock:
            yield self._connection if self.fence.admit() else None

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
