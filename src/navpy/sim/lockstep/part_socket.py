"""``NVPART01`` transport: loopback TCP, each message prefixed by its ``<I`` length.

Both sides use :class:`PartSocket`. A supervised read has one absolute wall
deadline for the whole framed message, however it is fragmented: a peer
trickling bytes still breaches it (``socket.timeout``, a host fault). EOF is
reported as :class:`PeerClosed` so the coordinator can tell it from a
protocol error.
"""

from __future__ import annotations

import socket
import struct
import time

from navpy.sim.lockstep.part_wire import Done, PartWireError, Register, Tick, decode, encode

LENGTH = struct.Struct("<I")
#: Largest message either side accepts: port bytes are bounded per tick by the
#: UART model (``MAX_INGRESS_PER_PORT``) and log lines by the product's rate.
MAX_MESSAGE = 16 << 20
LOOPBACK = "127.0.0.1"


class PeerClosed(ConnectionError):
    """The peer closed the connection (EOF)."""


class PartSocket:
    def __init__(self, sock: socket.socket) -> None:
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._sock = sock

    @classmethod
    def connect(cls, address: tuple[str, int], *, timeout_s: float) -> "PartSocket":
        return cls(socket.create_connection(address, timeout=timeout_s))

    def send(self, message: Register | Done | Tick, deadline: float | None = None) -> bytes:
        data = encode(message)
        self._sock.settimeout(None if deadline is None else _remaining(deadline))
        self._sock.sendall(LENGTH.pack(len(data)) + data)
        return data

    def recv(self, deadline: float | None = None) -> tuple[Register | Done | Tick, bytes]:
        """One message and its raw bytes (the journal hashes the bytes).

        ``deadline`` is a ``time.monotonic()`` instant for the whole message;
        ``None`` waits as long as it takes (the participant side, whose pace
        is the coordinator's).
        """
        (size,) = LENGTH.unpack(self._recv_exact(LENGTH.size, deadline))
        if size > MAX_MESSAGE:
            raise PartWireError(f"message of {size} bytes exceeds {MAX_MESSAGE}")
        data = self._recv_exact(size, deadline)
        return decode(data), data

    def close(self) -> None:
        try:
            self._sock.close()
        except OSError:
            pass

    def _recv_exact(self, size: int, deadline: float | None) -> bytes:
        data = bytearray()
        while len(data) < size:
            self._sock.settimeout(None if deadline is None else _remaining(deadline))
            chunk = self._sock.recv(size - len(data))
            if not chunk:
                raise PeerClosed("peer closed")
            data += chunk
        return bytes(data)


def _remaining(deadline: float) -> float:
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        raise socket.timeout("message deadline passed")
    return remaining


class PartListener:
    """The coordinator's loopback listener; the port goes only to host records."""

    def __init__(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.bind((LOOPBACK, 0))
        self._sock.listen()
        self.address: tuple[str, int] = self._sock.getsockname()

    def accept(self, timeout_s: float) -> PartSocket:
        self._sock.settimeout(timeout_s)
        connection, _ = self._sock.accept()
        connection.settimeout(None)
        return PartSocket(connection)

    def close(self) -> None:
        self._sock.close()


__all__ = ["LENGTH", "LOOPBACK", "MAX_MESSAGE", "PartListener", "PartSocket", "PeerClosed"]
