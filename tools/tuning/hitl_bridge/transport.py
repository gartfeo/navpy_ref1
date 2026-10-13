"""Byte transports of the HITL link: serial port, TCP client, TCP listener (stdlib + pyserial).

The bridge (link.py) and the Windows relay (relay.py) share these. Every transport has the
same small interface: open() raises OSError when the other end is not there (yet), read()
returns the bytes available or b'' after READ_TIMEOUT_S, write() sends everything, and any
OSError from read() or write() means the connection is gone and must be reopened.

Device strings (make_transport):
  COM11, /dev/ttyACM1     serial port (USB CDC; the baud rate is ignored by CDC)
  tcp:HOST:PORT           TCP client (SITL --serialN=tcp:<port> listens; the bridge connects)
  tcpin:PORT              TCP listener on 127.0.0.1:PORT, one client at a time, a new client
  tcpin:HOST:PORT         accepted after a disconnect (the Windows relay connects to it)
"""
from __future__ import annotations

import os
import select
import socket
from typing import Optional, Protocol

import serial

TCP_PREFIX = "tcp:"
TCP_LISTEN_PREFIX = "tcpin:"
LISTEN_HOST_DEFAULT = "127.0.0.1"   # WSL forwards Windows 127.0.0.1:<port> to it; not exposed to the LAN
DEFAULT_SERIAL_BAUD = 115200        # USB CDC ignores it; it matters only on a real UART
READ_TIMEOUT_S = 0.1                # longest one read (or one wait for a client) blocks, so a stop request is seen quickly
WRITE_TIMEOUT_S = 0.1               # a write that cannot complete in this time means the port is gone
RX_CHUNK_BYTES = 4096
TCP_CONNECT_TIMEOUT_S = 2.0
LISTEN_BACKLOG = 1                  # one board link at a time


class NoClientYet(OSError):
    """A listener had no client within READ_TIMEOUT_S: not a failure, just keep waiting."""


class Transport(Protocol):
    def open(self) -> None:
        """Open the port (or accept one client); raise OSError when it is not there (yet)."""

    def read(self) -> bytes:
        """Return the bytes available, b'' after READ_TIMEOUT_S without any; raise OSError when gone."""

    def write(self, data: bytes) -> None:
        """Write all bytes; raise OSError when the port is gone."""

    def close(self) -> None:
        """Close the connection (a listener keeps listening); never raises."""

    def shutdown(self) -> None:
        """Release everything, a listening socket included; never raises."""


class SerialTransport:
    def __init__(self, port: str, baud: int = DEFAULT_SERIAL_BAUD) -> None:
        self.port = port
        self.baud = baud
        self._ser: Optional[serial.Serial] = None

    def open(self) -> None:
        # serial.SerialException derives from IOError, i.e. OSError
        self._ser = serial.Serial(self.port, self.baud, timeout=READ_TIMEOUT_S, write_timeout=WRITE_TIMEOUT_S)
        # Drop what the PC buffered before the port was opened (frames from before a reboot or
        # from before this bridge): only frames read from now on are answered (contract v2).
        self._ser.reset_input_buffer()

    def read(self) -> bytes:
        assert self._ser is not None
        # Wait for the first byte, then take what arrived with it (a USB packet lands at
        # once), so one MAVLink frame is passed on in one piece without waiting longer.
        data = self._ser.read(1)
        waiting = self._ser.in_waiting if data else 0
        if waiting:
            data += self._ser.read(min(waiting, RX_CHUNK_BYTES))
        return data

    def write(self, data: bytes) -> None:
        assert self._ser is not None
        self._ser.write(data)

    def close(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            except OSError:
                pass
            self._ser = None

    def shutdown(self) -> None:
        self.close()


class _SocketConnection:
    """read/write/close of one connected TCP socket."""

    _sock: Optional[socket.socket] = None

    def _adopt(self, sock: socket.socket) -> None:
        sock.settimeout(READ_TIMEOUT_S)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)   # one small frame per loop
        self._sock = sock

    def read(self) -> bytes:
        assert self._sock is not None
        try:
            data = self._sock.recv(RX_CHUNK_BYTES)
        except socket.timeout:
            return b""
        if not data:
            raise ConnectionResetError("TCP peer closed the connection")
        return data

    def write(self, data: bytes) -> None:
        assert self._sock is not None
        self._sock.sendall(data)

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None


class TcpTransport(_SocketConnection):
    """TCP client."""

    def __init__(self, host: str, port: int) -> None:
        self.host = host
        self.port = port
        self._sock = None

    def open(self) -> None:
        self._adopt(socket.create_connection((self.host, self.port), timeout=TCP_CONNECT_TIMEOUT_S))

    def shutdown(self) -> None:
        self.close()


def bind_listener(host: str, port: int) -> socket.socket:
    """A listening TCP socket; OSError (EADDRINUSE) when the port is taken."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        if os.name == "nt":
            # Windows SO_REUSEADDR would let two listeners share the port; refuse instead
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        else:
            # Linux: still refuses a port another socket listens on, but not one whose old
            # connection waits in TIME_WAIT after a restart of the bridge
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((host, port))
        sock.listen(LISTEN_BACKLOG)
    except OSError:
        sock.close()
        raise
    return sock


class TcpServerTransport(_SocketConnection):
    """TCP listener: one client at a time; after a disconnect the next open() accepts a new one.

    The port is bound when the transport is made, so a busy port is refused at start.
    """

    def __init__(self, host: str, port: int) -> None:
        self.host = host
        self._listener: Optional[socket.socket] = bind_listener(host, port)
        self.port = self._listener.getsockname()[1]
        self._sock = None
        self.peer: Optional[tuple] = None

    def open(self) -> None:
        if self._listener is None:
            raise OSError("listener already shut down")
        ready, _, _ = select.select([self._listener], [], [], READ_TIMEOUT_S)
        if not ready:
            raise NoClientYet(f"no client on {self.host}:{self.port} yet")
        conn, self.peer = self._listener.accept()
        self._adopt(conn)

    def shutdown(self) -> None:
        self.close()
        if self._listener is not None:
            try:
                self._listener.close()
            except OSError:
                pass
            self._listener = None


def _split_host_port(text: str, device: str) -> tuple[str, int]:
    host, _, port = text.rpartition(":")
    if not port.isdigit():
        raise ValueError(f"bad port in {device!r}")
    return host, int(port)


def make_transport(device: str, baud: int = DEFAULT_SERIAL_BAUD) -> Transport:
    """'tcp:host:port' -> TCP client; 'tcpin:[host:]port' -> TCP listener; anything else is a
    serial port name (COM11, /dev/ttyACM1). A listener binds its port here (OSError if busy)."""
    if device.startswith(TCP_LISTEN_PREFIX):
        host, port = _split_host_port(device[len(TCP_LISTEN_PREFIX):], device)
        return TcpServerTransport(host or LISTEN_HOST_DEFAULT, port)
    if device.startswith(TCP_PREFIX):
        host, port = _split_host_port(device[len(TCP_PREFIX):], device)
        if not host:
            raise ValueError(f"expected tcp:host:port, got {device!r}")
        return TcpTransport(host, port)
    return SerialTransport(device, baud)
