"""Router-mirror record after launch verification: the verify port into a tlog.

The SITL router mirrors every UAV's traffic, and the backend's, to the chat's
verify port (``instance_ports.verify_port``). Once ``swarm_run`` has verified
the swarm it no longer listens there, so this recorder takes the port over and
writes what arrives to one ``.tlog`` that ``mavlogdump.py`` reads. It is not a
record of all ArduPilot messages: the mirror carries nothing from before
verification, nor NavPy's own link to its autopilot (serial0).

It runs only when :data:`TLOG_ENV` names the file (``delivery_demo`` sets it).
A failure here never changes the launch or its teardown: it lands in the status
file beside the tlog, which, not the Ready text, proves that recording started.
The standard stop kills the supervisor (``gcs_stop.py``), so the last status
update and flush do not run; the once-a-second write bounds what is lost, and a
status file without a stop reason means a forced stop.
"""

from __future__ import annotations

import datetime as dt
import json
import os
import socket
import struct
import threading
import time
from pathlib import Path
from typing import Any, BinaryIO, Callable

from gcs.backend import instance_ports as ip

TLOG_ENV = "NAVPY_MAVLINK_TLOG"
#: One file, no rotation; past this the newest traffic is what is lost.
MAX_BYTES = 4 * 1024 ** 3
#: The mirror ran at about 96 kB/s on the ground; this holds tens of seconds
#: of it if the thread is held up between reads.
RECEIVE_BUFFER_BYTES = 4 * 1024 * 1024
#: Whole records go to disk this often: the most a forced stop can lose.
WRITE_INTERVAL_S = 1.0
#: How long a normal supervisor exit waits for the last write.
STOP_TIMEOUT_S = 3.0
#: Read wake-up, so a stop request is seen without a packet.
READ_TIMEOUT_S = 0.2
DATAGRAM_MAX = 65535
#: Windows refuses to replace a file another process has open; a reader holds
#: the small status file for milliseconds, so a short bounded retry gets the
#: write through (the last one, with the stop reason, has no next second).
STATUS_REPLACE_ATTEMPTS = 10
STATUS_REPLACE_PAUSE_S = 0.02

_STAMP = struct.Struct(">Q")
_V1_MAGIC, _V2_MAGIC = 0xFE, 0xFD
_V2_SIGNED = 0x01
_SIGNATURE_LEN = 13


def status_path(tlog: Path) -> Path:
    """The status file kept beside *tlog*."""
    return tlog.with_name(tlog.stem + ".status.json")


def tlog_stamp(now_s: float) -> bytes:
    """A tlog record stamp: big-endian microseconds, low two bits clear as pymavlink writes them."""
    return _STAMP.pack(int(now_s * 1.0e6) & ~3)


def split_frames(datagram: bytes) -> tuple[list[bytes], int]:
    """Whole MAVLink frames in *datagram*, and how many trailing bytes were not one."""
    frames: list[bytes] = []
    offset = 0
    while offset < len(datagram):
        magic = datagram[offset]
        if magic == _V1_MAGIC and offset + 2 <= len(datagram):
            size = 6 + datagram[offset + 1] + 2
        elif magic == _V2_MAGIC and offset + 3 <= len(datagram):
            size = 10 + datagram[offset + 1] + 2
            if datagram[offset + 2] & _V2_SIGNED:
                size += _SIGNATURE_LEN
        else:
            break
        if offset + size > len(datagram):
            break
        frames.append(datagram[offset:offset + size])
        offset += size
    return frames, len(datagram) - offset


def bind_exclusive(port: int) -> socket.socket:
    """A UDP socket on *port* that no later ``SO_REUSEADDR`` listener can share.

    A second listener on the same port would take packets from this one (a
    manual reader once starved the verifier that way). Windows has
    ``SO_EXCLUSIVEADDRUSE``; elsewhere a plain bind without reuse is exclusive.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        exclusive = getattr(socket, "SO_EXCLUSIVEADDRUSE", None)
        if exclusive is not None:
            sock.setsockopt(socket.SOL_SOCKET, exclusive, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, RECEIVE_BUFFER_BYTES)
        sock.bind(("0.0.0.0", port))
        sock.settimeout(READ_TIMEOUT_S)
    except OSError:
        sock.close()
        raise
    return sock


def _iso(now_s: float) -> str:
    return dt.datetime.fromtimestamp(now_s).astimezone().isoformat(timespec="milliseconds")


class MavlinkRecorder:
    """One daemon thread copying the verify port into a tlog."""

    def __init__(
        self, tlog: Path, port: int, *,
        clock: Callable[[], float] = time.time,
        bind: Callable[[int], socket.socket] = bind_exclusive,
        max_bytes: int = MAX_BYTES,
    ) -> None:
        self.tlog = tlog
        self.port = port
        self._clock = clock
        self._bind = bind
        self._max_bytes = max_bytes
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._status: dict[str, object] = {
            "tlog": str(tlog), "port": port, "started": None, "bytes": 0, "records": 0,
            "unframed_bytes": 0, "last_write": None, "stop_reason": None, "error": None,
        }

    def start(self) -> None:
        """Bind and start the thread; a failure is written to the status file and raised."""
        self._status["started"] = _iso(self._clock())
        try:
            self.tlog.parent.mkdir(parents=True, exist_ok=True)
            sock = self._bind(self.port)
        except OSError as error:
            self._status["error"] = f"could not bind verify port {self.port}: {error}"
            self._status["stop_reason"] = "not started"
            self._write_status()
            raise
        self._write_status()
        self._thread = threading.Thread(
            target=self._run, args=(sock,), name="mavlink-recorder", daemon=True,
        )
        self._thread.start()

    def stop(self, timeout: float = STOP_TIMEOUT_S) -> bool:
        """Ask for the last write and wait at most *timeout*; whether the thread ended."""
        self._stop.set()
        if self._thread is None:
            return True
        self._thread.join(timeout)
        return not self._thread.is_alive()

    def _run(self, sock: socket.socket) -> None:
        reason = "supervisor exit"
        try:
            with sock, open(self.tlog, "ab", buffering=0) as out:
                reason = self._copy(sock, out)
        except Exception as error:  # noqa: BLE001 - the record must never take the supervisor down
            self._status["error"] = f"{type(error).__name__}: {error}"
            reason = "error"
        self._status["stop_reason"] = reason
        self._write_status()

    def _copy(self, sock: socket.socket, out: BinaryIO) -> str:
        pending = bytearray()
        next_write = self._clock() + WRITE_INTERVAL_S
        while not self._stop.is_set():
            try:
                datagram = sock.recv(DATAGRAM_MAX)
            except socket.timeout:
                datagram = b""
            now = self._clock()
            frames, unframed = split_frames(datagram)
            self._status["unframed_bytes"] += unframed
            for frame in frames:
                record = tlog_stamp(now) + frame
                if self._status["bytes"] + len(pending) + len(record) > self._max_bytes:
                    self._flush(out, pending)
                    return f"size cap {self._max_bytes} bytes reached; later traffic not recorded"
                pending += record
                self._status["records"] += 1
            if now >= next_write:
                self._flush(out, pending)
                next_write = now + WRITE_INTERVAL_S
        self._flush(out, pending)
        return "supervisor exit"

    def _flush(self, out: BinaryIO, pending: bytearray) -> None:
        view = memoryview(pending)
        written = 0
        while written < len(view):
            written += out.write(view[written:])
        view.release()
        self._status["bytes"] += written
        if written:
            self._status["last_write"] = _iso(self._clock())
        pending.clear()
        self._write_status()

    def _write_status(self) -> None:
        path = status_path(self.tlog)
        temporary = path.with_name(path.name + ".tmp")
        try:
            temporary.write_text(json.dumps(self._status, indent=2), encoding="utf-8")
            for _attempt in range(STATUS_REPLACE_ATTEMPTS):
                try:
                    os.replace(temporary, path)
                    return
                except PermissionError:  # Windows: a reader has the file open
                    time.sleep(STATUS_REPLACE_PAUSE_S)
        except OSError:
            pass  # the status is best effort; the next write tries again


def start_from_environment(
    port: int, environ: dict[str, str] | None = None, **options: Any,
) -> MavlinkRecorder | None:
    """Start a recorder when :data:`TLOG_ENV` names a file; ``None`` when it is unset."""
    target = (os.environ if environ is None else environ).get(TLOG_ENV)
    if not target:
        return None
    recorder = MavlinkRecorder(Path(target), port, **options)
    recorder.start()
    return recorder


def start_for_chat(chat: int) -> MavlinkRecorder | None:
    """The supervisor's hook: record chat *chat*'s verify port when asked to."""
    return start_from_environment(ip.verify_port(chat))
