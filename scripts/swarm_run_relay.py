"""Lossless WSL child-output relay primitives."""

from __future__ import annotations

import os
import stat
import sys
import threading
from dataclasses import dataclass
from typing import BinaryIO, Callable


WRITE_LOCK = threading.Lock()
RELAY_CHUNK = 65536
MAX_PENDING_RECORD = 1 << 20


def is_regular_file(fd: int) -> bool:
    """Return whether a descriptor has a seekable regular-file position."""
    try:
        return stat.S_ISREG(os.fstat(fd).st_mode)
    except OSError:
        return False


@dataclass
class Relay:
    """One child stream being copied onto a supervisor-owned descriptor."""

    fd: int
    name: str
    source: BinaryIO
    thread: threading.Thread | None = None
    writes_enabled: bool = True
    retiring: bool = False
    error: OSError | None = None
    source_error: OSError | ValueError | None = None
    discarded: int = 0


def write_locked(fd: int, data: bytes) -> tuple[int, OSError | None]:
    """Write every byte while the caller holds :data:`WRITE_LOCK`."""
    written = 0
    while written < len(data):
        try:
            count = os.write(fd, data[written:])
        except InterruptedError:
            continue
        except OSError as exc:
            return written, exc
        if count <= 0:
            return written, OSError(f"write to fd {fd} returned {count}")
        written += count
    return written, None


def write_all(fd: int, data: bytes) -> None:
    """Write all bytes, serialized against relay and status writers."""
    with WRITE_LOCK:
        written, error = write_locked(fd, data)
    if error is not None:
        raise error


def emit(message: str, *, err: bool = False) -> None:
    """Emit one atomic supervisor status record."""
    with WRITE_LOCK:
        print(message, file=sys.stderr if err else sys.stdout, flush=True)


def relay_write(relay: Relay, data: bytes) -> None:
    """Copy bytes to a relay destination or account for discarded bytes."""
    with WRITE_LOCK:
        if not relay.writes_enabled:
            relay.discarded += len(data)
            return
        written, error = write_locked(relay.fd, data)
        if error is not None:
            relay.error = relay.error or error
            relay.writes_enabled = False
            relay.discarded += len(data) - written


def relay_stream(
    relay: Relay,
    *,
    write: Callable[[Relay, bytes], None] = relay_write,
) -> None:
    """Drain one child stream to EOF without splitting newline records."""
    pending = b""
    try:
        while True:
            try:
                chunk = relay.source.read1(RELAY_CHUNK)
            except (OSError, ValueError) as exc:
                if not relay.retiring:
                    relay.source_error = exc
                break
            if not chunk:
                break
            pending += chunk
            cut = pending.rfind(b"\n") + 1
            if cut:
                write(relay, pending[:cut])
                pending = pending[cut:]
            while len(pending) >= MAX_PENDING_RECORD:
                write(relay, pending[:RELAY_CHUNK])
                pending = pending[RELAY_CHUNK:]
    finally:
        if pending:
            write(relay, pending)


def retire(relay: Relay) -> None:
    """Atomically forbid a relay from writing any further bytes."""
    with WRITE_LOCK:
        relay.retiring = True
        relay.writes_enabled = False
