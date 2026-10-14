"""Lockstep coordinator: the tick loop, the journal, the step link.

Per tick k (``docs/design/lockstep_contracts.md``): receive the SITL's frame
k (bytes it wrote since frame k-1, at ``source_us`` = t_k), let the
participant run tick k on those bytes, send reply k (the bytes each port
receives) and extend the journal hash chain over the raw frame and reply.
A nonzero out-of-loop-advance count is a host fault.
"""

from __future__ import annotations

import csv
import hashlib
import math
import random
import socket
import struct
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

from navpy.sim.lockstep.baton import LockstepFault
from navpy.sim.lockstep.step_wire import Frame, pack_reply, unpack_frame

LENGTH = struct.Struct("<I")
#: Upper bound of one injected coordinator host delay (seconds), acceptance (c).
MAX_HOST_DELAY_S = 0.005


class StepLink:
    """The relay's TCP side: length-prefixed frames in, replies out.

    Each socket read or send has the link's own ``timeout_s``. A caller may
    also pass a ``timeout_s`` limit for a whole frame or reply (the
    coordinator's exchange deadline): then every read of that frame draws on
    one absolute deadline as well. Without one, nothing else bounds it.
    """

    def __init__(self, port: int, *, timeout_s: float) -> None:
        self._timeout_s = timeout_s
        self._sock = socket.create_connection(("127.0.0.1", port), timeout=timeout_s)
        self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def recv_frame(self, timeout_s: float = math.inf) -> bytes:
        deadline = None if math.isinf(timeout_s) else time.monotonic() + timeout_s
        (size,) = LENGTH.unpack(self._recv_exact(LENGTH.size, deadline))
        return self._recv_exact(size, deadline)

    def send_reply(self, reply: bytes, timeout_s: float = math.inf) -> None:
        self._sock.settimeout(min(self._timeout_s, timeout_s))
        self._sock.sendall(LENGTH.pack(len(reply)) + reply)

    def close(self) -> None:
        self._sock.close()

    def _recv_exact(self, size: int, deadline: float | None) -> bytes:
        data = bytearray()
        while len(data) < size:
            self._sock.settimeout(self._read_timeout(deadline))
            chunk = self._sock.recv(size - len(data))
            if not chunk:
                raise ConnectionError("relay closed")
            data += chunk
        return bytes(data)

    def _read_timeout(self, deadline: float | None) -> float:
        """One read's timeout: the link's own, bounded by the frame's deadline if any."""
        if deadline is None:
            return self._timeout_s
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise socket.timeout("frame deadline passed")
        return min(self._timeout_s, remaining)


@dataclass(frozen=True)
class JournalRow:
    tick: int
    source_us: int
    helpers_quiet: bool
    frame_sha256: str
    reply_sha256: str
    chain: str


class Journal:
    """Hash chain over every tick's raw frame and reply, in delivery order."""

    def __init__(self) -> None:
        self._chain = bytes(32)
        self.rows: list[JournalRow] = []

    def add(self, frame: Frame, raw_frame: bytes, raw_reply: bytes) -> None:
        frame_sha = hashlib.sha256(raw_frame).digest()
        reply_sha = hashlib.sha256(raw_reply).digest()
        self._chain = hashlib.sha256(
            self._chain + frame.tick.to_bytes(8, "little") + frame_sha + reply_sha
        ).digest()
        self.rows.append(JournalRow(
            frame.tick, frame.source_us, frame.helpers_quiet,
            frame_sha.hex(), reply_sha.hex(), self._chain.hex(),
        ))

    @property
    def digest(self) -> str:
        return self._chain.hex()

    def write_csv(self, path: Path) -> None:
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream, lineterminator="\n")
            writer.writerow(JournalRow.__dataclass_fields__)
            for row in self.rows:
                writer.writerow(
                    (row.tick, row.source_us, int(row.helpers_quiet),
                     row.frame_sha256, row.reply_sha256, row.chain)
                )


class Participant(Protocol):
    def on_tick(self, frame: Frame) -> tuple[bytes, bytes]:
        """Run tick k on the frame's bytes; return the ingress for each port."""


def run(
    link: StepLink,
    participant: Participant,
    *,
    ticks: int,
    host_delay_seed: int = 0,
    stop_when: Callable[[], bool] = lambda: False,
) -> tuple[Journal, list[float]]:
    """Run until ``ticks`` or ``stop_when``.

    Returns the journal and the wall seconds of each full tick cycle (SITL
    loop, relay and participant), measured between successive frames.
    """
    journal = Journal()
    delays = random.Random(host_delay_seed) if host_delay_seed else None
    wall: list[float] = []
    last: float | None = None
    for tick in range(1, ticks + 1):
        raw_frame = link.recv_frame()
        now = time.perf_counter()
        if last is not None:
            wall.append(now - last)
        last = now
        frame = unpack_frame(raw_frame, expected_tick=tick)
        if frame.out_of_loop_advances:
            raise LockstepFault(
                f"out-of-loop clock advances: {frame.out_of_loop_advances} at tick {tick}"
            )
        ingress = participant.on_tick(frame)
        if delays:
            time.sleep(delays.random() * MAX_HOST_DELAY_S)
        raw_reply = pack_reply(tick, ingress)
        link.send_reply(raw_reply)
        journal.add(frame, raw_frame, raw_reply)
        if stop_when():
            break
    return journal, wall


__all__ = ["Journal", "JournalRow", "Participant", "StepLink", "run"]
