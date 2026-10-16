"""Per-frame timing of the HITL link as the Windows relay sees it (relay.py --timing-csv).

The relay forwards bytes without parsing them; this module only watches them. Each direction
gets a FrameSplitter that cuts the byte stream into MAVLink 2 frames and reads the time_usec of
the two HIL messages (the first payload field of both, uint64 little endian: MAVLink orders the
fields by size). The relay stamps every frame with the PC clock when its read returned and when
its write to the other side returned. Pairing an actuator frame with the state that answers it
(the same time_usec) splits the round trip into the part on the PC (relay -> bridge -> relay)
and, by the gaps between board frames as they arrive, shows stalls on the way from the board.
Nothing here touches the forwarded bytes, so the link behaves the same with or without it.
"""
from __future__ import annotations

import csv
import struct
import threading
from dataclasses import dataclass, field
from typing import Optional, TextIO

MAVLINK2_STX = 0xFD
MAVLINK2_HEADER_LEN = 10            # STX, len, incompat, compat, seq, sysid, compid, msgid (3)
MAVLINK2_CRC_LEN = 2
MAVLINK2_SIGNATURE_LEN = 13
MAVLINK_IFLAG_SIGNED = 0x01
MSG_HIL_ACTUATOR_CONTROLS = 93
MSG_HIL_STATE_QUATERNION = 115
TIME_USEC = struct.Struct("<Q")
UP = "up"                           # board -> bridge: HIL_ACTUATOR_CONTROLS
DOWN = "down"                       # bridge -> board: HIL_STATE_QUATERNION
DIRECTION_MSG = {UP: MSG_HIL_ACTUATOR_CONTROLS, DOWN: MSG_HIL_STATE_QUATERNION}
COLUMNS = ("direction", "time_usec", "read_s", "written_s")
# Actuator frames whose answer has not come yet are kept this many frames (160 ms at 200 Hz):
# a reply older than that is from a link the board already declared lost (50 ms).
PENDING_LIMIT = 32
# Board frames arrive one loop apart (5 ms at 200 Hz); a gap of this many milliseconds between two
# arrivals at the relay is a stall on the way from the board (the board's own state-age limit is 50 ms).
STALL_MS = 20.0


class FrameSplitter:
    """MAVLink 2 frames out of a byte stream (chunks of any size): (msgid, the first payload field as uint64)."""

    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, data: bytes) -> list[tuple[int, int]]:
        self._buf += data
        frames = []
        while True:
            start = self._buf.find(MAVLINK2_STX)
            if start < 0:
                self._buf.clear()
                return frames
            del self._buf[:start]
            if len(self._buf) < MAVLINK2_HEADER_LEN:
                return frames
            payload_len = self._buf[1]
            size = MAVLINK2_HEADER_LEN + payload_len + MAVLINK2_CRC_LEN
            if self._buf[2] & MAVLINK_IFLAG_SIGNED:
                size += MAVLINK2_SIGNATURE_LEN
            if len(self._buf) < size:
                return frames
            msgid = int.from_bytes(self._buf[7:10], "little")
            payload = bytes(self._buf[MAVLINK2_HEADER_LEN:MAVLINK2_HEADER_LEN + payload_len])
            # MAVLink 2 trims trailing zero bytes of the payload: pad before reading the first field
            time_usec = TIME_USEC.unpack((payload + bytes(TIME_USEC.size))[:TIME_USEC.size])[0]
            frames.append((msgid, time_usec))
            del self._buf[:size]


def _percentiles(values: list[float], points: tuple[float, ...]) -> list[float]:
    ordered = sorted(values)
    return [ordered[min(len(ordered) - 1, int(p / 100.0 * len(ordered)))] for p in points]


@dataclass
class TimingStats:
    """Running numbers over the frames seen so far."""
    pc_round_trip_ms: list[float] = field(default_factory=list)     # actuator read -> its state written
    up_forward_ms: list[float] = field(default_factory=list)        # actuator read -> written to the bridge
    up_gaps_ms: list[float] = field(default_factory=list)           # between consecutive actuator reads
    unanswered: int = 0

    def summary(self) -> str:
        if not self.pc_round_trip_ms:
            return "timing: no answered frame yet"
        p50, p90, p99 = _percentiles(self.pc_round_trip_ms, (50, 90, 99))
        stalls = [g for g in self.up_gaps_ms if g > STALL_MS]
        return (f"timing: {len(self.pc_round_trip_ms)} frames answered, PC round trip (relay read -> reply "
                f"written) ms p50 {p50:.2f} p90 {p90:.2f} p99 {p99:.2f} max {max(self.pc_round_trip_ms):.2f}; "
                f"{len(stalls)} gaps over {STALL_MS:g} ms between board frames (max "
                f"{max(self.up_gaps_ms, default=0.0):.1f} ms); {self.unanswered} frames unanswered")


class FrameTimer:
    """Stamps the HIL frames of both directions; thread-safe (one relay thread per direction)."""

    def __init__(self, stream: Optional[TextIO] = None) -> None:
        self._splitters = {UP: FrameSplitter(), DOWN: FrameSplitter()}
        self._lock = threading.Lock()
        self._pending: dict[int, float] = {}            # actuator time_usec -> relay read time
        self._last_up_read: Optional[float] = None
        self.stats = TimingStats()
        self._stream = stream
        self._writer = csv.writer(stream) if stream is not None else None
        if self._writer is not None:
            self._writer.writerow(COLUMNS)

    def note(self, direction: str, data: bytes, read_s: float, written_s: float) -> None:
        """One forwarded chunk: read_s when its read returned, written_s when its write returned."""
        frames = self._splitters[direction].feed(data)
        wanted = DIRECTION_MSG[direction]
        with self._lock:
            for msgid, time_usec in frames:
                if msgid != wanted:
                    continue
                if self._writer is not None:
                    self._writer.writerow((direction, time_usec, f"{read_s:.6f}", f"{written_s:.6f}"))
                if direction == UP:
                    self._on_actuators(time_usec, read_s, written_s)
                else:
                    self._on_state(time_usec, written_s)

    def _on_actuators(self, time_usec: int, read_s: float, written_s: float) -> None:
        if self._last_up_read is not None:
            self.stats.up_gaps_ms.append((read_s - self._last_up_read) * 1e3)
        self._last_up_read = read_s
        self.stats.up_forward_ms.append((written_s - read_s) * 1e3)
        self._pending[time_usec] = read_s
        while len(self._pending) > PENDING_LIMIT:
            self._pending.pop(next(iter(self._pending)))
            self.stats.unanswered += 1

    def _on_state(self, time_usec: int, written_s: float) -> None:
        read_s = self._pending.pop(time_usec, None)
        if read_s is not None:
            self.stats.pc_round_trip_ms.append((written_s - read_s) * 1e3)

    def summary(self) -> str:
        with self._lock:
            if self._stream is not None:
                self._stream.flush()
            return self.stats.summary()

    def close(self) -> None:
        with self._lock:
            if self._stream is not None:
                self._stream.close()
                self._stream = None
                self._writer = None
