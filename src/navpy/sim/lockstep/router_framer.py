"""mavlink-router's packet framing, one persistent decoder per inbound stream.

Reproduces ``Endpoint::read_msg`` of mavlink-router ``d59ef94`` (the
deployed ``/usr/bin/mavlink-routerd``), so the coordinator's router sees the
packets the binary sees (``lockstep_fleet.md``, Router and delivery):

- Bytes before a start marker (``0xFD`` MAVLink 2, ``0xFE`` MAVLink 1) are
  skipped: the autopilot's boot text, say.
- From a marker the frame's size comes from its header: header, payload,
  2 CRC bytes, and the 13-byte signature when MAVLink 2's signed flag is set.
  A frame not yet complete waits for the stream's next bytes (a packet
  split across ticks parses once complete).
- An id in the router entry table must pass its CRC (with the table's CRC
  extra), else the whole frame is skipped. An id outside the table is not
  checked and has no target.
- The target system and component come from the table's offsets; a field
  beyond the (MAVLink 2 zero-trimmed) payload is 0. No target is -1.

:meth:`RouterFramer.feed` returns the stream's next segments in byte
order: each complete packet, and each run of skipped bytes (``packet``
None). Together they are every byte fed, except an incomplete trailing
frame, which :attr:`RouterFramer.held` keeps until it completes.
"""

from __future__ import annotations

from dataclasses import dataclass

from pymavlink.generator.mavcrc import x25crc

from navpy.sim.lockstep.router_entries import (
    FLAG_TARGET_COMPONENT,
    FLAG_TARGET_SYSTEM,
    EntryTable,
    RouterEntry,
)

STX_V2 = 0xFD
STX_V1 = 0xFE
HEADER_V2 = 10
HEADER_V1 = 6
CRC_BYTES = 2
SIGNATURE_BYTES = 13
IFLAG_SIGNED = 0x01
#: A target field the message does not have (mavlink-router's -1).
NO_TARGET = -1


@dataclass(frozen=True)
class RouterPacket:
    raw: bytes
    msgid: int
    source: tuple[int, int]
    #: ``(system, component)``; :data:`NO_TARGET` where the entry has no such field.
    target: tuple[int, int]
    #: The id is in the router entry table (its CRC was checked).
    known: bool


@dataclass(frozen=True)
class Segment:
    #: The segment's offset in its stream (every byte fed counts).
    start: int
    data: bytes
    #: The packet, or None for skipped bytes (no marker, or a failed CRC).
    packet: RouterPacket | None


class RouterFramer:
    def __init__(self, table: EntryTable) -> None:
        self._table = table
        self._buffer = bytearray()
        #: The stream offset of the buffer's first byte.
        self._offset = 0

    @property
    def held_start(self) -> int:
        """The stream offset of the first held byte (of every byte fed, if none is held)."""
        return self._offset

    @property
    def fed(self) -> int:
        """Every byte fed so far: the stream offset the next byte gets."""
        return self._offset + len(self._buffer)

    @property
    def held(self) -> bytes:
        """An incomplete trailing frame (from its marker), waiting for more bytes."""
        return bytes(self._buffer)

    def feed(self, data: bytes) -> list[Segment]:
        self._buffer += data
        segments: list[Segment] = []
        while self._buffer:
            start = _marker(self._buffer)
            if start:
                segments.append(self._take(start, None))
                continue
            size = _frame_size(self._buffer)
            if size is None or len(self._buffer) < size:
                break
            frame = bytes(self._buffer[:size])
            segments.append(self._take(size, self._packet(frame)))
        return segments

    def _take(self, size: int, packet: RouterPacket | None) -> Segment:
        segment = Segment(self._offset, bytes(self._buffer[:size]), packet)
        del self._buffer[:size]
        self._offset += size
        return segment

    def _packet(self, frame: bytes) -> RouterPacket | None:
        v2 = frame[0] == STX_V2
        header = HEADER_V2 if v2 else HEADER_V1
        length = frame[1]
        if v2:
            source, msgid = (frame[5], frame[6]), int.from_bytes(frame[7:10], "little")
        else:
            source, msgid = (frame[3], frame[4]), frame[5]
        entry = self._table.get(msgid)
        if entry is not None and not _crc_ok(frame, header, length, entry):
            return None
        payload = frame[header:header + length]
        return RouterPacket(frame, msgid, source, _target(entry, payload), entry is not None)


def _marker(buffer: bytearray) -> int:
    """Bytes before the first start marker (all of them if there is none)."""
    if buffer[0] in (STX_V2, STX_V1):
        return 0
    found = [index for index in (buffer.find(STX_V2), buffer.find(STX_V1)) if index >= 0]
    return min(found) if found else len(buffer)


def _frame_size(buffer: bytearray) -> int | None:
    v2 = buffer[0] == STX_V2
    header = HEADER_V2 if v2 else HEADER_V1
    if len(buffer) < header:
        return None
    size = header + buffer[1] + CRC_BYTES
    if v2 and buffer[2] & IFLAG_SIGNED:
        size += SIGNATURE_BYTES
    return size


def _crc_ok(frame: bytes, header: int, length: int, entry: RouterEntry) -> bool:
    crc = x25crc(frame[1:header + length])
    crc.accumulate(bytes((entry.crc_extra,)))
    return crc.crc == int.from_bytes(frame[header + length:header + length + CRC_BYTES], "little")


def _target(entry: RouterEntry | None, payload: bytes) -> tuple[int, int]:
    if entry is None:
        return NO_TARGET, NO_TARGET
    return (_field(entry.flags & FLAG_TARGET_SYSTEM, entry.target_system_ofs, payload),
            _field(entry.flags & FLAG_TARGET_COMPONENT, entry.target_component_ofs, payload))


def _field(present: int, offset: int, payload: bytes) -> int:
    if not present:
        return NO_TARGET
    return payload[offset] if offset < len(payload) else 0


__all__ = ["NO_TARGET", "RouterFramer", "RouterPacket", "Segment"]
