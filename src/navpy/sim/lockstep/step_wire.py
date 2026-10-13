"""The serial-step wire (``NVSTEP02``): exact layout, the firmware's reference.

All integers little-endian, no implicit padding (``struct`` with ``<``).

Frame, SITL to coordinator, once per main loop at the FAST_TASK hook::

    header  <8sHHQQQQIB3s   magic, version, port_count, boot_id[2], tick,
                            source_us, out_of_loop_advances, helpers_quiet
                            (0 or 1), 3 reserved zero bytes      = 52 bytes
    port    <IIII           fifo_bytes, read_buffer_bytes,
                            egress_dropped_bytes, egress_len     = 16 bytes
            egress bytes    egress_len bytes, then the next port

Reply, coordinator to SITL::

    header  <8sHHQ          magic, version, port_count, tick (the echo) = 20 bytes
    port    <H              ingress_len (at most 65535), then the bytes

A wrong magic, version, port count or echo, an oversize or truncated block,
trailing bytes, or a tick that is not exactly the previous one plus one is a
protocol error; the firmware prints its invalid marker and exits.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass


MAGIC = b"NVSTEP02"
VERSION = 2
PORTS = 2  # serial0 (companion), serial1 (GCS/router)
MAX_INGRESS_PER_PORT = 0xFFFF
_RESERVED = bytes(3)

FRAME_HEADER = struct.Struct("<8sHHQQQQIB3s")
FRAME_PORT = struct.Struct("<IIII")
REPLY_HEADER = struct.Struct("<8sHHQ")
REPLY_PORT = struct.Struct("<H")


class StepWireError(ValueError):
    """A protocol error: the run stops as a host fault."""


@dataclass(frozen=True)
class PortState:
    fifo_bytes: int
    read_buffer_bytes: int
    egress_dropped_bytes: int
    egress: bytes


@dataclass(frozen=True)
class Frame:
    boot_id: tuple[int, int]
    tick: int
    source_us: int
    out_of_loop_advances: int
    helpers_quiet: bool
    ports: tuple[PortState, ...]


def pack_frame(frame: Frame) -> bytes:
    if len(frame.ports) != PORTS:
        raise StepWireError(f"a frame has {PORTS} ports, got {len(frame.ports)}")
    parts = [FRAME_HEADER.pack(
        MAGIC, VERSION, PORTS, frame.boot_id[0], frame.boot_id[1], frame.tick,
        frame.source_us, frame.out_of_loop_advances, int(frame.helpers_quiet), _RESERVED,
    )]
    for port in frame.ports:
        parts.append(FRAME_PORT.pack(
            port.fifo_bytes, port.read_buffer_bytes, port.egress_dropped_bytes, len(port.egress),
        ))
        parts.append(port.egress)
    return b"".join(parts)


def unpack_frame(data: bytes, *, expected_tick: int) -> Frame:
    if len(data) < FRAME_HEADER.size:
        raise StepWireError("truncated frame header")
    magic, version, ports, boot0, boot1, tick, source_us, advances, quiet, reserved = (
        FRAME_HEADER.unpack_from(data)
    )
    _check(magic, version, ports, tick, expected_tick)
    if reserved != _RESERVED:
        raise StepWireError(f"reserved bytes must be zero, got {reserved.hex()}")
    if quiet not in (0, 1):
        raise StepWireError(f"helpers_quiet must be 0 or 1, got {quiet}")
    offset, states = FRAME_HEADER.size, []
    for _ in range(PORTS):
        if len(data) < offset + FRAME_PORT.size:
            raise StepWireError("truncated port header")
        fifo, fill, dropped, length = FRAME_PORT.unpack_from(data, offset)
        offset += FRAME_PORT.size
        if len(data) < offset + length:
            raise StepWireError("truncated egress")
        states.append(PortState(fifo, fill, dropped, bytes(data[offset:offset + length])))
        offset += length
    if offset != len(data):
        raise StepWireError(f"{len(data) - offset} trailing bytes")
    return Frame((boot0, boot1), tick, source_us, advances, bool(quiet), tuple(states))


def pack_reply(tick: int, ingress: tuple[bytes, ...]) -> bytes:
    if len(ingress) != PORTS:
        raise StepWireError(f"a reply has {PORTS} ports, got {len(ingress)}")
    parts = [REPLY_HEADER.pack(MAGIC, VERSION, PORTS, tick)]
    for block in ingress:
        if len(block) > MAX_INGRESS_PER_PORT:
            raise StepWireError(f"{len(block)} ingress bytes exceed {MAX_INGRESS_PER_PORT}")
        parts.append(REPLY_PORT.pack(len(block)))
        parts.append(block)
    return b"".join(parts)


def unpack_reply(data: bytes, *, expected_tick: int) -> tuple[bytes, ...]:
    if len(data) < REPLY_HEADER.size:
        raise StepWireError("truncated reply header")
    magic, version, ports, tick = REPLY_HEADER.unpack_from(data)
    _check(magic, version, ports, tick, expected_tick)
    offset, blocks = REPLY_HEADER.size, []
    for _ in range(PORTS):
        if len(data) < offset + REPLY_PORT.size:
            raise StepWireError("truncated ingress length")
        (length,) = REPLY_PORT.unpack_from(data, offset)
        offset += REPLY_PORT.size
        if len(data) < offset + length:
            raise StepWireError("truncated ingress")
        blocks.append(bytes(data[offset:offset + length]))
        offset += length
    if offset != len(data):
        raise StepWireError(f"{len(data) - offset} trailing bytes")
    return tuple(blocks)


def _check(magic: bytes, version: int, ports: int, tick: int, expected_tick: int) -> None:
    if magic != MAGIC:
        raise StepWireError(f"bad magic {magic!r}")
    if version != VERSION:
        raise StepWireError(f"version {version}, expected {VERSION}")
    if ports != PORTS:
        raise StepWireError(f"{ports} ports, expected {PORTS}")
    if tick != expected_tick:
        raise StepWireError(f"tick {tick}, expected {expected_tick} (missing or duplicate)")


__all__ = [
    "FRAME_HEADER", "FRAME_PORT", "Frame", "MAGIC", "MAX_INGRESS_PER_PORT", "PORTS",
    "PortState", "REPLY_HEADER", "REPLY_PORT", "StepWireError", "VERSION",
    "pack_frame", "pack_reply", "unpack_frame", "unpack_reply",
]
