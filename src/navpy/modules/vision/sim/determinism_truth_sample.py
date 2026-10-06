"""Lossless, diagnostic-only SIM_STATE capture for evidence version 2.

The codec names our fork's existing SIM_STATE layout (21 floats, two int32
coordinates, uint64 time_us). No protocol definition is modified. Original
received frame bytes are retained, including truncation and transport metadata;
they are NOT a semantic selector digest. CRC checks detect frame corruption,
not authenticity: MAVLink signatures are preserved but not authenticated here.
No host time, vehicle cache, repacking or control input is used.
"""

from __future__ import annotations

import math
import struct
from typing import Any

from pymavlink.generator.mavcrc import x25crc

CODEC = "navlink.sim_state.21f2i-time_us.v1"
_MESSAGE_ID = 108
_CRC_EXTRA = 32
_PAYLOAD = struct.Struct("<21f2iQ")
_BASE_SIZE = 84
_MAX_FRAME_SIZE = 280  # MAVLink 2 maximum, including a 13-byte signature.


def frame_sample(wire: bytes) -> tuple[int, int, int, bytes]:
    """Validate a frame; return system, component, source stamp and payload.

    The returned payload is zero-extended to the codec's exact layout size.
    A legacy/MAVLink 1 frame has no source stamp and returns zero. Unknown
    incompatibility flags, layouts, bad CRCs and non-finite floats are refused.
    """
    if not wire or len(wire) > _MAX_FRAME_SIZE:
        raise ValueError("invalid frame size")
    if wire[0] == 0xFD and len(wire) >= 12:
        header, signed = 10, bool(wire[2] & 1)
        if wire[2] & ~1 or int.from_bytes(wire[7:10], "little") != _MESSAGE_ID:
            raise ValueError("unsupported MAVLink 2 frame")
        system, component = wire[5:7]
        expected = header + wire[1] + 2 + (13 if signed else 0)
        if not 1 <= wire[1] <= _PAYLOAD.size:
            raise ValueError("unsupported SIM_STATE payload")
    elif wire[0] == 0xFE and len(wire) >= 8:
        header = 6
        if wire[5] != _MESSAGE_ID or wire[1] != _BASE_SIZE:
            raise ValueError("unsupported MAVLink 1 frame")
        system, component = wire[3:5]
        expected = header + wire[1] + 2
    else:
        raise ValueError("not a MAVLink frame")
    end = header + wire[1]
    if len(wire) != expected:
        raise ValueError("truncated or concatenated frame")
    crc = x25crc(wire[1:end])
    crc.accumulate(bytes([_CRC_EXTRA]))
    if int.from_bytes(wire[end:end + 2], "little") != crc.crc:
        raise ValueError("SIM_STATE CRC mismatch")
    payload = wire[header:end].ljust(_PAYLOAD.size, b"\0")
    fields = _PAYLOAD.unpack(payload)
    if not all(math.isfinite(value) for value in fields[:21]):
        raise ValueError("non-finite SIM_STATE payload")
    return system, component, fields[-1], payload


def sample_status(
    stamp: int | None, system: int | None, component: int | None,
    wire: bytes | None,
) -> str:
    """Classify preserved observations identically during capture and reading."""
    if wire is None:
        return "frame_unavailable"
    try:
        wire_system, wire_component, wire_stamp, _ = frame_sample(wire)
    except ValueError:
        return "invalid_frame"
    if system is None or component is None:
        return "identity_unavailable"
    if (system, component) != (wire_system, wire_component):
        return "identity_mismatch"
    if stamp is None or stamp == 0:
        return "stamp_unavailable"
    if not 0 < stamp < 2**64:
        return "invalid_stamp"
    if stamp != wire_stamp:
        return "stamp_mismatch"
    return "captured"


def _integer(value: Any) -> int | None:
    return value if type(value) is int else None


def capture_truth(message: Any) -> tuple:
    """Copy a received message under the journal's existing fault/lock guard.

    Absent methods/fields are recorded as unavailable. A getter that raises is
    a recorder fault handled by that guard, not silently turned into data.
    Snapshot bytes before reading metadata; never retain a mutable message.
    """
    get_buffer = getattr(message, "get_msgbuf", None)
    buffer = get_buffer() if callable(get_buffer) else None
    wire = (
        bytes(buffer)
        if type(buffer) in (bytes, bytearray) and len(buffer) <= _MAX_FRAME_SIZE
        else None
    )
    stamp = _integer(getattr(message, "time_us", None))
    get_system = getattr(message, "get_srcSystem", None)
    get_component = getattr(message, "get_srcComponent", None)
    system = _integer(get_system()) if callable(get_system) else None
    component = _integer(get_component()) if callable(get_component) else None
    return CODEC, sample_status(stamp, system, component, wire), stamp, system, component, wire


def valid_record(record: Any) -> bool:
    """Exact immutable layout, with status derived from the retained evidence."""
    if type(record) is not tuple or len(record) != 6:
        return False
    codec, status, stamp, system, component, wire = record
    if type(codec) is not str or codec != CODEC or type(status) is not str:
        return False
    if any(value is not None and type(value) is not int
           for value in (stamp, system, component)):
        return False
    if wire is not None and (type(wire) is not bytes or len(wire) > _MAX_FRAME_SIZE):
        return False
    return status == sample_status(stamp, system, component, wire)


UNAVAILABLE = (CODEC, "frame_unavailable", None, None, None, None)
