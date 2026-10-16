"""NPYC frame header: the Unreal NavPyUnrealBridge native camera stream.

Each frame on the TCP stream is a little-endian header followed by the raw
pixels (written by NavPyGimbalCameraActor.cpp, SendNativeCameraFrame):

v1 (36 bytes): magic ``NPYC``, version, header size, width, height, pixel
format (1 = BGRA8, 2 = BGR8), reserved, frame index, Unreal wall time of
the capture (seconds, FPlatformTime), payload size.

v2 (56 bytes): v1, then
- ``board_time_us`` (uint64): board (autopilot) time of the vehicle pose
  the frame was rendered from, copied from the vehicle_state datagram;
- ``pose_sequence`` (uint64): that datagram's sequence number;
- ``pose_flags`` (uint32): POSE_STAMP_VALID when a stamped pose was applied,
  POSE_EXACT when the rendered pose IS the sample of that board time,
  POSE_INTERPOLATED when the rendered pose is the received samples
  interpolated at that board time (the receiver's board-time interpolation;
  the sequence is then the older sample's). VALID with EXACT or INTERPOLATED
  makes the board time the time of the pose the frame shows, its exposure
  time; receive-time smoothing sets neither.

The version field selects the layout; a v1 sender stays readable.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

NPYC_MAGIC = b"NPYC"
NPYC_VERSION_1 = 1
NPYC_VERSION_2 = 2
NPYC_PIXEL_FORMAT_BGRA8 = 1
NPYC_PIXEL_FORMAT_BGR8 = 2
POSE_STAMP_VALID = 1 << 0
POSE_EXACT = 1 << 1
POSE_INTERPOLATED = 1 << 2
# Unreal clamps the native render target to 16..8192 px per side
# (NavPyGimbalCameraActor::ConfigureNativeCameraCapture).
NPYC_MAX_DIMENSION = 8192

_V1 = struct.Struct("<4sHHHHHHQdI")
_V2_TAIL = struct.Struct("<QQI")
NPYC_V1_HEADER_SIZE = _V1.size                      # 36
NPYC_V2_HEADER_SIZE = _V1.size + _V2_TAIL.size      # 56
NPYC_PREFIX_SIZE = NPYC_V1_HEADER_SIZE              # every version starts with the v1 fields
_HEADER_SIZES = {NPYC_VERSION_1: NPYC_V1_HEADER_SIZE, NPYC_VERSION_2: NPYC_V2_HEADER_SIZE}
_BYTES_PER_PIXEL = {NPYC_PIXEL_FORMAT_BGRA8: 4, NPYC_PIXEL_FORMAT_BGR8: 3}


@dataclass(frozen=True)
class NpycHeader:
    version: int
    width: int
    height: int
    pixel_format: int
    frame_index: int
    capture_time_s: float           # Unreal's own wall clock: diagnostics only
    payload_size: int
    board_time_us: int = 0
    pose_sequence: int = 0
    pose_flags: int = 0

    @property
    def header_size(self) -> int:
        return _HEADER_SIZES[self.version]

    @property
    def exposure_on_board_clock(self) -> bool:
        """The board time is the time of the pose this frame shows (v2, valid, exact or interpolated)."""
        return (self.version >= NPYC_VERSION_2 and bool(self.pose_flags & POSE_STAMP_VALID)
                and bool(self.pose_flags & (POSE_EXACT | POSE_INTERPOLATED)))


def bytes_per_pixel(pixel_format: int) -> int:
    try:
        return _BYTES_PER_PIXEL[pixel_format]
    except KeyError:
        raise ValueError(f"NPYC pixel format {pixel_format} is neither BGRA8 nor BGR8") from None


def encode_header(header: NpycHeader) -> bytes:
    """The header bytes (tests and tools; the sender is the Unreal plugin)."""
    _check_dimensions(header.width, header.height, header.pixel_format, header.payload_size)
    head = _V1.pack(NPYC_MAGIC, header.version, header.header_size, header.width, header.height,
                    header.pixel_format, 0, header.frame_index, header.capture_time_s, header.payload_size)
    if header.version == NPYC_VERSION_1:
        return head
    return head + _V2_TAIL.pack(header.board_time_us, header.pose_sequence, header.pose_flags)


def decode_prefix(raw: bytes) -> tuple[int, int]:
    """(version, header size) from the first NPYC_PREFIX_SIZE bytes; ValueError if not a known NPYC header."""
    if len(raw) != NPYC_PREFIX_SIZE:
        raise ValueError("NPYC prefix has the wrong size")
    magic, version, header_size = struct.unpack_from("<4sHH", raw)
    if magic != NPYC_MAGIC:
        raise ValueError("NPYC header has an invalid magic")
    expected = _HEADER_SIZES.get(version)
    if expected is None:
        raise ValueError(f"NPYC version {version} is not supported")
    if header_size != expected:
        raise ValueError(f"NPYC v{version} header size {header_size}, expected {expected}")
    return version, header_size


def decode_header(raw: bytes) -> NpycHeader:
    """Decode and validate a complete header (prefix plus the version's tail)."""
    version, header_size = decode_prefix(raw[:NPYC_PREFIX_SIZE])
    if len(raw) != header_size:
        raise ValueError("NPYC header has the wrong size")
    (_magic, _version, _size, width, height, pixel_format, _reserved, frame_index, capture_time_s,
     payload_size) = _V1.unpack_from(raw)
    _check_dimensions(width, height, pixel_format, payload_size)
    board_time_us = pose_sequence = pose_flags = 0
    if version == NPYC_VERSION_2:
        board_time_us, pose_sequence, pose_flags = _V2_TAIL.unpack_from(raw, NPYC_V1_HEADER_SIZE)
    return NpycHeader(version, width, height, pixel_format, frame_index, capture_time_s, payload_size,
                      board_time_us, pose_sequence, pose_flags)


def _check_dimensions(width: int, height: int, pixel_format: int, payload_size: int) -> None:
    if not (0 < width <= NPYC_MAX_DIMENSION and 0 < height <= NPYC_MAX_DIMENSION):
        raise ValueError(f"NPYC frame size {width}x{height} out of range")
    if payload_size != width * height * bytes_per_pixel(pixel_format):
        raise ValueError("NPYC payload size does not match the dimensions and pixel format")


__all__ = [
    "NPYC_MAGIC",
    "NPYC_PIXEL_FORMAT_BGR8",
    "NPYC_PIXEL_FORMAT_BGRA8",
    "NPYC_PREFIX_SIZE",
    "NPYC_V1_HEADER_SIZE",
    "NPYC_V2_HEADER_SIZE",
    "NPYC_VERSION_1",
    "NPYC_VERSION_2",
    "NpycHeader",
    "POSE_EXACT",
    "POSE_INTERPOLATED",
    "POSE_STAMP_VALID",
    "bytes_per_pixel",
    "decode_header",
    "decode_prefix",
    "encode_header",
]
