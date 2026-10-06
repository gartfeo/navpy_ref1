"""Clean-room parser for ArduPilot's @PARAM/param.pck format.

References used (no GPL ArduPilot or MAVProxy source consulted):
    - libraries/AP_Filesystem/README.md (ArduPilot dev documentation)
    - https://mavlink.io/en/services/parameter.html

Format summary (little-endian throughout):

    Header (6 bytes):
        u16 magic       = 0x671b, or 0x671c when defaults are included
        u16 num_params  = number of records in this snapshot
        u16 total_params = total parameter count for the vehicle

    Per parameter record:
        byte 0  = (flags << 4) | type    # both 4-bit fields
        byte 1  = (name_len << 4) | common_len
        suffix  = (name_len + 1) bytes of UTF-8 name (the chars NOT shared
                  with the previous record's name)
        value   = sizeof(ap_type) bytes (LE)
        default = sizeof(ap_type) bytes (LE), present iff (flags & 0x1)

The name reconstruction is delta-compressed: the first `common_len`
characters are taken from the previous record's full name; the next
(`name_len` + 1) bytes are the differing suffix. The first record must
have `common_len == 0`.

Type-width table (from the AP_Filesystem README enumeration):

    AP_TYPE_NONE   = 0   (must not appear in a record)
    AP_TYPE_INT8   = 1   (1 byte, signed)
    AP_TYPE_INT16  = 2   (2 bytes, signed)
    AP_TYPE_INT32  = 3   (4 bytes, signed)
    AP_TYPE_FLOAT  = 4   (4 bytes, IEEE 754 binary32)

The README notes that records do not split across MAVFTP read blocks —
the server pads each block so a record always fits inside one. In the
concatenated stream returned by `MavftpClient.read_file`, those pad
bytes appear as zero bytes between records. The parser skips zero bytes
between records up to a small budget; a zero byte where a valid record
header is expected (i.e. type nibble == 0) is treated as padding.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Optional, Tuple, Union

# AP_Param leaf-type enumeration (from AP_Filesystem README).
AP_TYPE_NONE = 0
AP_TYPE_INT8 = 1
AP_TYPE_INT16 = 2
AP_TYPE_INT32 = 3
AP_TYPE_FLOAT = 4

# (struct format, byte width) per AP_TYPE_*.
_TYPE_FORMAT: dict[int, Tuple[str, int]] = {
    AP_TYPE_INT8: ("<b", 1),
    AP_TYPE_INT16: ("<h", 2),
    AP_TYPE_INT32: ("<i", 4),
    AP_TYPE_FLOAT: ("<f", 4),
}

PCK_MAGIC = 0x671B
PCK_MAGIC_WITH_DEFAULTS = 0x671C
HEADER_SIZE = 6
_HEADER_STRUCT = struct.Struct("<HHH")  # magic, num_params, total_params

# Flag bits (only bit 0 is documented).
FLAG_HAS_DEFAULT = 0x1


class ParamPckError(ValueError):
    """Raised when a `param.pck` blob is malformed."""


@dataclass(frozen=True)
class ParamRecord:
    name: str
    value: Union[int, float]
    ap_type: int
    flags: int
    default: Union[int, float, None]
    default_known: bool


@dataclass(frozen=True)
class ParamPck:
    magic: int
    num_params: int
    total_params: int
    records: Tuple[ParamRecord, ...]


def parse_param_pck(data: bytes, *, defaults_requested: bool = False) -> ParamPck:
    """Parse a complete `@PARAM/param.pck` byte blob.

    Args:
        data: full file content as returned by `MavftpClient.read_file`.
        defaults_requested: True iff the caller used `?withdefaults=1`. When
            true, records that omit the per-record `FLAG_HAS_DEFAULT` bit
            still get `default = value, default_known = True` (the spec's
            "default omitted because it equals the current value" case).

    Raises:
        ParamPckError on truncated, malformed, or inconsistent input.
    """
    if len(data) < HEADER_SIZE:
        raise ParamPckError(
            f"truncated header: need {HEADER_SIZE} bytes, got {len(data)}")

    magic, num_params, total_params = _HEADER_STRUCT.unpack_from(data, 0)
    if magic not in (PCK_MAGIC, PCK_MAGIC_WITH_DEFAULTS):
        raise ParamPckError(
            f"bad magic: expected 0x{PCK_MAGIC:04x} or "
            f"0x{PCK_MAGIC_WITH_DEFAULTS:04x}, got 0x{magic:04x}")
    if num_params > total_params:
        raise ParamPckError(
            f"num_params {num_params} > total_params {total_params}")

    defaults_in_file = defaults_requested or magic == PCK_MAGIC_WITH_DEFAULTS
    records: list[ParamRecord] = []
    pos = HEADER_SIZE
    prev_name = ""

    for _ in range(num_params):
        pos = _skip_padding(data, pos)
        if pos >= len(data):
            raise ParamPckError(
                f"truncated record stream: expected {num_params} records, "
                f"got {len(records)}")
        record, pos = _parse_record(
            data, pos, prev_name, defaults_requested=defaults_in_file)
        records.append(record)
        prev_name = record.name

    # After all records, only zero padding may remain.
    tail = data[pos:]
    if any(b != 0 for b in tail):
        raise ParamPckError(
            f"unexpected non-zero trailing bytes (offset {pos}, "
            f"len {len(tail)})")

    return ParamPck(
        magic=magic,
        num_params=num_params,
        total_params=total_params,
        records=tuple(records),
    )


# -----------------------------------------------------------------------
# Internals
# -----------------------------------------------------------------------
def _decode_nibbles(byte: int) -> Tuple[int, int]:
    """Return `(low_nibble, high_nibble)` for one packed byte.

    Isolated so a future SITL-validated correction (if the public-doc
    bitfield order ever surprises us) is a one-line change.
    """
    return byte & 0x0F, byte >> 4


def _skip_padding(data: bytes, pos: int) -> int:
    """Advance past zero pad bytes between records.

    A record always begins with a non-zero byte (its type nibble must be
    >= 1, the flags nibble may be 0), so zero bytes here are pad bytes
    inserted by the server to keep records inside a single MAVFTP block.
    """
    n = len(data)
    while pos < n and data[pos] == 0:
        pos += 1
    return pos


def _parse_record(
    data: bytes,
    pos: int,
    prev_name: str,
    *,
    defaults_requested: bool,
) -> Tuple[ParamRecord, int]:
    if pos + 2 > len(data):
        raise ParamPckError(f"truncated record header at offset {pos}")

    ap_type, flags = _decode_nibbles(data[pos])
    common_len, name_len_field = _decode_nibbles(data[pos + 1])
    pos += 2

    if ap_type == AP_TYPE_NONE:
        raise ParamPckError(
            f"invalid AP_TYPE_NONE at offset {pos - 2} (record types "
            "must be 1..4)")
    if ap_type not in _TYPE_FORMAT:
        raise ParamPckError(
            f"unknown AP_TYPE {ap_type} at offset {pos - 2}")

    if not prev_name and common_len != 0:
        raise ParamPckError(
            f"first record must have common_len=0, got {common_len}")
    if common_len > len(prev_name):
        raise ParamPckError(
            f"common_len {common_len} exceeds previous name length "
            f"{len(prev_name)}")

    suffix_len = name_len_field + 1  # README: name_len = non_common - 1
    if pos + suffix_len > len(data):
        raise ParamPckError(
            f"truncated name suffix at offset {pos} (need {suffix_len})")

    suffix_bytes = data[pos:pos + suffix_len]
    pos += suffix_len
    # Decode as strict ASCII. The packed format counts `common_len` in
    # bytes; Python string slicing counts codepoints. ArduPilot parameter
    # names are documented ASCII (A-Z, 0-9, '_'), so refusing non-ASCII
    # eliminates a silent mis-reconstruction class without losing real
    # data. (Per Codex post-step finding for Step 1b.)
    try:
        suffix = suffix_bytes.decode("ascii")
    except UnicodeDecodeError as exc:
        raise ParamPckError(
            f"non-ASCII name suffix at offset {pos - suffix_len}: {exc}"
        ) from None

    name = prev_name[:common_len] + suffix

    fmt, width = _TYPE_FORMAT[ap_type]
    if pos + width > len(data):
        raise ParamPckError(
            f"truncated value for '{name}' at offset {pos} (need {width})")
    (value,) = struct.unpack_from(fmt, data, pos)
    pos += width

    has_default_flag = bool(flags & FLAG_HAS_DEFAULT)
    if has_default_flag:
        if pos + width > len(data):
            raise ParamPckError(
                f"truncated default for '{name}' at offset {pos} "
                f"(need {width})")
        (default,) = struct.unpack_from(fmt, data, pos)
        pos += width
        default_known = True
    elif defaults_requested:
        # Caller used ?withdefaults=1; absent flag means "default == value".
        default = value
        default_known = True
    else:
        default = None
        default_known = False

    return (
        ParamRecord(
            name=name,
            value=value,
            ap_type=ap_type,
            flags=flags,
            default=default,
            default_known=default_known,
        ),
        pos,
    )
