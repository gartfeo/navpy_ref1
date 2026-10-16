"""SIYI frames built and parsed here, independently of navpy's codec, so the codec itself is checked too."""
from __future__ import annotations

import struct
from dataclasses import dataclass


def crc16_xmodem(data: bytes) -> int:
    """CRC-16/XMODEM (poly 0x1021, init 0), the SIYI frame checksum."""
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def frame(cmd: int, data: bytes = b"", seq: int = 1, ctrl: int = 0x01) -> bytes:
    head = b"\x55\x66" + bytes((ctrl,)) + struct.pack("<HH", len(data), seq) + bytes((cmd,)) + data
    return head + struct.pack("<H", crc16_xmodem(head))


@dataclass(frozen=True)
class Frame:
    ctrl: int
    seq: int
    cmd: int
    data: bytes


def parse(raw: bytes) -> Frame:
    assert raw[:2] == b"\x55\x66", raw.hex()
    length, seq = struct.unpack_from("<HH", raw, 3)
    assert len(raw) == 10 + length, (len(raw), length)
    assert struct.unpack_from("<H", raw, 8 + length)[0] == crc16_xmodem(raw[:8 + length]), "bad CRC"
    return Frame(raw[2], seq, raw[7], raw[8:8 + length])
