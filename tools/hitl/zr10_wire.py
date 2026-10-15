"""SIYI frame helpers for the emulated ZR10, on navpy's codec (siyi_message.py).

Frame: 55 66, CTRL, LEN (LE), SEQ (LE), CMD, DATA, CRC16-XMODEM (LE). Replies use CTRL 02 like the real unit.
"""
from __future__ import annotations

import struct

from navpy.plugins.gimbal.siyi.siyi_message import SIYIMESSAGE

HEADER = "5566"
MIN_FRAME_CHARS = 20    # STX 2 + CTRL 1 + LEN 2 + SEQ 2 + CMD 1 + CRC 2 bytes, in hex characters
CTRL_ACK = "02"         # the control byte of every captured real-unit reply (aas-hitl protocol.py:32)


class ReplyEncoder(SIYIMESSAGE):
    """navpy's encoder with the gimbal's reply control byte; every frame sent gets the next sequence number."""

    def __init__(self) -> None:
        super().__init__()
        self._ctr = CTRL_ACK

    def frame(self, data_hex: str, cmd_id: str) -> bytes:
        return bytes.fromhex(self.encodeMsg(data_hex, cmd_id))


def split_frames(datagram: bytes, decoder: SIYIMESSAGE) -> tuple[list[tuple[str, bytes]], int]:
    """Every frame in one datagram, in order, as (command id, data); and the count of bad frames.

    A bad header resyncs to the next 55 66; a short frame or trailing bytes end the datagram; a bad CRC drops the
    frame (navpy's decodeMsg checks it).
    """
    text = datagram.hex()
    frames: list[tuple[str, bytes]] = []
    bad = 0
    while len(text) >= MIN_FRAME_CHARS:
        if text[:4] != HEADER:
            bad += 1
            text = text[2:]
            while len(text) >= 4 and text[:4] != HEADER:
                text = text[2:]
            continue
        length = int(text[8:10] + text[6:8], 16)
        frame_chars = MIN_FRAME_CHARS + 2 * length
        if len(text) < frame_chars:
            return frames, bad + 1
        frame, text = text[:frame_chars], text[frame_chars:]
        decoded = decoder.decodeMsg(frame)
        if decoded is None:
            bad += 1
            continue
        data_hex, _length, cmd_id, _seq = decoded
        frames.append((cmd_id, bytes.fromhex(data_hex)))
    if text:
        bad += 1
    return frames, bad


def tenths(value: float) -> int:
    return max(-32768, min(32767, int(round(value * 10.0))))


def int16s(*values: float) -> str:
    """Values as little-endian int16 tenths, in hex."""
    return struct.pack(f"<{len(values)}h", *(tenths(v) for v in values)).hex()


__all__ = ["CTRL_ACK", "ReplyEncoder", "int16s", "split_frames", "tenths"]
