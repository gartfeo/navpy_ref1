"""NVSTEP02 golden vectors: the firmware (step 7) must produce the same bytes."""

from __future__ import annotations

import pytest

from navpy.sim.lockstep.step_wire import (
    Frame,
    PortState,
    StepWireError,
    pack_frame,
    pack_reply,
    unpack_frame,
    unpack_reply,
)

FRAME = Frame((1, 2), 7, 140000, 0, True, (PortState(3, 4, 0, b"ab"), PortState(0, 0, 5, b"")))
FRAME_HEX = (
    "4e56535445503032" "0200" "0200"            # magic, version 2, 2 ports
    "0100000000000000" "0200000000000000"       # boot id
    "0700000000000000" "e022020000000000"       # tick 7, source_us 140000
    "00000000" "01" "000000"                    # advances 0, helpers_quiet, reserved
    "03000000" "04000000" "00000000" "02000000" "6162"  # port 0 + egress "ab"
    "00000000" "00000000" "05000000" "00000000"         # port 1, 5 dropped, no egress
)
REPLY_HEX = "4e56535445503032" "0200" "0200" "0700000000000000" "0200" "7879" "0000"


def test_frame_and_reply_match_their_golden_bytes():
    assert pack_frame(FRAME).hex() == FRAME_HEX
    assert unpack_frame(bytes.fromhex(FRAME_HEX), expected_tick=7) == FRAME
    assert pack_reply(7, (b"xy", b"")).hex() == REPLY_HEX
    assert unpack_reply(bytes.fromhex(REPLY_HEX), expected_tick=7) == (b"xy", b"")


@pytest.mark.parametrize(
    ("mutate", "message"),
    [
        (lambda data: b"NVSTEP01" + data[8:], "bad magic"),
        (lambda data: data[:8] + b"\x03\x00" + data[10:], "version"),
        (lambda data: data + b"\x00", "trailing"),
        (lambda data: data[:-1], "truncated"),
    ],
)
def test_malformed_replies_are_protocol_errors(mutate, message):
    with pytest.raises(StepWireError, match=message):
        unpack_reply(mutate(bytes.fromhex(REPLY_HEX)), expected_tick=7)


def test_a_missing_or_duplicate_tick_and_an_oversize_block_are_errors():
    with pytest.raises(StepWireError, match="missing or duplicate"):
        unpack_frame(bytes.fromhex(FRAME_HEX), expected_tick=8)
    with pytest.raises(StepWireError, match="exceed"):
        pack_reply(7, (bytes(70000), b""))


def test_nonzero_reserved_bytes_are_a_protocol_error():
    data = bytearray.fromhex(FRAME_HEX)
    data[49] = 1
    with pytest.raises(StepWireError, match="reserved"):
        unpack_frame(bytes(data), expected_tick=7)
