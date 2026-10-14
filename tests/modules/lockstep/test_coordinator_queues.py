"""The coordinator's queues toward the SITLs (``lockstep_fleet.md``, Queues).

``FIFO_CAP`` and the per-port block are pinned against the serial-step
firmware's source lines (``tests/fixtures/lockstep/firmware_fifo_source.json``,
at the commit ``scripts/lockstep_firmware.lock.json`` locks).
"""

from __future__ import annotations

import hashlib
import json
import re
from pathlib import Path

from navpy.sim.lockstep.coordinator_queues import (
    FIFO_CAP,
    MAX_BLOCK,
    Leftover,
    PortQueue,
    Progress,
    QUEUED,
    UNDELIVERED,
    ReplyQueues,
    admission_room,
)
from navpy.sim.lockstep.step_wire import MAX_INGRESS_PER_PORT

FIXTURES = Path(__file__).resolve().parents[2] / "fixtures" / "lockstep"
LOCK = Path(__file__).resolve().parents[3] / "scripts" / "lockstep_firmware.lock.json"


def test_fifo_cap_and_the_block_match_the_firmware_source():
    source = json.loads((FIXTURES / "firmware_fifo_source.json").read_text(encoding="utf-8"))
    assert source["commit"].startswith(json.loads(LOCK.read_text(encoding="utf-8"))["commit"])
    lines = source["lines"]
    (shift,) = re.findall(r"BUFFER_BYTES = 1U << (\d+);", lines["libraries/SITL/SIM_NavPyStepPort.h:30"])
    buffer_bytes = 1 << int(shift)
    # The FIFO is a ByteBuffer of BUFFER_BYTES + 1 bytes, and a ByteBuffer
    # keeps one byte free (space = size - available - 1): it holds BUFFER_BYTES.
    assert "ByteBuffer{BUFFER_BYTES + 1}" in lines["libraries/SITL/SIM_NavPyStepPort.cpp:25"]
    assert lines["libraries/AP_HAL/utility/RingBuffer.cpp:90"].strip() == "ret += _head - tail - 1;"
    assert FIFO_CAP == buffer_bytes + 1 - 1
    # A push beyond the space fails the run (never a drop): admission keeps below it.
    assert "space() < len" in lines["libraries/SITL/SIM_NavPyStepPort.cpp:43"]
    assert "ingress_fifo_full" in lines["libraries/SITL/SIM_NavPyStepPort.cpp:44"]
    (block,) = re.findall(r"MAX_BLOCK = (0x[0-9A-F]+);", lines["ArduPlane/NavPySerialStep.cpp:39"])
    assert MAX_BLOCK == MAX_INGRESS_PER_PORT == int(block, 16)


def test_admission_takes_the_wire_block_or_the_fifo_space_whichever_is_less():
    assert admission_room(0) == MAX_BLOCK
    assert admission_room(FIFO_CAP - 100) == 100
    assert admission_room(FIFO_CAP) == 0
    # Every admission fits the firmware FIFO: fifo_bytes + n <= FIFO_CAP.
    for fifo in (0, 1, FIFO_CAP - MAX_BLOCK, FIFO_CAP - MAX_BLOCK + 1, FIFO_CAP - 1, FIFO_CAP):
        assert fifo + admission_room(fifo) <= FIFO_CAP


def test_old_bytes_go_first_and_a_packet_split_across_admissions_reports_each_fragment():
    queue = PortQueue()
    queue.append(b"A" * 6, "a")
    queue.append(b"--")
    queue.append(b"B" * 5, "b")
    assert queue.admit(4, 1) == (b"AAAA", [Progress("a", 4, False)])
    assert queue.pending == 9
    assert queue.admit(5, 2) == (b"AA--B", [Progress("a", 2, True), Progress("b", 1, False)])
    assert queue.admit(100, 3) == (b"BBBB", [Progress("b", 4, True)])
    assert queue.pending == 0 and queue.admit(10, 4) == (b"", [])


def test_every_byte_keeps_its_phase_until_framing_resolves_it():
    queue = PortQueue("s0")
    base = queue.stream_base("navpy", 0)
    queue.append(b"PPP")  # a packet's prefix, written in DONE 10
    assert queue.admit(100, 10) == (b"PPP", [])  # reply 10 takes it at once, unclassified
    queue.append(b"pp")  # the suffix, in DONE 11: now the packet is complete
    assert queue.stream_base("navpy", 3) == base
    assert queue.resolve(base, base + 5) == [(("admitted", 10), 3), (QUEUED, 2)]
    queue.register("p", base, base + 5)
    assert queue.admit(1, 11) == (b"p", [Progress("p", 1, False)])
    queue.append(b"xy")
    queue.park()  # what is queued becomes undelivered; then drain writes go on
    queue.write_drain(4, 12)
    assert queue.resolve(base, base + 11) == [(("admitted", 10), 3), (("admitted", 11), 1),
                                              (UNDELIVERED, 3), (("drain", 12), 4)]
    queue.prune(base + 7)
    assert queue.resolve(base, base + 11) == [(("drain", 12), 4)]


def test_parking_returns_what_is_still_queued_then_the_queues_are_empty():
    queues = ReplyQueues(("s0", "s1"))
    queues["s1"].append(b"A" * 6, "a")
    queues["s1"].append(b"B" * 3, "b")
    queues.admit("s1", FIFO_CAP - 4, 1)  # room 4
    summary, leftovers = queues.park()
    assert summary == {"s1": {"bytes": 5, "sha256": hashlib.sha256(b"AABBB").hexdigest()}}
    assert leftovers == [Leftover("s1", "a", 4, 2), Leftover("s1", "b", 0, 3)]
    assert queues.pending() == {"s0": 0, "s1": 0}
