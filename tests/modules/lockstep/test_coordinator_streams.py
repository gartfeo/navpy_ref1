"""Every byte written toward a SITL serial0 is counted once, under its write phase.

``lockstep_fleet.md``, Queues and Evidence (``coordinator_streams``). Seeded
random streams of valid packets, bad-CRC frames, garbage and a trailing
partial frame are split across replies with a small FIFO room, parking and
one or more drain ticks; the journaled byte classes must add up to the bytes
each phase carried.
"""

from __future__ import annotations

import random

import pytest

from navpy.sim.lockstep.coordinator_queues import FIFO_CAP
from tests.modules.lockstep.router_doubles import PortsHarness, Sources, serial0

SEEDS = tuple(range(40))
DEVICE = serial0(2)


def _stream(rng: random.Random) -> bytes:
    src = Sources()
    out = bytearray()
    for _ in range(rng.randint(1, 8)):
        kind = rng.choice(("assign", "heartbeat", "bad_crc", "garbage"))
        if kind == "assign":
            out += src.assign((2, 191), 1, rng.randint(1, 9))
        elif kind == "heartbeat":
            out += src.heartbeat(2, 191)
        elif kind == "bad_crc":
            frame = bytearray(src.heartbeat(2, 191))
            frame[-1] ^= 0xFF
            out += frame
        else:
            out += bytes(rng.randint(0x20, 0x7E) for _ in range(rng.randint(1, 12)))
    if rng.random() < 0.5:  # a frame framing never resolves
        out += src.heartbeat(2, 191)[:rng.randint(1, 11)]
    return bytes(out)


def _chunks(rng: random.Random, data: bytes, parts: int) -> list[bytes]:
    cuts = sorted(rng.randint(0, len(data)) for _ in range(parts - 1))
    return [data[a:b] for a, b in zip([0, *cuts], [*cuts, len(data)])]


def _parked_packet_bytes(run: PortsHarness, before_parking: int) -> int:
    """Packet bytes parked undelivered: an ``undelivered`` packet's pending bytes, and
    the parked share of a packet framing resolved in drain (``discarded{drain}``)."""
    pending = sum(e[3] for _k, e in run.events("undelivered"))
    spans = {tuple(e[1]): e[2] for _k, e in run.events("emitted")}
    took = {}
    for _k, e in run.events("admitted"):
        took[tuple(e[1])] = took.get(tuple(e[1]), 0) + e[2]
    for _k, e in run.events("discarded"):
        start, end = spans[tuple(e[1])]
        pending += max(0, min(end, before_parking) - start) - took.get(tuple(e[1]), 0)
    return pending


@pytest.mark.parametrize("seed", SEEDS)
def test_every_written_byte_is_counted_once_under_the_phase_it_was_written_in(seed):
    rng = random.Random(seed)
    run = PortsHarness()
    navpy = run.roster.add(2, DEVICE)
    replies, drains = rng.randint(1, 5), rng.randint(1, 3)
    chunks = _chunks(rng, _stream(rng), replies + drains)
    admitted = 0
    for k, chunk in enumerate(chunks[:replies], start=1):
        room = rng.randint(1, 40)
        _, ingress = run.tick(k, admitted=[navpy], done=[(navpy, DEVICE, chunk)],
                              fifo={DEVICE: FIFO_CAP - room})
        admitted += len(ingress[DEVICE])
    parked = run.park(replies + 1)
    undelivered = parked["undelivered"].get(DEVICE, {}).get("bytes", 0)
    for k, chunk in enumerate(chunks[replies:], start=replies + 2):
        run.drain(k, [(navpy, DEVICE, chunk)])
    closing = run.ports.closing()
    phases = closing["stream_bytes"].get(DEVICE, {})
    written = {"admitted": admitted, "undelivered": undelivered,
               "drain": sum(len(c) for c in chunks[replies:])}
    assert {p: sum(phases.get(p, {}).values()) for p in written} == written
    assert set(phases) <= set(written)
    counts = closing["mavlink_counts"][DEVICE]
    assert phases.get("admitted", {}).get("skipped", 0) == counts["to_sitl"]["skipped_bytes"]
    assert phases.get("drain", {}).get("skipped", 0) == counts["drain_to_sitl"]["skipped_bytes"]
    assert sum(e[2] for _k, e in run.events("admitted")) == phases.get("admitted", {}).get("packet", 0)
    assert _parked_packet_bytes(run, admitted + undelivered) == phases.get("undelivered", {}).get("packet", 0)
    assert run.journal().problems() == []
