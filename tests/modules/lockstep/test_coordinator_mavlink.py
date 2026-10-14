"""The run journal's decoded MAVLink counts per link and direction (step 9g)."""

from __future__ import annotations

from pymavlink.dialects.v20 import ardupilotmega as mavlink

from navpy.sim.lockstep.coordinator_journal import TickRecord
from navpy.sim.lockstep.coordinator_mavlink import (
    DRAIN_TO_SITL,
    FROM_SITL,
    TO_SITL,
    MavlinkTally,
    MessageKey,
    parse_key,
)
from navpy.sim.lockstep.mavlink_wire import encoder

SERIAL0, SERIAL1 = "lockstep:serial0", "lockstep:serial1"
REQUEST = mavlink.MAVLINK_MSG_ID_TASK_ASSIGN_REQUEST


def _assign(target: int, source: int = 1) -> bytes:
    link, sink = encoder(source, 191)
    link.task_assign_request_send(0, 0, 0, 0, target, 1, 1, 0, 0, 0, 0.0)
    return sink.drain()


def _heartbeat() -> bytes:
    link, sink = encoder(1, 1)
    link.heartbeat_send(1, 3, 0, 0, 0)
    return sink.drain()


def test_counts_per_link_and_direction_keep_the_source_and_target():
    tally = MavlinkTally((SERIAL0, SERIAL1))
    record = TickRecord(1, "mission", 0)
    tally.feed(record.data, TO_SITL, {SERIAL0: _assign(1) + _assign(2) + _assign(1, source=3)})
    tally.feed(record.data, FROM_SITL, {SERIAL0: b"boot text\n" + _heartbeat(), SERIAL1: _heartbeat()})
    totals = tally.totals()
    own, peer, relayed = f"{REQUEST}:1.191>1", f"{REQUEST}:1.191>2", f"{REQUEST}:3.191>1"
    assert totals[SERIAL0][TO_SITL]["messages"] == {own: 1, peer: 1, relayed: 1}
    assert totals[SERIAL0][FROM_SITL] == {"messages": {"0:1.1>-": 1}, "skipped_bytes": 10}
    assert totals[SERIAL1][FROM_SITL]["messages"] == {"0:1.1>-": 1}
    assert totals[SERIAL1][TO_SITL]["messages"] == {}
    assert totals[SERIAL1][DRAIN_TO_SITL]["messages"] == {}
    # Per tick only the NAVLINK messages, sparse.
    assert record.data["navlink"] == {SERIAL0: {TO_SITL: {own: 1, peer: 1, relayed: 1}}}
    assert parse_key(relayed) == MessageKey(REQUEST, 3, 191, 1)
    assert parse_key("0:1.1>-") == MessageKey(0, 1, 1, None)


def test_a_message_split_across_ticks_counts_once_in_the_tick_it_completes():
    tally = MavlinkTally((SERIAL0,))
    data = _assign(2)
    first, second = TickRecord(1, "mission", 0), TickRecord(2, "mission", 20_000)
    tally.feed(first.data, TO_SITL, {SERIAL0: data[:7]})
    tally.feed(second.data, TO_SITL, {SERIAL0: data[7:]})
    assert "navlink" not in first.data
    assert second.data["navlink"][SERIAL0][TO_SITL] == {f"{REQUEST}:1.191>2": 1}


def test_a_message_outside_the_dialect_keeps_its_id_and_source():
    """pymavlink decodes no header for an unknown id (its source would read 0)."""
    message_id = 0x0F00AB
    frame = bytes([0xFD, 1, 0, 0, 7, 2, 1]) + message_id.to_bytes(3, "little") + b"x" + b"cc"
    tally = MavlinkTally((SERIAL0,))
    tally.feed({}, FROM_SITL, {SERIAL0: frame + _heartbeat()})
    assert tally.totals()[SERIAL0][FROM_SITL]["messages"] == {f"{message_id}:2.1>-": 1, "0:1.1>-": 1}
