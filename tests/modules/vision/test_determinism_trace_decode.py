"""The trace file read back strictly: every record it holds, or a refusal.

``decode_trace`` inverts ``trace_lines``. Each line decodes to the record it
was made from, checked against its layout, and a line that is not exactly the
one the writer makes from that record is refused, never repaired: D7 and D8.1
of the LANDING2 step-1 plan.
"""

from __future__ import annotations

import json
from collections.abc import Callable
from types import SimpleNamespace

import pytest

from navpy.modules.vision.sim import determinism_events as events
from navpy.modules.vision.sim.determinism_command_log import (
    COMMAND_RAISED,
    COMMAND_UNREADABLE,
)
from navpy.modules.vision.sim.determinism_events import (
    ASSOCIATION_COMMITTED,
    ASSOCIATION_FENCED,
    ASSOCIATION_REFUSED,
    COUNTED_EVENTS,
    DISCARD_PUBLISH_OVERWRITTEN,
    EVENT_ASSOCIATION,
    EVENT_DECIMATE,
    EVENT_LIFECYCLE,
    EVENT_OUTCOMES,
    EVENT_OUTPUT,
    EVENT_STAGE,
    EVENT_SUBSCRIPTION,
    EVENT_TRUTH,
    EVENT_VIOLATION,
    LIFECYCLE_CLOSED,
    OUTPUT_DELIVERED,
    OUTPUT_EMPTY,
    SLOT_PAIRS,
    STAGE_PROJECTION_FAILED,
    STAGE_STAGED,
    SUBSCRIPTION_CLOSED,
    SUBSCRIPTION_OPENED,
    TRUTH_RECORDED,
    TRUTH_UNREADABLE,
    VIOLATION_SOURCE_REPEATED,
)
from navpy.modules.vision.sim.determinism_evidence import trace_lines
from navpy.modules.vision.sim.determinism_row_layouts import (
    association_row,
    discard_row,
    lifecycle_row,
    output_row,
    stage_row,
    subscription_row,
    truth_row,
)
from navpy.modules.vision.sim.determinism_slots import slot_index
from navpy.modules.vision.sim.determinism_trace_decode import (
    DecodedTrace,
    EvidenceRefused,
    decode_trace,
    strict_json,
)

PERIOD_US = 20_000
FRAME = SimpleNamespace(
    pixel=SimpleNamespace(
        u_px=320.5,
        v_px=-12.25,
        aircraft_pitch_deg=-4.0,
        aircraft_roll_deg=3.0,
        source_timestamp_s=10.02,
    )
)
PASSES = ((1, None, None), (2, 1, 1), (3, 3, 1))
COMMANDS = (
    (1, None),
    (2, b"cmd\x00\xff"),
    (3, COMMAND_RAISED),
    (4, COMMAND_UNREADABLE),
)
LEDGER = (
    (1, 10_020, True, None),
    (2, None, True, None),
    (3, 10_010, False, "stale_boot"),
)


def _every_row() -> list[tuple]:
    """A row of every layout, each built as the journal builds it."""
    return [
        association_row(1, ASSOCIATION_COMMITTED, 10_020_000, PERIOD_US, 1),
        association_row(1, ASSOCIATION_REFUSED, None, PERIOD_US, 2),
        association_row(1, ASSOCIATION_FENCED, 10_040_000, PERIOD_US, None),
        truth_row(1, TRUTH_RECORDED, 10_020_000, PERIOD_US),
        truth_row(1, TRUTH_UNREADABLE, None, PERIOD_US),
        stage_row(1, STAGE_STAGED, 10_020_000, 10_020_000, FRAME, PERIOD_US),
        stage_row(1, STAGE_PROJECTION_FAILED, None, None, None, PERIOD_US),
        # A frame with no pixel: its payload is unreadable, and marked so.
        stage_row(1, STAGE_STAGED, 10_040_000, 10_040_000, object(), PERIOD_US),
        discard_row(
            1, DISCARD_PUBLISH_OVERWRITTEN, 10_020_000, 10_040_000, PERIOD_US
        ),
        output_row(1, OUTPUT_DELIVERED, 10_020_000, 10_040_000, 3, PERIOD_US),
        output_row(1, OUTPUT_EMPTY, None, None, None, PERIOD_US),
        lifecycle_row(1, LIFECYCLE_CLOSED, 2, 10_040_000, PERIOD_US),
        subscription_row(1, SUBSCRIPTION_OPENED, None),
        subscription_row(1, SUBSCRIPTION_CLOSED, 7),
        (EVENT_VIOLATION, 1, VIOLATION_SOURCE_REPEATED, (10_040_000, 10_040_000)),
    ]


ROWS = len(_every_row())
FIRST_PASS = ROWS
FIRST_COMMAND = FIRST_PASS + len(PASSES)
FIRST_LEDGER = FIRST_COMMAND + len(COMMANDS)
VIOLATION_LINE = ROWS - 1
LIFECYCLE_LINE = next(
    number for number, row in enumerate(_every_row()) if row[0] == EVENT_LIFECYCLE
)


def _file(**records: tuple) -> bytes:
    """The trace file ``trace_lines`` makes from these records."""
    capture = SimpleNamespace(
        rows=tuple(records.get("rows", _every_row())),
        commands=SimpleNamespace(
            passes=records.get("passes", PASSES),
            entries=records.get("commands", COMMANDS),
        ),
        ledger=SimpleNamespace(entries=records.get("ledger", LEDGER)),
    )
    return trace_lines(capture).data


def test_every_record_decodes_to_the_record_it_was_made_from() -> None:
    assert decode_trace(_file()) == DecodedTrace(
        rows=tuple(_every_row()),
        passes=PASSES,
        commands=COMMANDS,
        ledger=LEDGER,
    )


def test_a_trace_with_no_records_is_an_empty_file() -> None:
    empty = _file(rows=(), passes=(), commands=(), ledger=())
    assert empty == b""
    assert decode_trace(empty) == DecodedTrace((), (), (), ())


def test_every_event_has_a_layout_and_every_outcome_its_event() -> None:
    """The reader knows every event the vocabulary names, and each outcome
    constant as its own event's: a name added there must be added here."""
    assert {row[0] for row in _every_row()} == set(COUNTED_EVENTS)
    assert tuple(EVENT_OUTCOMES) == COUNTED_EVENTS
    prefixes = {
        "ASSOCIATION_": EVENT_ASSOCIATION,
        "TRUTH_": EVENT_TRUTH,
        "STAGE_": EVENT_STAGE,
        "DISCARD_": EVENT_DECIMATE,
        "OUTPUT_": EVENT_OUTPUT,
        "LIFECYCLE_": EVENT_LIFECYCLE,
        "SUBSCRIPTION_": EVENT_SUBSCRIPTION,
        "VIOLATION_": EVENT_VIOLATION,
    }
    named: dict[str, set[str]] = {event: set() for event in COUNTED_EVENTS}
    for name, value in vars(events).items():
        for prefix, event in prefixes.items():
            if name.startswith(prefix) and isinstance(value, str):
                named[event].add(value)
    assert {
        event: set(outcomes) for event, outcomes in EVENT_OUTCOMES.items()
    } == named


def test_each_slot_pair_is_a_stamp_and_its_slot_as_documented() -> None:
    """The pairs are the layouts ``determinism_events`` documents, and each
    holds a stamp and that stamp's slot in every row the builders make."""
    assert SLOT_PAIRS == {
        EVENT_ASSOCIATION: ((3, 4),),
        EVENT_TRUTH: ((3, 4),),
        EVENT_STAGE: ((3, 4), (5, 6)),
        EVENT_DECIMATE: ((3, 4), (5, 6)),
        EVENT_OUTPUT: ((3, 4), (5, 6)),
        EVENT_LIFECYCLE: ((4, 5),),
        EVENT_SUBSCRIPTION: (),
        EVENT_VIOLATION: (),
    }
    for row in _every_row():
        for stamp, slot in SLOT_PAIRS[row[0]]:
            assert row[slot] == slot_index(row[stamp], PERIOD_US), row


def _lines() -> list[str]:
    return _file().decode("ascii").splitlines(keepends=True)


def _edited(number: int, old: str, new: str) -> bytes:
    """The valid file with ``old`` replaced once in line ``number``."""
    lines = _lines()
    assert lines[number].count(old) == 1, (lines[number], old)
    lines[number] = lines[number].replace(old, new)
    return "".join(lines).encode("utf-8")


def _swapped(first: int, second: int) -> bytes:
    lines = _lines()
    lines[first], lines[second] = lines[second], lines[first]
    return "".join(lines).encode("ascii")


def _inserted(after: int, text: str) -> bytes:
    lines = _lines()
    lines.insert(after + 1, text)
    return "".join(lines).encode("ascii")


def _reordered_keys() -> bytes:
    lines = _lines()
    record = json.loads(lines[0])
    lines[0] = json.dumps(
        {"kind": record["kind"], "index": 0, "fields": record["fields"]},
        separators=(",", ":"),
    ) + "\n"
    return "".join(lines).encode("ascii")


HEX = b"cmd\x00\xff".hex()
REFUSED: dict[str, Callable[[], bytes]] = {
    "ends mid-line": lambda: _file()[:-1],
    "a line cut short": lambda: _edited(0, ',"index":0,"kind":"row"}', ""),
    "a key missing": lambda: _edited(0, ',"kind":"row"}', "}"),
    "not ascii": lambda: _edited(0, '"committed"', '"committéd"'),
    "a space": lambda: _edited(0, ',"index":0', ', "index":0'),
    "keys out of order": _reordered_keys,
    "an extra key": lambda: _edited(0, '"index":0,', '"index":0,"extra":1,'),
    "a duplicate key": lambda: _edited(0, '"index":0,', '"index":0,"index":0,'),
    "a float": lambda: _edited(0, "10020000,", "10020000.0,"),
    "NaN": lambda: _edited(0, "10020000,", "NaN,"),
    "a bool for an int": lambda: _edited(
        0, '"association",1,', '"association",true,'
    ),
    "a record that is not a list": lambda: _edited(
        0, '["association",1,"committed",10020000,501,1]', '"association"'
    ),
    "an unknown event": lambda: _edited(0, '"association"', '"associations"'),
    "an unknown outcome": lambda: _edited(0, '"committed"', '"commited"'),
    "another event's outcome": lambda: _edited(0, '"committed"', '"staged"'),
    "a field short": lambda: _edited(0, ",501,1]", ",501]"),
    "a field over": lambda: _edited(0, ",501,1]", ",501,1,1]"),
    "a violation detail not a pair": lambda: _edited(
        VIOLATION_LINE, "[10040000,10040000]", "[10040000]"
    ),
    "a lifecycle with no resulting epoch": lambda: _edited(
        LIFECYCLE_LINE,
        '"%s",2,' % LIFECYCLE_CLOSED,
        '"%s",null,' % LIFECYCLE_CLOSED,
    ),
    # Deeper than Python can walk recursively, not deeper than its JSON
    # parser reads (review round 1).
    "nested deeper than can be read": lambda: (
        b'{"fields":' + b"[" * 600 + b"0" + b"]" * 600
        + b',"index":0,"kind":"row"}\n'
    ),
    "upper-case hex": lambda: _edited(FIRST_COMMAND + 1, HEX, HEX.upper()),
    "bytes as text": lambda: _edited(
        FIRST_COMMAND + 1, '{"hex":"%s"}' % HEX, '"%s"' % HEX
    ),
    "a pass with text": lambda: _edited(FIRST_PASS, "[1,null,null]", '[1,"1",null]'),
    "accepted as a number": lambda: _edited(FIRST_LEDGER, ",true,", ",1,"),
    "an unknown kind": lambda: _edited(0, '"kind":"row"', '"kind":"rows"'),
    "kinds out of order": lambda: _swapped(ROWS - 1, FIRST_PASS),
    "an index skipped": lambda: _edited(1, '"index":1,', '"index":2,'),
    "an empty line": lambda: _inserted(0, "\n"),
    "a carriage return": lambda: _edited(0, "}\n", "}\r\n"),
}


@pytest.mark.parametrize("name", sorted(REFUSED))
def test_a_file_that_is_not_exactly_what_the_writer_makes_is_refused(
    name: str,
) -> None:
    data = REFUSED[name]()
    assert data != _file()
    with pytest.raises(EvidenceRefused):
        decode_trace(data)


@pytest.mark.parametrize(
    "text",
    ["NaN", "Infinity", "-Infinity", "1e999", "-1e999", '{"a": 1, "a": 1}'],
)
def test_strict_json_refuses_what_pythons_reader_lets_through(text: str) -> None:
    """Python's reader takes each: the constants, and 1e999, too large
    for a float, as floats that are not finite, and the last as
    {"a": 1}. The manifest is read with strict_json too, so this holds
    for it even where no rule looks (test_determinism_reader)."""
    with pytest.raises(EvidenceRefused):
        strict_json(text)


def test_strict_json_reads_a_finite_number_as_the_float_it_is() -> None:
    assert strict_json("[1.5, 1e308, -0.25]") == [1.5, 1e308, -0.25]
