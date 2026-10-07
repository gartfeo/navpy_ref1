"""D7: a traced case's evidence, in the form it leaves the process.

Every store is sealed, one after another and never nested, and only then
captured, once; the trace file, the summary and the manifest are all made
from that capture. The trace file holds every record the stores hold, one
canonical JSON object per line, and decoding a line gives back exactly the
record it came from: every row layout, the worker's passes and commands, and
the ATTITUDE ledger's rulings. A value the encoding has no exact form for
fails the encode instead of being coerced. The manifest carries every status
field of every store, so a field a store gains later cannot be left behind.

The files, and what an interrupt does while they are written, are tested in
``tests/scripts/test_pixel_pn_determinism_evidence.py``.
"""

from __future__ import annotations

import dataclasses
import hashlib
import json
import threading
from collections.abc import Callable
from dataclasses import replace
from types import SimpleNamespace
from typing import Any

import pytest

from navpy.modules.vision.sim.determinism_admission_ledger import (
    AdmissionLedger,
    LedgerCapture,
    PassObserver,
)
from navpy.modules.vision.sim.determinism_command_log import (
    COMMAND_RAISED,
    CommandCapture,
    CommandLoopLog,
)
from navpy.modules.vision.sim.determinism_events import (
    ASSOCIATION_COMMITTED,
    ASSOCIATION_FENCED,
    DISCARD_UNBRACKETABLE,
    EVENT_ASSOCIATION,
    EVENT_DECIMATE,
    EVENT_LIFECYCLE,
    EVENT_OUTPUT,
    EVENT_STAGE,
    EVENT_SUBSCRIPTION,
    EVENT_TRUTH,
    EVENT_VIOLATION,
    LIFECYCLE_CLOSED,
    OUTPUT_DISPATCHED,
    STAGE_STAGED,
    SUBSCRIPTION_CLOSED,
    SUBSCRIPTION_OPENED,
    TRUTH_RECORDED,
)
from navpy.modules.vision.sim.determinism_evidence import (
    GATE_VARIABLES,
    KIND_COMMAND,
    KIND_LEDGER,
    KIND_PASS,
    KIND_ROW,
    MANIFEST_KEYS as READ_ORDER,
    MANIFEST_SCHEMA,
    MANIFEST_VERSION,
    SUMMARY_NAME,
    TRACE_NAME,
    encode_field,
    evidence_manifest,
    file_record,
    sealed_capture,
    store_status,
    trace_lines,
    trace_record,
)
from navpy.modules.vision.sim.determinism_journal import RowJournal
from navpy.modules.vision.sim.determinism_row_log import TraceStatus
from navpy.modules.vision.sim.determinism_trace import DeterminismTrace
from navpy.modules.vision.sim.determinism_trace_gate import (
    DETERMINISM_TRACE_CAPACITY_ENV,
    DETERMINISM_TRACE_ENV,
)
from navpy.modules.vision.sim.determinism_trace_summary import (
    summarise,
    summarise_capture,
)

PERIOD_US = 20_000  # 50 Hz autopilot scheduler period.
# Why the router rejected the one rejected ruling here; any text will do.
REJECTED = "stale"
# The manifest's keys, in the order written: the order the reader requires.
MANIFEST_KEYS = [
    "schema",
    "version",
    "trace",
    "summary",
    "period_us",
    "capacity",
    "gate",
    "identity",
    "finalization",
    "stores",
    "eligibility",
    "error",
    "held",
]


def _ruling(
    ruling: int, stamp: int | None, accepted: bool, reason: str | None = None
) -> SimpleNamespace:
    """What the router's tap hands the ledger: one ruling on one ATTITUDE."""
    return SimpleNamespace(
        ruling=ruling, time_boot_ms=stamp, accepted=accepted, reason=reason
    )


def _frame(source_timestamp_s: float) -> SimpleNamespace:
    """A frame whose payload digests to 40 bytes."""
    return SimpleNamespace(
        pixel=SimpleNamespace(
            u_px=1.5,
            v_px=-2.5,
            aircraft_pitch_deg=3.0,
            aircraft_roll_deg=4.0,
            source_timestamp_s=source_timestamp_s,
        )
    )


def _every_record() -> DeterminismTrace:
    """A leg holding every row layout, a pass, three kinds of command entry,
    and an admitted, a rejected and a stampless ruling."""
    trace = DeterminismTrace(PERIOD_US)
    trace.ledger.append(_ruling(1, 1_000, True))
    trace.ledger.append(_ruling(2, 900, False, REJECTED))
    trace.ledger.append(_ruling(3, None, True))
    trace.record_subscription(epoch=1, outcome=SUBSCRIPTION_OPENED)
    trace.record_association(
        epoch=1, outcome=ASSOCIATION_COMMITTED, source_s=1.0
    )
    # A stamp that went backwards: a VIOLATION row, then its ASSOCIATION.
    trace.record_association(epoch=1, outcome=ASSOCIATION_FENCED, source_s=0.5)
    trace.record_truth(epoch=1, outcome=TRUTH_RECORDED)
    trace.record_stage(epoch=1, outcome=STAGE_STAGED, frame=_frame(1.0))
    trace.record_discard(
        epoch=1, reason=DISCARD_UNBRACKETABLE, victim=_frame(0.98)
    )
    PassObserver(trace.ledger, trace.command_log).note_iteration(1)
    trace.record_output(
        epoch=1,
        outcome=OUTPUT_DISPATCHED,
        taken_at_us=1_000_000,
        frame=_frame(1.0),
    )
    trace.command_log.note_command(
        1,
        SimpleNamespace(
            yaw=0.1, pitch=0.2, cmd_roll=None, cmd_pitch=-0.3, cmd_thr=None
        ),
    )
    trace.command_log.note_command(2, None)
    trace.command_log.note_command(3, None, raised=True)
    trace.record_lifecycle(epoch=1, outcome=LIFECYCLE_CLOSED, resulting_epoch=2)
    trace.record_subscription(epoch=2, outcome=SUBSCRIPTION_CLOSED)
    return trace


def _empty_capture():
    return sealed_capture(DeterminismTrace(PERIOD_US))


def _decoded(value: Any) -> Any:
    """A line's fields back as the record they came from: arrays as tuples,
    ``{"hex": ...}`` as bytes, and everything else as JSON gave it."""
    if isinstance(value, list):
        return tuple(_decoded(item) for item in value)
    if isinstance(value, dict):
        assert list(value) == ["hex"], value
        return bytes.fromhex(value["hex"])
    return value


def _records(data: bytes) -> list[tuple[str, int, Any]]:
    return [
        (record["kind"], record["index"], _decoded(record["fields"]))
        for record in map(json.loads, data.decode("ascii").splitlines())
    ]


def _members(record_type: type) -> set[str]:
    """Every field of a store's record, and every verdict it derives."""
    return {field.name for field in dataclasses.fields(record_type)} | {
        name
        for name, member in vars(record_type).items()
        if isinstance(member, property)
    }


class _WatchedLock:
    """A real lock that notes when it is taken, and whether another watched
    lock was held then. ``taken`` is the positive control: a watcher that
    nothing acquires reports no overlap forever."""

    def __init__(
        self,
        name: str,
        held: set[str],
        overlaps: list[tuple[str, ...]],
        taken: set[str],
    ) -> None:
        self._name = name
        self._held = held
        self._overlaps = overlaps
        self._taken = taken
        self._lock = threading.Lock()

    def __enter__(self) -> _WatchedLock:
        self._lock.acquire()
        self._taken.add(self._name)
        if self._held:
            self._overlaps.append((*sorted(self._held), self._name))
        self._held.add(self._name)
        return self

    def __exit__(self, *_: object) -> None:
        self._held.discard(self._name)
        self._lock.release()


def _noting(
    order: list[object], name: str, seal: Callable[[Any], None]
) -> Callable[[Any], None]:
    def noted(self: Any) -> None:
        order.append(name)
        seal(self)

    return noted


def test_every_store_is_sealed_alone_and_in_order_before_the_one_capture(
    monkeypatch,
):
    """D5's order: the journal, the command log, the ledger, each under its
    own lock alone, and only then the capture, with all three sealed."""
    trace = _every_record()
    held: set[str] = set()
    overlaps: list[tuple[str, ...]] = []
    taken: set[str] = set()
    order: list[object] = []
    stores = {
        "journal": (RowJournal, trace.journal),
        "commands": (CommandLoopLog, trace.command_log),
        "ledger": (AdmissionLedger, trace.ledger),
    }
    for name, (owner, store) in stores.items():
        store._lock = _WatchedLock(name, held, overlaps, taken)
        monkeypatch.setattr(owner, "seal", _noting(order, name, owner.seal))
    real_capture = trace.capture

    def capture():
        order.append(
            ("capture", [store._seal.sealed for _, store in stores.values()])
        )
        return real_capture()

    trace.capture = capture

    sealed_capture(trace)

    assert order == [
        "journal", "commands", "ledger", ("capture", [True, True, True])
    ]
    assert taken == set(stores)
    assert overlaps == []


def test_a_recording_after_the_seal_changes_nothing_and_is_counted():
    trace = _every_record()

    sealed = sealed_capture(trace)
    trace.record_truth(epoch=1, outcome=TRUTH_RECORDED)
    trace.command_log.note_command(9, None)
    trace.ledger.append(_ruling(4, 2_000, True))
    later = trace.capture()

    assert later.rows == sealed.rows
    assert later.commands.entries == sealed.commands.entries
    assert later.ledger.entries == sealed.ledger.entries
    assert (sealed.status.refused, sealed.commands.refused) == (0, 0)
    assert sealed.ledger.refused == 0
    assert (later.status.refused, later.commands.refused) == (1, 1)
    assert later.ledger.refused == 1


def test_every_record_decodes_back_to_itself_in_store_order():
    """Semantic decoding of every layout. Compared by repr as well as by
    equality, because 1 == 1.0 == True: a float or an int that came back in
    place of an exact int or a bool would pass equality alone."""
    capture = sealed_capture(_every_record())
    expected = [
        *((KIND_ROW, index, row) for index, row in enumerate(capture.rows)),
        *(
            (KIND_PASS, index, sample)
            for index, sample in enumerate(capture.commands.passes)
        ),
        *(
            (KIND_COMMAND, index, entry)
            for index, entry in enumerate(capture.commands.entries)
        ),
        *(
            (KIND_LEDGER, index, entry)
            for index, entry in enumerate(capture.ledger.entries)
        ),
    ]

    decoded = _records(trace_lines(capture).data)

    # The positive control: every layout is really in there, bytes included.
    assert {row[0] for row in capture.rows} == {
        EVENT_ASSOCIATION,
        EVENT_DECIMATE,
        EVENT_LIFECYCLE,
        EVENT_OUTPUT,
        EVENT_STAGE,
        EVENT_SUBSCRIPTION,
        EVENT_TRUTH,
        EVENT_VIOLATION,
    }
    assert COMMAND_RAISED in {digest for _, digest in capture.commands.entries}
    assert decoded == expected
    assert repr(decoded) == repr(expected)


def test_each_line_is_one_canonical_json_object_and_the_figures_match():
    capture = sealed_capture(_every_record())

    lines = trace_lines(capture)

    text = lines.data.decode("ascii")
    assert text.endswith("\n")
    for line in text.splitlines():
        record = json.loads(line)
        assert list(record) == ["fields", "index", "kind"]
        assert line == json.dumps(record, sort_keys=True, separators=(",", ":"))
    assert lines.sha256 == hashlib.sha256(lines.data).hexdigest()
    assert lines.lines == len(text.splitlines())
    assert lines.kinds == {
        KIND_ROW: len(capture.rows),
        KIND_PASS: 1,
        KIND_COMMAND: 3,
        KIND_LEDGER: 3,
    }


def test_an_empty_trace_is_an_empty_file_with_every_kind_counted():
    lines = trace_lines(_empty_capture())

    assert (lines.data, lines.lines) == (b"", 0)
    assert lines.sha256 == hashlib.sha256(b"").hexdigest()
    assert lines.kinds == {
        KIND_ROW: 0, KIND_PASS: 0, KIND_COMMAND: 0, KIND_LEDGER: 0
    }


class _Int(int):
    """An int that is not exactly one."""


class _Text(str):
    """A str that is not exactly one."""


@pytest.mark.parametrize(
    "value",
    [1.5, {"micros": 1}, [1, 2], bytearray(b"\x01"), _Int(3), _Text("x")],
    ids=["float", "dict", "list", "bytearray", "int_subclass", "str_subclass"],
)
def test_a_value_with_no_exact_encoding_fails_the_encode(value):
    """Nothing the stores record is one of these. One that appears is a
    defect, and coerced into the nearest JSON it would read back as a clean
    record of something else, so the encode fails instead."""
    capture = replace(
        _empty_capture(),
        rows=((EVENT_TRUTH, 1, TRUTH_RECORDED, value, None),),
    )

    with pytest.raises(TypeError):
        trace_lines(capture)


def test_big_ints_and_bools_come_back_exactly():
    capture = replace(
        _empty_capture(),
        rows=((EVENT_TRUTH, 2**70, TRUTH_RECORDED, True, None),),
    )

    [(_, _, decoded)] = _records(trace_lines(capture).data)

    assert repr(decoded) == repr((EVENT_TRUTH, 2**70, TRUTH_RECORDED, True, None))


def test_text_outside_ascii_is_escaped_and_comes_back_exactly():
    """A row's text is whatever str it was given -- a ruling's reason is
    ``str(reason)`` -- so the escape is what keeps the file ASCII."""
    text = "café → \U0001f680"
    capture = replace(
        _empty_capture(),
        rows=((EVENT_TRUTH, 1, text, None, None),),
    )

    data = trace_lines(capture).data

    assert data.isascii()
    [(_, _, decoded)] = _records(data)
    assert decoded == (EVENT_TRUTH, 1, text, None, None)


def test_bytes_are_hex_and_tuples_are_arrays():
    assert encode_field((b"\x00\xff", (1, None), "x")) == [
        {"hex": "00ff"}, [1, None], "x"
    ]


def test_every_status_field_of_every_store_is_carried():
    """A field or verdict a store gains later is carried without a change to
    the manifest; this fails if it is not."""
    capture = sealed_capture(_every_record())

    status = store_status(capture)

    assert set(status) == {"journal", "commands", "ledger", "complete"}
    assert set(status["journal"]) == _members(TraceStatus)
    assert set(status["commands"]) == _members(CommandCapture)
    assert set(status["ledger"]) == _members(LedgerCapture)
    assert status["complete"] is capture.complete
    # The records themselves are the trace file's: here they are counted.
    assert (status["commands"]["entries"], status["commands"]["passes"]) == (
        3, 1
    )
    assert status["ledger"]["entries"] == 3
    # A latched violation, and the per-epoch latch as [epoch, row] pairs.
    violation = encode_field(capture.status.first_violation)
    assert violation[0] == EVENT_VIOLATION
    assert status["journal"]["first_violation"] == violation
    assert status["journal"]["violations_by_epoch"] == [[1, violation]]
    assert status["ledger"]["rejections"] == [[REJECTED, 1]]
    assert status["ledger"]["stampless"] == 1
    json.dumps(status)  # JSON-native throughout: nothing left to a fallback


def test_the_manifest_places_every_part(monkeypatch):
    monkeypatch.setenv(DETERMINISM_TRACE_ENV, "1")
    monkeypatch.delenv(DETERMINISM_TRACE_CAPACITY_ENV, raising=False)
    capture = sealed_capture(_every_record())
    lines = trace_lines(capture)
    identity = {"harness": {"sha256": "a" * 64, "files": 3}}
    finalization = {"worker": "stopped", "epoch": 1}

    manifest = evidence_manifest(
        capture,
        trace=trace_record(lines, None),
        summary=file_record(SUMMARY_NAME, None),
        identity=identity,
        finalization=finalization,
        error=None,
        held=2,
    )

    assert list(manifest) == MANIFEST_KEYS
    assert READ_ORDER == tuple(MANIFEST_KEYS), "the order read back"
    assert (manifest["schema"], manifest["version"]) == (
        MANIFEST_SCHEMA, MANIFEST_VERSION
    )
    assert MANIFEST_VERSION == 2
    assert manifest["trace"] == {
        "file": TRACE_NAME,
        "sha256": lines.sha256,
        "lines": lines.lines,
        "kinds": lines.kinds,
        "error": None,
    }
    assert manifest["summary"] == {"file": SUMMARY_NAME, "error": None}
    assert (manifest["period_us"], manifest["capacity"]) == (
        PERIOD_US, capture.status.capacity
    )
    assert list(manifest["gate"]) == list(GATE_VARIABLES)
    assert manifest["gate"] == {
        DETERMINISM_TRACE_ENV: "1", DETERMINISM_TRACE_CAPACITY_ENV: None
    }
    assert manifest["identity"] == identity
    assert manifest["finalization"] == finalization
    assert manifest["stores"] == store_status(capture)
    # D8's verdict is the reader's alone: the writer never makes one.
    assert manifest["eligibility"] is None
    assert manifest["error"] is None
    assert manifest["held"] == 2
    json.dumps(manifest)


def test_a_manifest_without_a_capture_claims_no_figure():
    error = "RuntimeError('row store is gone')"

    manifest = evidence_manifest(
        None,
        trace=trace_record(None, None),
        summary=file_record(SUMMARY_NAME, None),
        identity=None,
        finalization={"epoch": None},
        error=error,
        held=0,
    )

    assert list(manifest) == MANIFEST_KEYS
    for key in ("period_us", "capacity", "stores", "identity"):
        assert manifest[key] is None, key
    assert manifest["trace"] == {
        "file": TRACE_NAME,
        "sha256": None,
        "lines": None,
        "kinds": None,
        "error": None,
    }
    assert manifest["error"] == error
    assert manifest["held"] == 0


def test_a_trace_file_that_is_not_in_place_is_vouched_for_by_nothing():
    lines = trace_lines(sealed_capture(_every_record()))
    error = "PermissionError(13, 'Access is denied')"

    assert trace_record(lines, error) == {
        "file": TRACE_NAME,
        "sha256": None,
        "lines": None,
        "kinds": None,
        "error": error,
    }


def test_the_summary_reduces_the_capture_it_is_given():
    """One capture feeds all three files, so the summary is made from the
    capture handed to it, never from another it takes itself."""
    trace = _every_record()
    capture = sealed_capture(trace)

    assert summarise_capture(capture, epoch=1) == summarise(trace, epoch=1)
    fewer = replace(capture, rows=capture.rows[:1])
    assert summarise_capture(fewer, epoch=1)["rows"] == 1
    assert summarise(trace, epoch=1)["rows"] > 1


if __name__ == "__main__":  # pragma: no cover
    pytest.main([__file__])
