"""The reader: a case's files read back into the capture they were made from.

``read_evidence`` holds a manifest to D8.1 of
the LANDING2 step-1 plan: its keys, schema and
version, and the trace file's sha256, line count and count per kind. It also
holds the two checks delivery step 6's review left it: a manifest it cannot
read is refused, and ``held`` is required, never read as zero. Then it
rebuilds the capture from the trace file's records and the manifest's store
figures, and refuses a manifest whose figures the records do not reproduce
exactly, types included. It never repairs.
"""

from __future__ import annotations

import dis
import json
import sys
from collections.abc import Callable
from dataclasses import replace
from pathlib import Path
from types import CodeType
from typing import Any

import pytest

from navpy.modules.vision.sim.determinism_admission_ledger import AdmissionLedger
from navpy.modules.vision.sim.determinism_evidence import (
    MANIFEST_KEYS,
    MANIFEST_NAME,
    SUMMARY_NAME,
    TRACE_NAME,
    evidence_manifest,
    file_record,
    sealed_capture,
    trace_record,
)
from navpy.modules.vision.sim.determinism_events import (
    ASSOCIATION_COMMITTED,
    EVENT_VIOLATION,
    STAGE_STAGED,
    TRUTH_RECORDED,
    VIOLATION_SOURCE_REPEATED,
)
from navpy.modules.vision.sim.determinism_reader import (
    read_case,
    read_evidence,
)
from navpy.modules.vision.sim.determinism_row_log import BoundedRowLog
from navpy.modules.vision.sim.determinism_trace import (
    DeterminismTrace,
    TraceCapture,
)
from navpy.modules.vision.sim.determinism_trace_decode import EvidenceRefused
from tests.modules.vision.determinism_case_factory import (
    IDENTITY,
    PERIOD_US,
    case_of,
    eligible_capture,
    eligible_trace,
    evidence_bytes,
    finalization,
    ruling,
)


def test_a_case_reads_back_as_the_capture_it_was_made_from() -> None:
    capture = eligible_capture()
    manifest_data, trace_data = evidence_bytes(capture)
    case = read_evidence(manifest_data, trace_data)
    assert case.capture == capture
    assert case.manifest == json.loads(manifest_data)
    assert list(case.manifest) == list(MANIFEST_KEYS)


def _every_figure_set() -> Any:
    """A capture with every status figure away from its default: overflow in
    each store, a violation, a callback fault, failures, refusals, a drain,
    an unreadable payload, a gap, a step back and a stampless admission."""
    trace = DeterminismTrace(PERIOD_US, capacity=3)
    for source_s in (10.04, 10.04, 10.06):  # the second repeats: a violation
        trace.record_association(
            epoch=1, outcome=ASSOCIATION_COMMITTED, source_s=source_s
        )
    trace.record_truth(epoch=1, outcome=TRUTH_RECORDED)  # past capacity
    trace.journal.count_callback_fault()
    for number, stamp in ((1, 10_040), (3, 10_000), (4, None), (5, 9_000)):
        trace.ledger.append(ruling(number, stamp))
    for iteration in (1, 2, 3, 4):
        trace.command_log.note_pass(iteration, None, None)
        trace.command_log.note_command(iteration, None)
    capture = sealed_capture(trace)
    return replace(
        capture,
        status=replace(
            capture.status,
            failed=True,
            drained=True,
            payload_unreadable=True,
            refused=2,
        ),
        commands=replace(capture.commands, failed=True, refused=1),
        ledger=replace(capture.ledger, failed=True, refused=3),
    )


def test_every_figure_of_every_store_reads_back_exactly() -> None:
    capture = _every_figure_set()
    assert capture.status.first_violation is not None
    assert capture.ledger.gapped and capture.ledger.discontinuous
    assert capture.ledger.stampless == 1
    assert case_of(capture).capture == capture


def _bytes(edit: Callable[[dict], object]) -> tuple[bytes, bytes]:
    """The eligible case's files, the manifest edited as a JSON object."""
    manifest_data, trace_data = evidence_bytes(eligible_capture())
    manifest = json.loads(manifest_data)
    edit(manifest)
    return json.dumps(manifest, indent=2).encode("utf-8"), trace_data


def _raw(text: Callable[[str], str]) -> tuple[bytes, bytes]:
    """The eligible case's files, the manifest edited as text."""
    manifest_data, trace_data = evidence_bytes(eligible_capture())
    return text(manifest_data.decode("utf-8")).encode("utf-8"), trace_data


def _set(path: tuple[str, ...], value: Any) -> Callable[[dict], None]:
    def edit(manifest: dict) -> None:
        for key in path[:-1]:
            manifest = manifest[key]
        manifest[path[-1]] = value

    return edit


def _delete(*path: str) -> Callable[[dict], None]:
    def edit(manifest: dict) -> None:
        for key in path[:-1]:
            manifest = manifest[key]
        del manifest[path[-1]]

    return edit


def _add(path: tuple[str, ...], amount: int) -> Callable[[dict], None]:
    def edit(manifest: dict) -> None:
        for key in path[:-1]:
            manifest = manifest[key]
        manifest[path[-1]] += amount

    return edit


def _reordered(manifest: dict) -> None:
    items = list(manifest.items())
    manifest.clear()
    manifest.update(reversed(items))


def _no_capture() -> tuple[bytes, bytes]:
    """The manifest the writer makes when the capture itself failed."""
    manifest = evidence_manifest(
        None,
        trace=trace_record(None, None),
        summary=file_record(SUMMARY_NAME, None),
        identity=IDENTITY,
        finalization=finalization(),
        error="RuntimeError('the capture failed')",
        held=0,
    )
    return json.dumps(manifest, indent=2).encode("utf-8"), b""


def _tampered_trace() -> tuple[bytes, bytes]:
    manifest_data, trace_data = evidence_bytes(eligible_capture())
    return manifest_data, trace_data.replace(b"10020000", b"10020001", 1)


def _two_kinds_swapped(manifest: dict) -> None:
    kinds = manifest["trace"]["kinds"]
    kinds["row"] -= 1
    kinds["pass"] += 1


def _journal(complete: bool, **figures: Any) -> Callable[[dict], None]:
    """The journal's figures edited, and the derived ``complete`` flags
    set as those figures make them, so only the edit can be refused."""
    def edit(manifest: dict) -> None:
        manifest["stores"]["journal"].update(figures, complete=complete)
        manifest["stores"]["complete"] = complete

    return edit


def _capacity(capacity: int) -> Callable[[dict], None]:
    def edit(manifest: dict) -> None:
        manifest["capacity"] = capacity
        manifest["stores"]["journal"]["capacity"] = capacity

    return edit


def _unflagged_unreadable() -> tuple[bytes, bytes]:
    """A frame the recorder could not read, held as a STAGE row, and the
    journal's figures edited to say none was, the derived ones to match."""
    trace = eligible_trace()
    trace.record_stage(epoch=1, outcome=STAGE_STAGED, frame=object())
    manifest_data, trace_data = evidence_bytes(sealed_capture(trace))
    manifest = json.loads(manifest_data)
    _journal(True, payload_unreadable=False)(manifest)
    return json.dumps(manifest, indent=2).encode("utf-8"), trace_data


# Deeper than Python can walk recursively, not deeper than its JSON parser
# reads.
NESTED = "[" * 600 + "0" + "]" * 600


def _unlatched_violation(**latches: Any) -> Callable[[], tuple[bytes, bytes]]:
    """A violation held as a row, the source having stepped back to 10.0 s,
    and the journal's latches for it edited away."""
    def files() -> tuple[bytes, bytes]:
        trace = eligible_trace()
        trace.record_association(
            epoch=1, outcome=ASSOCIATION_COMMITTED, source_s=10.0
        )
        manifest_data, trace_data = evidence_bytes(sealed_capture(trace))
        manifest = json.loads(manifest_data)
        journal = manifest["stores"]["journal"]
        assert journal["first_violation"] is not None
        journal.update(latches)
        return json.dumps(manifest, indent=2).encode("utf-8"), trace_data

    return files


def _dropping_ledger() -> tuple[dict, bytes]:
    """The manifest, as a dict, and the trace file of a ledger that held
    rulings 1..3 and dropped the fourth past its capacity."""
    trace = DeterminismTrace(PERIOD_US, capacity=3)
    for number in (1, 2, 3, 4):
        trace.ledger.append(ruling(number, 10_000 + number))
    manifest_data, trace_data = evidence_bytes(sealed_capture(trace))
    manifest = json.loads(manifest_data)
    ledger = manifest["stores"]["ledger"]
    assert (ledger["dropped"], ledger["observed"]) == (1, 4)
    return manifest, trace_data


def _hidden_ledger_drop() -> tuple[bytes, bytes]:
    """That ledger's figures edited to say nothing was dropped, the derived
    ones to match. Its latest ruling observed is still the fourth."""
    manifest, trace_data = _dropping_ledger()
    ledger = manifest["stores"]["ledger"]
    ledger.update(dropped=0, overflowed=False, incomplete=False)
    manifest["stores"]["complete"] = manifest["stores"]["journal"]["complete"]
    return json.dumps(manifest, indent=2).encode("utf-8"), trace_data


def _negative_ruling_past_a_drop() -> tuple[bytes, bytes]:
    """That ledger's latest ruling observed edited to -1. Past a drop its
    entries say nothing of that figure, so only the count check can refuse
    it."""
    manifest, trace_data = _dropping_ledger()
    manifest["stores"]["ledger"]["observed"] = -1
    return json.dumps(manifest, indent=2).encode("utf-8"), trace_data


def _one_past_the_capacity(
    add: Callable[[DeterminismTrace], object],
) -> Callable[[], tuple[bytes, bytes]]:
    """An eligible trace's nine rows, one other store made to hold ten
    records by ``add``, and the capacity the three share edited to nine:
    the rows within it and that store one past it, as no writer leaves
    them."""
    def files() -> tuple[bytes, bytes]:
        trace = eligible_trace()
        add(trace)
        capture = sealed_capture(trace)
        held = (capture.ledger.entries, capture.commands.entries,
                capture.commands.passes)
        assert (len(capture.rows), max(map(len, held))) == (9, 10)
        manifest_data, trace_data = evidence_bytes(capture)
        manifest = json.loads(manifest_data)
        _capacity(9)(manifest)
        return json.dumps(manifest, indent=2).encode("utf-8"), trace_data

    return files


def _ten_rulings(trace: DeterminismTrace) -> None:
    for number in range(4, 11):
        trace.ledger.append(ruling(number, None, accepted=False, reason="stale"))


def _ten_passes(trace: DeterminismTrace) -> None:
    for number in range(3, 11):
        trace.command_log.note_pass(number, 3, 3)


def _ten_commands(trace: DeterminismTrace) -> None:
    for number in range(3, 12):
        trace.command_log.note_command(number, None)


def _journal_drop_claimed() -> tuple[bytes, bytes]:
    """An eligible capture's journal said to have dropped a row, holding
    nine under a capacity of 150,000: no store drops a record but when
    full, and none shrinks but by a drain."""
    capture = eligible_capture()
    status = replace(capture.status, dropped=1, overflowed=True)
    return evidence_bytes(replace(capture, status=status))


def _ledger_drop_claimed() -> tuple[bytes, bytes]:
    """Its ledger said to have dropped a ruling, holding three."""
    capture = eligible_capture()
    return evidence_bytes(replace(capture, ledger=replace(capture.ledger, dropped=1)))


def _command_drop_claimed() -> tuple[bytes, bytes]:
    """Its command log said to have dropped a record, holding one command
    and two pass samples."""
    capture = eligible_capture()
    commands = replace(capture.commands, dropped=1)
    status = replace(capture.status, commands_incomplete=True)
    return evidence_bytes(replace(capture, commands=commands, status=status))


# A violation row no trace below holds.
OTHER = (EVENT_VIOLATION, 1, VIOLATION_SOURCE_REPEATED, (10_040_000, 10_040_000))


def _stage_unreadable(trace: DeterminismTrace) -> None:
    """A frame the recorder cannot read, staged: the payload latch."""
    trace.record_stage(epoch=1, outcome=STAGE_STAGED, frame=object())


def _step_back(trace: DeterminismTrace) -> None:
    """The source stepped back to 10.0 s: a violation, latched and held."""
    trace.record_association(epoch=1, outcome=ASSOCIATION_COMMITTED, source_s=10.0)


def _recorded(
    *records: Callable[[DeterminismTrace], object],
    capacity: int | None = None,
    drained: bool = False,
) -> TraceCapture:
    """An eligible trace given ``records``, drained if ``drained``."""
    trace = eligible_trace(capacity=capacity)
    for record in records:
        record(trace)
    if drained:
        trace.journal.drain()
    return sealed_capture(trace)


def _latched(
    capture: Callable[[], TraceCapture], latches: Callable[[Any], dict]
) -> Callable[[], tuple[bytes, bytes]]:
    """``capture``'s journal latches edited by ``latches``. Nothing was
    dropped, drained or failed, so the latches must be what its rows show."""
    def files() -> tuple[bytes, bytes]:
        made = capture()
        status = replace(made.status, **latches(made.status))
        return evidence_bytes(replace(made, status=status))

    return files


def _past_a_drop(
    record: Callable[[DeterminismTrace], object], **latches: Any
) -> Callable[[], tuple[bytes, bytes]]:
    """An eligible trace given ``record``, at a capacity of exactly its rows,
    then one row more, dropped; its latches edited to ``latches``. Past a
    drop a latch may name a row no longer held, but a HELD row still has to
    be latched: only the one-way checks can refuse these."""
    def files() -> tuple[bytes, bytes]:
        rows = len(_recorded(record).rows)
        trace = eligible_trace(capacity=rows)
        record(trace)
        trace.record_truth(epoch=1, outcome=TRUTH_RECORDED)
        capture = sealed_capture(trace)
        assert (capture.status.rows, capture.status.dropped) == (rows, 1)
        status = replace(capture.status, **latches)
        return evidence_bytes(replace(capture, status=status))

    return files


def _fewer_passes_counted() -> tuple[bytes, bytes]:
    """Its command log said to have counted one pass, holding two samples:
    ``note_pass`` counts a pass before it holds or drops its sample."""
    capture = eligible_capture()
    return evidence_bytes(
        replace(capture, commands=replace(capture.commands, iterations=1))
    )


REFUSED: dict[str, Callable[[], tuple[bytes, bytes]]] = {
    "not JSON": lambda: _raw(lambda text: text[:-3]),
    "not UTF-8": lambda: (b"\xff\xfe{}", b""),
    "not an object": lambda: _raw(lambda text: "[" + text + "]"),
    "NaN": lambda: _raw(lambda text: text.replace('"held": 0', '"held": NaN')),
    "NaN where no rule reads": lambda: _raw(
        lambda text: text.replace('"pid": 1\n', '"pid": NaN\n')
    ),
    "a duplicate key": lambda: _raw(
        lambda text: text.replace('"held": 0', '"held": 0, "held": 0')
    ),
    "held missing": lambda: _bytes(_delete("held")),
    "held as text": lambda: _bytes(_set(("held",), "0")),
    "held negative": lambda: _bytes(_set(("held",), -1)),
    "held a bool": lambda: _bytes(_set(("held",), False)),
    "held a float": lambda: _bytes(_set(("held",), 0.0)),
    "held null": lambda: _bytes(_set(("held",), None)),
    "an extra key": lambda: _bytes(_set(("extra",), None)),
    "keys out of order": lambda: _bytes(_reordered),
    "another schema": lambda: _bytes(_set(("schema",), "navpy.other")),
    "another version": lambda: _bytes(_set(("version",), 3)),
    "the version as text": lambda: _bytes(_set(("version",), "1")),
    "a verdict claimed": lambda: _bytes(_set(("eligibility",), "ELIGIBLE")),
    "no capture": _no_capture,
    "the trace file not written": lambda: _bytes(
        _set(("trace",), trace_record(None, "OSError('disk full')"))
    ),
    "the trace file missing": lambda: (evidence_bytes(eligible_capture())[0], None),
    "the trace file changed": _tampered_trace,
    "the sha256 in upper case": lambda: _bytes(
        lambda manifest: manifest["trace"].update(
            sha256=manifest["trace"]["sha256"].upper()
        )
    ),
    "a line count off": lambda: _bytes(_add(("trace", "lines"), 1)),
    "a kind's count off": lambda: _bytes(_two_kinds_swapped),
    "a kind missing": lambda: _bytes(_delete("trace", "kinds", "ledger")),
    "a store figure the records contradict": lambda: _bytes(
        _set(("stores", "ledger", "gapped"), True)
    ),
    "a store figure of another type": lambda: _bytes(
        _set(("stores", "journal", "failed"), 0)
    ),
    "an int figure given as a bool": lambda: _bytes(
        _set(("stores", "journal", "dropped"), False)
    ),
    "a derived figure of another type": lambda: _bytes(
        _set(("stores", "complete"), 1)
    ),
    "a record count off": lambda: _bytes(_add(("stores", "commands", "entries"), 1)),
    "the row count off": lambda: _bytes(_add(("stores", "journal", "rows"), 1)),
    "a store figure missing": lambda: _bytes(
        _delete("stores", "commands", "refused")
    ),
    "a store figure extra": lambda: _bytes(_set(("stores", "ledger", "extra"), 0)),
    "the capacity off": lambda: _bytes(_add(("capacity",), 1)),
    "the period a float": lambda: _bytes(_set(("period_us",), 20000.0)),
    "the summary named wrong": lambda: _bytes(
        _set(("summary",), file_record("other.json", None))
    ),
    "the summary's record with a key extra": lambda: _bytes(
        _set(("summary", "extra"), None)
    ),
    "the summary's record not an object": lambda: _bytes(
        _set(("summary",), SUMMARY_NAME)
    ),
    # Review round 1: figures the journal's rows, or its own counts, show
    # false; a negative count; a non-finite number; too deep a value.
    "an unreadable payload the journal does not flag": _unflagged_unreadable,
    "dropped rows with no overflow": lambda: _bytes(_journal(True, dropped=1)),
    "an overflow with no dropped row": lambda: _bytes(
        _journal(False, overflowed=True)
    ),
    "more rows than the capacity": lambda: _bytes(_capacity(8)),
    "the commands' completeness misstated": lambda: _bytes(
        _journal(False, commands_incomplete=True)
    ),
    "a negative count": lambda: _bytes(_set(("stores", "journal", "dropped"), -1)),
    # The count check alone: the journal at its capacity, so review round
    # 4's fix, which refuses the case above too, holds no drop against it.
    "a negative count in a full journal": lambda: _bytes(
        lambda manifest: [
            edit(manifest)
            for edit in (_capacity(9), _set(("stores", "journal", "dropped"), -1))
        ]
    ),
    "a negative ruling": lambda: _bytes(_set(("stores", "ledger", "observed"), -1)),
    "a number too large to be finite": lambda: _raw(
        lambda text: text.replace('"pid": 1\n', '"pid": 1e999\n')
    ),
    "a figure nested deeper than can be read": lambda: _raw(
        lambda text: text.replace(
            '"first_violation": null', '"first_violation": ' + NESTED
        )
    ),
    "a held violation with no first violation latched": _unlatched_violation(
        first_violation=None
    ),
    "a held violation not latched for its epoch": _unlatched_violation(
        violations_by_epoch=[]
    ),
    # Review round 2: ledger figures its entries show false, nothing having
    # been dropped and nothing faulted.
    "the latest ruling observed misstated": lambda: _bytes(
        _set(("stores", "ledger", "observed"), 2)
    ),
    "the latest ruling admitted misstated": lambda: _bytes(
        _set(("stores", "ledger", "admitted"), 2)
    ),
    "a ledger's drop edited away": _hidden_ledger_drop,
    # And a negative ruling where only the count check can see it: past a
    # drop, the entries say nothing of the latest ruling observed.
    "a negative ruling past a drop": _negative_ruling_past_a_drop,
    # Review round 3: the capacity the three stores share, held to the
    # records of each, not the journal's rows alone.
    "a capacity below the ledger's entries": _one_past_the_capacity(_ten_rulings),
    "a capacity below the pass samples": _one_past_the_capacity(_ten_passes),
    "a capacity below the command entries": _one_past_the_capacity(_ten_commands),
    # Review round 4: a drop claimed by a store below its capacity, which no
    # store makes.
    "a journal drop claimed below the capacity": _journal_drop_claimed,
    "a ledger drop claimed below the capacity": _ledger_drop_claimed,
    "a command drop claimed below the capacity": _command_drop_claimed,
    # Review round 5: a latch no held row shows, where nothing was dropped,
    # drained or failed; and a pass count below the pass samples held.
    "an unreadable payload latched that no row shows": _latched(
        eligible_capture, lambda status: {"payload_unreadable": True}
    ),
    "a violation latched that no row shows": _latched(
        eligible_capture,
        lambda status: {"first_violation": OTHER, "violations_by_epoch": {1: OTHER}},
    ),
    "an epoch latched that no held row shows": _latched(
        lambda: _recorded(_step_back),
        lambda status: {
            "violations_by_epoch": {**status.violations_by_epoch, 2: OTHER}
        },
    ),
    "a first violation other than the first held": _latched(
        lambda: _recorded(_step_back), lambda status: {"first_violation": OTHER}
    ),
    "an epoch's latch other than its first held row": _latched(
        lambda: _recorded(_step_back),
        lambda status: {"violations_by_epoch": {1: OTHER}},
    ),
    "fewer passes counted than held": _fewer_passes_counted,
    # And past a drop, a held row its latches do not show.
    "an unreadable payload held unflagged past a drop": _past_a_drop(
        _stage_unreadable, payload_unreadable=False
    ),
    "a held violation with no first violation, past a drop": _past_a_drop(
        _step_back, first_violation=None
    ),
    "a held violation unlatched for its epoch, past a drop": _past_a_drop(
        _step_back, violations_by_epoch={}
    ),
}


@pytest.mark.parametrize("name", sorted(REFUSED))
def test_a_manifest_that_does_not_vouch_for_its_files_is_refused(
    name: str,
) -> None:
    manifest_data, trace_data = REFUSED[name]()
    with pytest.raises(EvidenceRefused):
        read_evidence(manifest_data, trace_data)


def _interrupted_at_each_line(
    code: CodeType, record: Callable[[DeterminismTrace], object], capacity: int
) -> list[tuple[int, bool, Any]]:
    """For each line ``code`` starts, a fresh trace of ``capacity`` whose
    ``record`` is interrupted as it reaches that line, then sealed: each
    capture an interrupt there leaves, and whether it was reached. It is
    raised inside ``code``'s own frame, never on the exit line of a with
    statement, which would leave a lock held (test_admission_tap)."""
    results = []
    lines = {line for _, line in dis.findlinestarts(code) if line is not None}
    for line in sorted(lines):
        trace = DeterminismTrace(PERIOD_US, capacity=capacity)

        def land(frame: Any, event: str, arg: object, line: int = line) -> Any:
            if frame.f_code is code and event == "line" and frame.f_lineno == line:
                raise KeyboardInterrupt(line)
            return land

        reached = False
        previous = sys.gettrace()
        sys.settrace(land)
        try:
            record(trace)
        except KeyboardInterrupt:
            reached = True
        finally:
            sys.settrace(previous)
        results.append((line, reached, sealed_capture(trace)))
    return results


def test_an_interrupt_anywhere_in_a_dropping_append_leaves_a_readable_capture(
) -> None:
    """Review round 2: an interrupt landing between the count of a dropped
    row and its overflow flag left a capture no log reports, which the
    reader refused. Every capture an interrupt in the append leaves has to
    be failed, and read."""
    results = _interrupted_at_each_line(
        BoundedRowLog.append.__code__,
        lambda trace: trace.record_truth(epoch=1, outcome=TRUTH_RECORDED),
        capacity=0,
    )
    assert sum(reached for _, reached, _ in results) >= 3
    for line, reached, capture in results:
        assert capture.status.failed is reached, line
        assert read_evidence(*evidence_bytes(capture)).capture == capture, line


def _ledger_change() -> CodeType:
    """The body of ``AdmissionLedger.append``: its one recording."""
    return next(
        const
        for const in AdmissionLedger.append.__code__.co_consts
        if isinstance(const, CodeType) and const.co_name == "change"
    )


@pytest.mark.parametrize("capacity", [0, 3])
def test_an_interrupt_anywhere_in_a_ledger_append_leaves_a_readable_capture(
    capacity: int,
) -> None:
    """Review round 2: the ledger's live figures advance before its entry is
    held or dropped, and an interrupt between leaves it failed. So the
    reader may hold those figures to the entries only where nothing was
    dropped and nothing faulted."""
    results = _interrupted_at_each_line(
        _ledger_change(),
        lambda trace: trace.ledger.append(ruling(1, 10_001)),
        capacity=capacity,
    )
    assert sum(reached for _, reached, _ in results) >= 4
    for line, reached, capture in results:
        assert capture.ledger.failed is reached, line
        assert read_evidence(*evidence_bytes(capture)).capture == capture, line


def test_a_trace_made_with_a_negative_capacity_reads_back() -> None:
    """Review round 2: a budget below zero holds nothing, as zero does, in
    every store alike. The journal kept -1, which no count is."""
    trace = DeterminismTrace(PERIOD_US, capacity=-1)
    trace.record_truth(epoch=1, outcome=TRUTH_RECORDED)
    trace.ledger.append(ruling(1, 10_001))
    capture = sealed_capture(trace)
    assert capture.status.capacity == 0
    assert case_of(capture).capture == capture


def test_a_ledger_whose_last_ruling_was_rejected_reads_back() -> None:
    """Review round 2: the latest ruling admitted is the last one accepted,
    not the last one held, so a rejection after it leaves it where it was."""
    trace = eligible_trace()
    trace.ledger.append(ruling(4, None, accepted=False, reason="stale"))
    capture = sealed_capture(trace)
    assert (capture.ledger.observed, capture.ledger.admitted) == (4, 3)
    assert case_of(capture).capture == capture


@pytest.mark.parametrize("given", [3, 4])
def test_a_trace_whose_stores_each_reach_the_capacity_reads_back(given: int) -> None:
    """Review round 3: every store holds at most the capacity the three
    share, so one that reaches it, or drops past it, is no contradiction."""
    trace = DeterminismTrace(PERIOD_US, capacity=3)
    for number in range(1, given + 1):
        trace.ledger.append(ruling(number, 10_000 + number))
        trace.command_log.note_pass(number, number, number)
        trace.command_log.note_command(number, None)
    capture = sealed_capture(trace)
    held = (capture.ledger.entries, capture.commands.entries, capture.commands.passes)
    assert [len(records) for records in held] == [3, 3, 3]
    assert case_of(capture).capture == capture


@pytest.mark.parametrize("store", ["passes", "commands"])
def test_a_command_log_dropping_from_one_store_reads_back(store: str) -> None:
    """Review round 4: the command log counts one drop for its pass samples
    and its entries, so a drop needs only one of the two full."""
    trace = DeterminismTrace(PERIOD_US, capacity=3)
    for number in range(1, 5):
        if store == "passes":
            trace.command_log.note_pass(number, None, None)
        else:
            trace.command_log.note_command(number, None)
    capture = sealed_capture(trace)
    assert capture.commands.dropped == 1
    assert case_of(capture).capture == capture


def test_an_interrupt_anywhere_in_a_drain_leaves_a_readable_capture() -> None:
    """Review round 4: a journal that dropped a row holds its capacity until
    it is drained, and an interrupt inside the drain can leave it empty and
    not yet marked drained. It is failed then, and read."""
    def overflow_and_drain(trace: DeterminismTrace) -> None:
        for _ in range(4):
            trace.record_truth(epoch=1, outcome=TRUTH_RECORDED)
        trace.journal.drain()

    results = _interrupted_at_each_line(
        BoundedRowLog.drain.__code__, overflow_and_drain, capacity=3
    )
    assert sum(reached for _, reached, _ in results) >= 3
    assert any(
        capture.status.rows == 0 and not capture.status.drained
        for _, _, capture in results
    )
    for line, reached, capture in results:
        assert capture.status.dropped == 1, line
        assert capture.status.failed is reached, line
        assert read_evidence(*evidence_bytes(capture)).capture == capture, line


LATCHES_LEFT: dict[str, Callable[[], TraceCapture]] = {
    "an unreadable payload dropped": lambda: _recorded(_stage_unreadable, capacity=9),
    "a violation dropped": lambda: _recorded(_step_back, capacity=9),
    "both latched, then drained": lambda: _recorded(
        _stage_unreadable, _step_back, drained=True
    ),
}


@pytest.mark.parametrize("name", sorted(LATCHES_LEFT))
def test_a_latch_whose_row_is_no_longer_held_reads_back(name: str) -> None:
    """Review round 5: a latch is set before its row is held, and outlives
    it, so a row dropped past the capacity or drained leaves a latch no held
    row shows, and that is no contradiction."""
    capture = LATCHES_LEFT[name]()
    status = capture.status
    assert status.payload_unreadable or status.first_violation is not None
    assert status.dropped or status.drained
    assert not any(row[0] == EVENT_VIOLATION for row in capture.rows)
    assert case_of(capture).capture == capture


def _twice_stepped_back(trace: DeterminismTrace) -> None:
    for source_s in (10.04, 10.0):
        trace.record_association(
            epoch=1, outcome=ASSOCIATION_COMMITTED, source_s=source_s
        )


def test_a_journal_holding_two_violations_in_one_leg_reads_back() -> None:
    """Review round 5: both latches name the FIRST violation, trace-wide and
    for its epoch, and a later one in the leg is held beside it."""
    capture = _recorded(_twice_stepped_back)
    violations = [row for row in capture.rows if row[0] == EVENT_VIOLATION]
    assert len(violations) == 2
    assert capture.status.first_violation == violations[0]
    assert case_of(capture).capture == capture


@pytest.mark.parametrize("latch", ["payload", "violation"])
def test_an_interrupt_anywhere_in_a_latching_record_leaves_a_readable_capture(
    latch: str,
) -> None:
    """Review round 5: each latch is set before its row is held, so an
    interrupt between the two leaves a latch no held row shows. The journal
    is failed then, and read."""
    code, record = {
        "payload": (BoundedRowLog.append.__code__, _stage_unreadable),
        "violation": (BoundedRowLog.note_violation.__code__, _twice_stepped_back),
    }[latch]
    results = _interrupted_at_each_line(code, record, capacity=3)
    assert sum(reached for _, reached, _ in results) >= 3
    assert any(
        (capture.status.payload_unreadable and not capture.status.rows)
        or (
            capture.status.first_violation is not None
            and not any(row[0] == EVENT_VIOLATION for row in capture.rows)
        )
        for _, _, capture in results
    )
    for line, reached, capture in results:
        assert capture.status.failed is reached, line
        assert read_evidence(*evidence_bytes(capture)).capture == capture, line


def _written(directory: Path, *, manifest: bool = True, trace: bool = True) -> None:
    manifest_data, trace_data = evidence_bytes(eligible_capture())
    if manifest:
        (directory / MANIFEST_NAME).write_bytes(manifest_data)
    if trace:
        (directory / TRACE_NAME).write_bytes(trace_data)


def test_a_case_directory_reads_as_its_files_do(tmp_path: Path) -> None:
    _written(tmp_path)
    assert read_case(tmp_path).capture == eligible_capture()


def test_a_missing_manifest_is_refused(tmp_path: Path) -> None:
    """Delivery step 6's review: nothing can be counted in a manifest that
    was never written, so its absence is the refusal."""
    _written(tmp_path, manifest=False)
    with pytest.raises(EvidenceRefused, match="missing"):
        read_case(tmp_path)


def test_an_unreadable_manifest_is_refused(tmp_path: Path) -> None:
    (tmp_path / MANIFEST_NAME).mkdir()
    with pytest.raises(EvidenceRefused, match="cannot be read"):
        read_case(tmp_path)


def test_a_missing_trace_file_is_refused(tmp_path: Path) -> None:
    _written(tmp_path, trace=False)
    with pytest.raises(EvidenceRefused, match="missing"):
        read_case(tmp_path)
