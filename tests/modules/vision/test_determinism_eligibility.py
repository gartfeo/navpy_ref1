"""D8: a case is ELIGIBLE only if all thirteen rules hold.

Every variant departs from the factory's ``eligible_trace``, which meets them
all, and is read back through the writer's own functions, so its checksums
are valid: what it breaks is a rule, never the file. Each is judged with
exactly the rules it breaks named, no fewer and no more. The rules are D8 of
the LANDING2 step-1 plan; rule 1, the manifest
itself, is the reader's (``test_determinism_reader``).
"""

from __future__ import annotations

import sys
from collections.abc import Callable
from dataclasses import replace
from types import SimpleNamespace
from typing import Any

import pytest

from navpy.modules.vision.sim.determinism_command_log import COMMAND_UNREADABLE
from navpy.modules.vision.sim.determinism_events import (
    ASSOCIATION_FENCED,
    ASSOCIATION_RULING,
    DISCARD_PUBLISH_OVERWRITTEN,
    EVENT_VIOLATION,
    OUTPUT_WORKER_ITERATION,
    SLOT_PAIRS,
    STAGE_STAGED,
    SUBSCRIPTION_RULING,
    TRUTH_RECORDED,
    VIOLATION_SOURCE_REPEATED,
)
from navpy.modules.vision.sim.determinism_eligibility import (
    ELIGIBLE,
    INSPECTABLE,
    judge,
    refusal,
)
from navpy.modules.vision.sim.determinism_evidence import (
    EVIDENCE_STEP,
    SOURCE_CLOSE_STEP,
    sealed_capture,
)
from navpy.modules.vision.sim.determinism_ledger_rules import owed_rulings
from navpy.modules.vision.sim.determinism_reader import CaseEvidence
from navpy.modules.vision.sim.determinism_row_layouts import (
    discard_row,
    stage_row,
)
from navpy.modules.vision.sim.determinism_trace import (
    DeterminismTrace,
    TraceCapture,
)
from navpy.modules.vision.sim.determinism_trace_decode import EvidenceRefused
from tests.modules.vision.determinism_case_factory import (
    IDENTITY,
    PERIOD_US,
    admission,
    case_of,
    edited_case,
    eligible_capture,
    eligible_trace,
    ruling,
    with_rows,
)

# Where ``eligible_trace`` records each row.
OPENED, ACTIVATED, TRUTH, OUTPUT, CLOSED, ENDED = 0, 1, 4, 5, 7, 8
ASSOCIATIONS = (2, 3, 6)  # rulings 1, 2 and 3
FRAME = SimpleNamespace(
    pixel=SimpleNamespace(
        u_px=1.5, v_px=2.5, aircraft_pitch_deg=-4.0,
        aircraft_roll_deg=3.0, source_timestamp_s=10.04,
    )
)
# The widest int the reader takes, since Python reads none wider from text;
# 4300 digits where the limit is off.
WIDEST = int("9" * (getattr(sys, "get_int_max_str_digits", lambda: 0)() or 4300))


def _with(row: tuple, field: int, value: Any) -> tuple:
    return row[:field] + (value,) + row[field + 1:]


def _rows(edit: Callable[[list[tuple]], None]) -> TraceCapture:
    capture = eligible_capture()
    rows = list(capture.rows)
    edit(rows)
    return with_rows(capture, rows)


def _field(index: int, field: int, value: Any) -> TraceCapture:
    def edit(rows: list[tuple]) -> None:
        rows[index] = _with(rows[index], field, value)

    return _rows(edit)


def _without(*indexes: int) -> TraceCapture:
    return _rows(lambda rows: [rows.pop(i) for i in sorted(indexes, reverse=True)])


def _swapped(first: int, second: int) -> TraceCapture:
    def edit(rows: list[tuple]) -> None:
        rows[first], rows[second] = rows[second], rows[first]

    return _rows(edit)


def _status(**changes: Any) -> TraceCapture:
    capture = eligible_capture()
    return replace(capture, status=replace(capture.status, **changes))


def _commands(**changes: Any) -> TraceCapture:
    capture = eligible_capture()
    commands = replace(capture.commands, **changes)
    status = replace(capture.status, commands_incomplete=commands.incomplete)
    return replace(capture, commands=commands, status=status)


def _ledger(**changes: Any) -> TraceCapture:
    capture = eligible_capture()
    return replace(capture, ledger=replace(capture.ledger, **changes))


def _overflowed(fill: Callable[[DeterminismTrace], object]) -> CaseEvidence:
    """``eligible_trace`` at a capacity of its nine rows, then given one
    record past it in a store by ``fill``: a real overflow. No store drops
    a record but when full, and the reader refuses a drop claimed by one
    that is not (review round 4)."""
    trace = eligible_trace(capacity=9)
    fill(trace)
    return case_of(sealed_capture(trace))


def _unreadable_payload() -> TraceCapture:
    """A frame the recorder could not read, held as a STAGE row."""
    trace = eligible_trace()
    trace.record_stage(epoch=1, outcome=STAGE_STAGED, frame=object())
    return sealed_capture(trace)


def _rulings_to_ten(trace: DeterminismTrace) -> None:
    """Rulings 4..10, rejected after the window: the tenth is dropped."""
    for number in range(4, 11):
        trace.ledger.append(ruling(number, None, accepted=False, reason="stale"))


def _passes_to_ten(trace: DeterminismTrace) -> None:
    """Passes 3..10, each issuing a command, as the worker notes them: the
    tenth pass sample is dropped, and its command held."""
    for number in range(3, 11):
        trace.command_log.note_pass(number, 3, 3)
        trace.command_log.note_command(number, None)


def _after_close(entry: tuple) -> CaseEvidence:
    """A fourth ruling, made after the window closed, so no row is owed it."""
    capture = eligible_capture()
    # Its live figures as the ledger's append would leave them.
    ledger = replace(
        capture.ledger,
        entries=capture.ledger.entries + (entry,),
        observed=entry[0],
        admitted=entry[0] if entry[2] else capture.ledger.admitted,
    )
    return case_of(replace(capture, ledger=ledger), admission=admission(end=4))


def _rejected_cutoff() -> CaseEvidence:
    """A fourth ruling, REJECTED, inside the window, and a third pass whose
    admitted cutoff names it. The last ruling ACCEPTED at or below its
    observed cutoff is the third."""
    capture = _field(CLOSED, SUBSCRIPTION_RULING, 4)
    commands = replace(
        capture.commands,
        passes=capture.commands.passes + ((3, 4, 4),),
        iterations=3,
    )
    ledger = replace(
        capture.ledger,
        entries=capture.ledger.entries + ((4, 10_070, False, "stale_boot"),),
        observed=4,
    )
    return case_of(
        replace(capture, commands=commands, ledger=ledger),
        admission=admission(end=4),
    )


def _rejection_inside() -> CaseEvidence:
    """Ruling 3 is REJECTED, and the third association names ruling 4: a
    rejection is owed no row, since the router dispatches only admissions."""
    def edit(rows: list[tuple]) -> None:
        rows[ASSOCIATIONS[2]] = _with(rows[ASSOCIATIONS[2]], ASSOCIATION_RULING, 4)
        rows[CLOSED] = _with(rows[CLOSED], SUBSCRIPTION_RULING, 4)

    capture = _rows(edit)
    first, second, _ = capture.ledger.entries
    ledger = replace(
        capture.ledger,
        entries=(
            first, second, (3, 10_050, False, "stale_boot"),
            (4, 10_060, True, None),
        ),
        observed=4,
        admitted=4,
    )
    return case_of(replace(capture, ledger=ledger), admission=admission(end=4))


def _only_association_lost() -> TraceCapture:
    """s0 is ruling 1, so the window holds ruling 2 alone, and its row is lost."""
    def edit(rows: list[tuple]) -> None:
        rows[OPENED] = _with(rows[OPENED], SUBSCRIPTION_RULING, 1)
        del rows[ASSOCIATIONS[1]]

    return _rows(edit)


def _fenced(micros: int) -> TraceCapture:
    """The second association FENCED and carrying ``micros``: a fenced row
    is held to its ruling's stamp as a committed one is."""
    def edit(rows: list[tuple]) -> None:
        row = _with(rows[ASSOCIATIONS[1]], 2, ASSOCIATION_FENCED)
        rows[ASSOCIATIONS[1]] = _with(row, 3, micros)

    return _rows(edit)


def _closed_first_at_one_ruling() -> TraceCapture:
    """Both window rows at ruling 3, the CLOSED row first: rulings in order
    are not enough, the rows themselves must be OPENED then CLOSED."""
    def edit(rows: list[tuple]) -> None:
        rows[OPENED] = _with(rows[OPENED], SUBSCRIPTION_RULING, 3)
        rows[OPENED], rows[CLOSED] = rows[CLOSED], rows[OPENED]

    return _rows(edit)


def _identity(**changes: Any) -> dict[str, Any]:
    identity = {**IDENTITY, **changes}
    return identity


def _case(**changes: Any) -> dict[str, Any]:
    return _identity(case={**IDENTITY["case"], **changes})


def _endpoint(teardown: Any) -> dict[str, Any]:
    return _identity(endpoints={**IDENTITY["endpoints"], "teardown": teardown})


def _failed(step: str) -> list[dict[str, str]]:
    return [{"step": step, "error": "RuntimeError('failed')"}]


GAP = (lambda e: (e[0], e[2]))(eligible_capture().ledger.entries)
NOT_LIVE = admission(
    live=False, first=None, end=None, faults=None, faulted=None,
    error="the vehicle has no admission tap", window_rows=None,
)
VARIANTS: dict[str, tuple[Callable[[], CaseEvidence], set[int]]] = {
    "period 0": (lambda: case_of(sealed_capture(eligible_trace(0))), {2}),
    # Review round 4: real overflows, since the reader refuses a drop no
    # store could make. The ledger's also leaves rule 6 unmet, and the
    # command log's, a pass sample dropped, rules 7 and 8.
    "the journal overflowed": (
        lambda: _overflowed(
            lambda trace: trace.record_truth(epoch=1, outcome=TRUTH_RECORDED)
        ),
        {4},
    ),
    "the journal failed": (lambda: case_of(_status(failed=True)), {4}),
    "the journal drained": (lambda: case_of(_status(drained=True)), {4}),
    # Review round 5: a real unreadable frame, since the reader refuses the
    # latch with no such row held.
    "a payload unreadable": (lambda: case_of(_unreadable_payload()), {4}),
    "a callback fault": (lambda: case_of(_status(callback_faults=1)), {4}),
    "the command log overflowed": (
        lambda: _overflowed(_passes_to_ten), {4, 7, 8}
    ),
    "the command log failed": (lambda: case_of(_commands(failed=True)), {4}),
    "a command unreadable": (
        lambda: case_of(_commands(entries=((2, COMMAND_UNREADABLE),))), {4}
    ),
    "the ledger overflowed": (lambda: _overflowed(_rulings_to_ten), {4, 6}),
    "the ledger failed": (lambda: case_of(_ledger(failed=True)), {4}),
    "a gap in the ledger": (
        lambda: case_of(_ledger(entries=GAP)), {4, 6, 7, 9}
    ),
    "the tap not live": (
        lambda: case_of(eligible_capture(), admission=NOT_LIVE), {5, 6, 7, 9}
    ),
    "the tap not live, its window rows read": (
        lambda: case_of(eligible_capture(), admission=admission(live=False)),
        {5},
    ),
    "no admission evidence": (
        lambda: case_of(eligible_capture(), admission=None), {5, 6, 7, 9}
    ),
    "admission evidence not an object": (
        lambda: case_of(eligible_capture(), admission="live"), {5, 6, 7, 9}
    ),
    "the ledger subscribed once the source had started": (
        lambda: case_of(eligible_capture(), admission=admission(window_rows=1)),
        {5},
    ),
    "the window rows unread": (
        lambda: case_of(
            eligible_capture(), admission=admission(window_rows=None)
        ),
        {5},
    ),
    "a counted delivery fault": (
        lambda: case_of(eligible_capture(), admission=admission(faults=1)), {6}
    ),
    "an uncounted loss": (
        lambda: case_of(eligible_capture(), admission=admission(faulted=True)),
        {6},
    ),
    "the fault count unknown": (
        lambda: case_of(eligible_capture(), admission=admission(faults=None)),
        {6},
    ),
    "the fault flag unknown": (
        lambda: case_of(eligible_capture(), admission=admission(faulted=None)),
        {6},
    ),
    "the lost last ruling": (
        lambda: case_of(eligible_capture(), admission=admission(end=4)), {6}
    ),
    "never cancelled": (
        lambda: case_of(eligible_capture(), admission=admission(end=None)),
        {6, 7, 9},
    ),
    # Review round 1: a count no list could hold is judged, not made.
    "an end past any count": (
        lambda: case_of(eligible_capture(), admission=admission(end=10**30)),
        {6},
    ),
    "a duplicate-and-missing pass": (
        lambda: case_of(_commands(passes=((2, None, None), (2, 2, 2)))), {7}
    ),
    "a pass missing": (lambda: case_of(_commands(iterations=3)), {7}),
    "iterations past any count": (
        lambda: case_of(_commands(iterations=10**30)), {7}
    ),
    # Review round 2: the widest figure the reader takes is judged, never
    # raised. A reason prints only figures the case holds: first - 1 of the
    # widest first is too wide for Python to print.
    "the widest first": (
        lambda: case_of(eligible_capture(), admission=admission(first=-WIDEST)),
        {6},
    ),
    "the widest first and end, below zero": (
        lambda: case_of(
            eligible_capture(), admission=admission(first=-WIDEST, end=-WIDEST)
        ),
        {6, 7, 9},
    ),
    "the widest end": (
        lambda: case_of(eligible_capture(), admission=admission(end=WIDEST)),
        {6},
    ),
    "the widest iteration count": (
        lambda: case_of(_commands(iterations=WIDEST)), {7}
    ),
    "a future cutoff": (
        lambda: case_of(_commands(passes=((1, None, None), (2, 4, 2)))), {7}
    ),
    "a future cutoff, its admitted cutoff the last accepted": (
        lambda: case_of(_commands(passes=((1, None, None), (2, 4, 3)))), {7}
    ),
    "a stale cutoff": (
        lambda: case_of(_commands(passes=((1, None, None), (2, 2, 1)))), {7}
    ),
    "a decreasing cutoff": (
        lambda: case_of(_commands(passes=((1, 2, 2), (2, 1, 1)))), {7}
    ),
    "a rejected cutoff": (_rejected_cutoff, {7}),
    "an output with no pass": (
        lambda: case_of(_field(OUTPUT, OUTPUT_WORKER_ITERATION, 5)), {8}
    ),
    "a command with no pass": (
        lambda: case_of(_commands(entries=((5, None),))), {8}
    ),
    "commands out of order": (
        lambda: case_of(_commands(entries=((2, None), (1, None)))), {8}
    ),
    "a command repeated": (
        lambda: case_of(_commands(entries=((2, None), (2, None)))), {8}
    ),
    "a second OPENED row": (
        lambda: case_of(_rows(lambda rows: rows.insert(1, rows[OPENED]))), {9}
    ),
    "no CLOSED row": (lambda: case_of(_without(CLOSED)), {9}),
    "CLOSED before OPENED": (lambda: case_of(_swapped(OPENED, CLOSED)), {9}),
    "CLOSED before OPENED, both at one ruling": (
        lambda: case_of(_closed_first_at_one_ruling()), {9}
    ),
    "s0 after s1": (
        lambda: case_of(_rows(lambda rows: rows.__setitem__(
            OPENED, _with(rows[OPENED], SUBSCRIPTION_RULING, 3)
        ) or rows.__setitem__(
            CLOSED, _with(rows[CLOSED], SUBSCRIPTION_RULING, 2)
        ))),
        {9},
    ),
    "s1 past the end": (
        lambda: case_of(_field(CLOSED, SUBSCRIPTION_RULING, 4)), {9}
    ),
    "an association naming no admission": (
        lambda: case_of(_field(ASSOCIATIONS[2], ASSOCIATION_RULING, 7)), {9}
    ),
    "associations out of order": (
        lambda: case_of(_swapped(ASSOCIATIONS[0], ASSOCIATIONS[1])), {9}
    ),
    "a lost first association": (
        lambda: case_of(_without(ASSOCIATIONS[0])), {9}
    ),
    "a lost last association": (
        lambda: case_of(_without(ASSOCIATIONS[1])), {9}
    ),
    "a lost only association": (lambda: case_of(_only_association_lost()), {9}),
    "an association twice": (
        lambda: case_of(_rows(lambda rows: rows.insert(
            ASSOCIATIONS[1], rows[ASSOCIATIONS[1]]
        ))),
        {9},
    ),
    "a stamp mismatch": (
        lambda: case_of(_field(ASSOCIATIONS[1], 3, 10_041_000)), {9}
    ),
    "a stamp under a millisecond off": (
        lambda: case_of(_field(ASSOCIATIONS[1], 3, 10_040_500)), {9}
    ),
    "a fenced row's stamp mismatch": (lambda: case_of(_fenced(10_041_000)), {9}),
    "a discontinuity": (lambda: _after_close((4, 10_000, True, None)), {10}),
    "a stampless admission": (lambda: _after_close((4, None, True, None)), {10}),
    "a refusal in the journal": (lambda: case_of(_status(refused=1)), {11}),
    "a refusal in the command log": (
        lambda: case_of(_commands(refused=1)), {11}
    ),
    "a refusal in the ledger": (lambda: case_of(_ledger(refused=1)), {11}),
    "the stop not confirmed": (
        lambda: case_of(eligible_capture(), worker="not_confirmed"), {11}
    ),
    "the worker never started": (
        lambda: case_of(eligible_capture(), worker="never_started"), {11}
    ),
    "no ending epoch": (lambda: case_of(eligible_capture(), epoch=None), {12}),
    "no CLOSED lifecycle at the ending epoch": (
        lambda: case_of(eligible_capture(), epoch=2), {12}
    ),
    "the CLOSED lifecycle lost": (lambda: case_of(_without(ENDED)), {12}),
    "a failed close, stores complete and the stop confirmed": (
        lambda: case_of(
            eligible_capture(), teardown_errors=_failed(SOURCE_CLOSE_STEP)
        ),
        {12},
    ),
    "the evidence step failed": (
        lambda: case_of(
            eligible_capture(), teardown_errors=_failed(EVIDENCE_STEP)
        ),
        {12},
    ),
    "teardown errors unreadable": (
        lambda: case_of(eligible_capture(), teardown_errors="failed"), {12}
    ),
    "a failure held for raising": (
        lambda: case_of(eligible_capture(), held=1), {12}
    ),
    "the summary not written": (
        lambda: case_of(eligible_capture(), summary_error="OSError('full')"),
        {12},
    ),
    "a missing case": (lambda: case_of(eligible_capture(), identity=None), {13}),
    "identity not an object": (
        lambda: edited_case(eligible_capture(), identity=["case"]), {13}
    ),
    "no harness identity": (
        lambda: case_of(eligible_capture(), identity=_identity(harness=None)),
        {13},
    ),
    "no case name": (
        lambda: case_of(eligible_capture(), identity=_case(name=None)), {13}
    ),
    "no definition hash": (
        lambda: case_of(
            eligible_capture(), identity=_case(definition_sha256=None)
        ),
        {13},
    ),
    "a definition hash that is not one": (
        lambda: case_of(
            eligible_capture(), identity=_case(definition_sha256="d" * 63)
        ),
        {13},
    ),
    "endpoint hashes that differ": (
        lambda: case_of(
            eligible_capture(),
            identity=_endpoint({"sha256": "f" * 64, "files": 3}),
        ),
        {13},
    ),
    "an endpoint that could not be hashed": (
        lambda: case_of(
            eligible_capture(), identity=_endpoint({"error": "OSError()"})
        ),
        {13},
    ),
}


def test_the_baseline_meets_every_rule() -> None:
    verdict = judge(case_of(eligible_capture()))
    assert (verdict.status, verdict.reasons, verdict.eligible) == (
        ELIGIBLE, (), True
    )


@pytest.mark.parametrize("name", sorted(VARIANTS))
def test_each_variant_fails_exactly_the_rules_it_breaks(name: str) -> None:
    make, rules = VARIANTS[name]
    verdict = judge(make())
    assert verdict.status == INSPECTABLE
    assert not verdict.eligible
    assert set(verdict.failed_rules) == rules, verdict.reasons


def _all_layouts() -> TraceCapture:
    """The baseline with a STAGE and a DECIMATE row added, so every layout
    that carries a stamp is present. Still eligible."""
    extra = (
        stage_row(1, STAGE_STAGED, 10_040_000, 10_040_000, FRAME, PERIOD_US),
        discard_row(
            1, DISCARD_PUBLISH_OVERWRITTEN, 10_040_000, 10_060_000, PERIOD_US
        ),
    )
    return _rows(lambda rows: rows.__setitem__(slice(ENDED, ENDED), extra))


SLOTS = [
    (index, stamp, slot)
    for index, row in enumerate(_all_layouts().rows)
    for stamp, slot in SLOT_PAIRS[row[0]]
]


@pytest.mark.parametrize(("index", "stamp", "slot"), SLOTS)
def test_a_slot_that_disagrees_with_its_stamp_fails_rule_3(
    index: int, stamp: int, slot: int
) -> None:
    """Off by one where there is a stamp, present where there is none."""
    capture = _all_layouts()
    assert judge(case_of(capture)).eligible
    row = capture.rows[index]
    wrong = 0 if row[stamp] is None else row[slot] + 1
    rows = list(capture.rows)
    rows[index] = _with(row, slot, wrong)
    verdict = judge(case_of(with_rows(capture, rows)))
    assert set(verdict.failed_rules) == {3}, verdict.reasons


def test_a_stamp_without_its_slot_fails_rule_3() -> None:
    capture = _field(ASSOCIATIONS[0], 4, None)
    assert set(judge(case_of(capture)).failed_rules) == {3}


STILL_ELIGIBLE: dict[str, Callable[[], CaseEvidence]] = {
    "every layout": lambda: case_of(_all_layouts()),
    "a fenced association": lambda: case_of(_fenced(10_040_000)),
    "a rejection inside the window": _rejection_inside,
    "a later step's failure": lambda: case_of(
        eligible_capture(), teardown_errors=_failed("logger.close")
    ),
    "a repeated stamp's violation row": lambda: case_of(_violation_held((
        EVENT_VIOLATION, 1, VIOLATION_SOURCE_REPEATED,
        (10_040_000, 10_040_000),
    ))),
}


def _violation_held(row: tuple) -> TraceCapture:
    """``row`` held among the associations, and latched as the journal's
    ``note_violation`` latches every violation it holds: the reader refuses
    a held violation with no latch (review round 1)."""
    capture = _rows(lambda rows: rows.insert(ASSOCIATIONS[2], row))
    return replace(capture, status=replace(
        capture.status, first_violation=row, violations_by_epoch={row[1]: row}
    ))


def test_rule_6_numbers_each_ruling_as_well_as_counting_them() -> None:
    """Three rulings held where rulings 1..3 were owed, but not in order:
    their count alone would pass them (review round 1)."""
    first, second, third = eligible_capture().ledger.entries
    # Held in that order, the latest ruling observed and admitted is the last
    # one held, as the ledger's append would leave them.
    case = case_of(_ledger(
        entries=(first, third, second), observed=second[0], admitted=second[0]
    ))
    assert owed_rulings(case) == ["the ledger does not hold exactly rulings 1..3"]


def test_an_end_below_first_owes_no_ruling_as_the_empty_range_did() -> None:
    """first..end holds no ruling when end is below first - 1, as when it
    is first - 1, so an empty ledger holds exactly what was owed: the count
    owed is never below zero. Rule 9 refuses such a window (review
    round 1)."""
    empty = _ledger(entries=(), observed=None, admitted=None)
    case = case_of(empty, admission=admission(first=5, end=2))
    assert owed_rulings(case) == []


@pytest.mark.parametrize("name", sorted(STILL_ELIGIBLE))
def test_what_no_rule_names_leaves_a_case_eligible(name: str) -> None:
    """A VIOLATION row, a rejection the router never dispatched, and a
    teardown step that is neither the close nor the evidence's."""
    verdict = judge(STILL_ELIGIBLE[name]())
    assert verdict.eligible, verdict.reasons


def test_every_failed_rule_is_a_reason_in_rule_order() -> None:
    verdict = judge(case_of(
        eligible_capture(), worker="not_confirmed", held=1, identity=None
    ))
    assert verdict.failed_rules == (11, 12, 13)
    assert [reason.rule for reason in verdict.reasons] == sorted(
        reason.rule for reason in verdict.reasons
    )


def test_the_reasons_of_many_rules_come_in_rule_order() -> None:
    """Eight rules at once, the ledger's among them, each reason where its
    rule's number puts it."""
    verdict = judge(case_of(
        _status(failed=True),
        admission=NOT_LIVE,
        worker="not_confirmed",
        held=1,
        identity=None,
    ))
    rules = [reason.rule for reason in verdict.reasons]
    assert verdict.failed_rules == (4, 5, 6, 7, 9, 11, 12, 13)
    assert rules == sorted(rules)


# What each store variant's rule-4 reason says: the figure that broke it,
# never only that the capture is not complete.
STORE_REASONS = {
    "the journal overflowed": "the journal overflowed",
    "the journal failed": "the journal failed",
    "the journal drained": "the journal was drained",
    "a payload unreadable": "a payload was unreadable",
    "a callback fault": "callback faults: 1",
    "the command log overflowed": "commands dropped: 1",
    "the command log failed": "the command log failed",
    "a command unreadable": "commands unreadable: 1",
    "the ledger overflowed": "ledger entries dropped: 1",
    "the ledger failed": "the ledger failed",
    "a gap in the ledger": "the ledger has a gap",
}


@pytest.mark.parametrize("name", sorted(STORE_REASONS))
def test_rule_4_names_the_figure_that_broke_it(name: str) -> None:
    make, _ = VARIANTS[name]
    texts = [reason.text for reason in judge(make()).reasons if reason.rule == 4]
    assert texts == [STORE_REASONS[name]]


def test_a_case_that_cannot_be_read_fails_rule_1_alone() -> None:
    """The reader's refusal is the verdict: no other rule can be judged on
    evidence that does not vouch for itself."""
    verdict = refusal(EvidenceRefused("the manifest is missing"))
    assert (verdict.status, verdict.failed_rules) == (INSPECTABLE, (1,))
    assert "the manifest is missing" in verdict.reasons[0].text
