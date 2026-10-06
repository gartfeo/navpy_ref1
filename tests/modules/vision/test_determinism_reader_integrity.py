"""Stored evidence must not contradict records that its own writer retains."""

import dis
import sys
from dataclasses import replace
from types import CodeType

import pytest

from navpy.modules.vision.sim.determinism_evidence import sealed_capture
from navpy.modules.vision.sim.determinism_command_log import CommandLoopLog
from navpy.modules.vision.sim.determinism_events import (
    ASSOCIATION_COMMITTED,
    EVENT_ASSOCIATION,
    EVENT_VIOLATION,
    TRUTH_RECORDED,
)
from navpy.modules.vision.sim.determinism_reader import read_evidence
from navpy.modules.vision.sim.determinism_trace_decode import EvidenceRefused
from tests.modules.vision.determinism_case_factory import (
    eligible_trace,
    evidence_bytes,
)


def _violated_trace(capacity=13):
    trace = eligible_trace(capacity=capacity)
    for stamp in (10.0, 9.0):
        trace.record_association(
            epoch=1, outcome=ASSOCIATION_COMMITTED, source_s=stamp
        )
    return trace


@pytest.mark.parametrize("field", ["first_violation", "violations_by_epoch", "both"])
def test_overflow_cannot_replace_a_retained_first_violation(field):
    trace = _violated_trace()
    trace.record_truth(epoch=1, outcome=TRUTH_RECORDED)
    capture = sealed_capture(trace)
    violations = [row for row in capture.rows if row[0] == EVENT_VIOLATION]
    assert len(violations) == 2 and capture.status.dropped == 1
    read_evidence(*evidence_bytes(capture))
    values = {"first_violation": violations[1], "violations_by_epoch": {1: violations[1]}}
    changes = values if field == "both" else {field: values[field]}
    changed = replace(capture, status=replace(capture.status, **changes))
    with pytest.raises(EvidenceRefused):
        read_evidence(*evidence_bytes(changed))


@pytest.mark.parametrize("field", ["first_violation", "violations_by_epoch"])
def test_boolean_epoch_is_not_an_integer_in_stored_latches(field):
    capture = sealed_capture(_violated_trace())
    first = capture.status.first_violation
    malformed = (first[0], True, *first[2:])
    value = malformed if field == "first_violation" else {True: malformed}
    changed = replace(capture, status=replace(capture.status, **{field: value}))
    with pytest.raises(EvidenceRefused):
        read_evidence(*evidence_bytes(changed))


def test_iteration_count_includes_samples_provably_dropped():
    trace = eligible_trace(capacity=9)
    for iteration in range(3, 11):
        trace.command_log.note_pass(iteration, None, None)
    capture = sealed_capture(trace)
    assert (capture.commands.iterations, len(capture.commands.passes)) == (10, 9)
    assert capture.commands.dropped == 1 and len(capture.commands.entries) < 9
    read_evidence(*evidence_bytes(capture))
    changed = replace(capture, commands=replace(capture.commands, iterations=9))
    with pytest.raises(EvidenceRefused):
        read_evidence(*evidence_bytes(changed))


@pytest.mark.parametrize("passes", [2, 9])
def test_command_drops_are_not_invented_as_pass_samples(passes):
    trace = eligible_trace(capacity=9)
    for iteration in range(3, passes + 1):
        trace.command_log.note_pass(iteration, None, None)
    for _ in range(9):
        trace.command_log.note_command(2, None)
    capture = sealed_capture(trace)
    assert capture.commands.dropped == 1
    assert len(capture.commands.entries) == 9
    assert capture.commands.iterations == passes
    assert read_evidence(*evidence_bytes(capture)).capture == capture


@pytest.mark.parametrize("capacity", [0, 9, 10, 12, 13])
def test_real_overflow_with_retained_or_dropped_latches_reads_back(capacity):
    trace = _violated_trace(capacity=capacity)
    trace.record_truth(epoch=1, outcome=TRUTH_RECORDED)
    capture = sealed_capture(trace)
    assert capture.status.dropped > 0
    if capacity == 0:
        assert (capture.commands.iterations, capture.commands.dropped) == (2, 3)
        assert not capture.commands.entries and not capture.commands.passes
    assert read_evidence(*evidence_bytes(capture)).capture == capture


def test_drained_first_violation_may_differ_from_first_remaining():
    trace = _violated_trace()
    trace.journal.drain()
    trace.record_association(
        epoch=1, outcome=ASSOCIATION_COMMITTED, source_s=8.0
    )
    capture = sealed_capture(trace)
    held = [row for row in capture.rows if row[0] == EVENT_VIOLATION]
    assert held and capture.status.first_violation != held[0]
    assert read_evidence(*evidence_bytes(capture)).capture == capture


@pytest.mark.parametrize("component", ["key", "row"])
@pytest.mark.parametrize("drained", [False, True])
def test_boolean_epoch_is_rejected_independently_in_key_and_row(component, drained):
    trace = _violated_trace()
    if drained:
        trace.journal.drain()
    capture = sealed_capture(trace)
    first = capture.status.first_violation
    key = True if component == "key" else 1
    row = (first[0], True, *first[2:]) if component == "row" else first
    status = replace(capture.status, violations_by_epoch={key: row})
    with pytest.raises(EvidenceRefused):
        read_evidence(*evidence_bytes(replace(capture, status=status)))


def test_interrupted_pass_drop_keeps_its_capture_readable():
    change = next(
        item for item in CommandLoopLog.note_pass.__code__.co_consts
        if isinstance(item, CodeType) and item.co_name == "change"
    )
    reached = set()
    for line in sorted({line for _, line in dis.findlinestarts(change)}):
        trace = eligible_trace(capacity=9)
        for iteration in range(3, 10):
            trace.command_log.note_pass(iteration, None, None)

        def interrupt(frame, event, arg):
            if frame.f_code is change and event == "line" and frame.f_lineno == line:
                reached.add(line)
                raise KeyboardInterrupt
            return interrupt

        previous = sys.gettrace()
        sys.settrace(interrupt)
        try:
            trace.command_log.note_pass(10, None, None)
        except KeyboardInterrupt:
            pass
        finally:
            sys.settrace(previous)
        capture = sealed_capture(trace)
        assert read_evidence(*evidence_bytes(capture)).capture == capture, line
    assert len(reached) >= 5


@pytest.mark.parametrize("capacity", [0, 11, 30])
def test_multiple_epochs_with_held_or_dropped_violations_read_back(capacity):
    trace = eligible_trace(capacity=capacity)
    for epoch in (1, 2, 3):
        trace.record_association(
            epoch=epoch, outcome=ASSOCIATION_COMMITTED, source_s=9.0 - epoch
        )
    capture = sealed_capture(trace)
    assert list(capture.status.violations_by_epoch) == [1, 2, 3]
    assert read_evidence(*evidence_bytes(capture)).capture == capture


@pytest.mark.parametrize("drained", [False, True])
@pytest.mark.parametrize("replacement", [None, "later"])
def test_global_latch_agrees_with_epoch_latch_after_loss(drained, replacement):
    trace = _violated_trace(capacity=9)
    if drained:
        trace.journal.drain()
    capture = sealed_capture(trace)
    first = capture.status.first_violation
    value = (first[0], first[1], first[2], (8_000_000, 10_060_000))
    status = replace(capture.status, first_violation=None if replacement is None else value)
    with pytest.raises(EvidenceRefused):
        read_evidence(*evidence_bytes(replace(capture, status=status)))


def test_a_latch_cannot_be_another_valid_event_type():
    capture = sealed_capture(_violated_trace(capacity=9))
    association = next(row for row in capture.rows if row[0] == EVENT_ASSOCIATION)
    status = replace(
        capture.status, first_violation=association,
        violations_by_epoch={1: association},
    )
    with pytest.raises(EvidenceRefused):
        read_evidence(*evidence_bytes(replace(capture, status=status)))


@pytest.mark.parametrize("capacity", [11, 30])
def test_epoch_latch_order_preserves_retained_prefix(capacity):
    trace = eligible_trace(capacity=capacity)
    for epoch in (1, 2, 3):
        trace.record_association(
            epoch=epoch, outcome=ASSOCIATION_COMMITTED, source_s=9.0 - epoch
        )
    capture = sealed_capture(trace)
    latches = capture.status.violations_by_epoch
    # Keep global-to-first agreement, while moving an epoch known only from a
    # dropped row ahead of a retained epoch (or reordering fully held rows).
    order = (2, 1, 3) if capacity == 11 else (1, 3, 2)
    status = replace(capture.status, violations_by_epoch={key: latches[key] for key in order})
    if capacity == 11:
        status = replace(status, first_violation=latches[2])
    with pytest.raises(EvidenceRefused):
        read_evidence(*evidence_bytes(replace(capture, status=status)))


def test_boolean_latch_is_rejected_when_all_violations_were_dropped():
    capture = sealed_capture(_violated_trace(capacity=9))
    first = capture.status.first_violation
    malformed = (first[0], True, *first[2:])
    status = replace(capture.status, first_violation=malformed,
                     violations_by_epoch={True: malformed})
    with pytest.raises(EvidenceRefused):
        read_evidence(*evidence_bytes(replace(capture, status=status)))


def test_malformed_latch_error_identifies_manifest_metadata():
    capture = sealed_capture(_violated_trace())
    first = capture.status.first_violation
    status = replace(capture.status, first_violation=(first[0], True, *first[2:]))
    with pytest.raises(EvidenceRefused, match="^a violation latch is malformed:"):
        read_evidence(*evidence_bytes(replace(capture, status=status)))
