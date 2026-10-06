"""A post-leg reader must see each ledger as it stood at ONE instant.

Review round 2 produced ``rows=1, rows_recorded=2, complete=True`` from a
summary: the rows and the status beside them were read under two acquisitions
of the row lock, and a row appended between the two was counted by one and
missing from the other -- a combination no instant ever held. The command
ledger had the same shape across its entries and its counters.

The write here lands in exactly that gap, on a real lock: the lock is released
first and the write runs after, so a reader that needs a second acquisition
sees it and a reader that took one does not.
"""

from __future__ import annotations

import dataclasses
import threading
from collections.abc import Callable
from types import SimpleNamespace

import pytest

from navpy.modules.vision.sim.determinism_command_log import CommandLoopLog
from navpy.modules.vision.sim.determinism_events import TRUTH_RECORDED
from navpy.modules.vision.sim.determinism_trace import DeterminismTrace
from navpy.modules.vision.sim.determinism_trace_summary import summarise

PERIOD_US = 20_000  # 50 Hz autopilot scheduler period.


class _WriteInTheGap:
    """A real lock that, once armed, runs one write right after a release.

    Disarmed BEFORE the write runs, because the write takes this same lock and
    would otherwise fire itself again on its own way out.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._write: Callable[[], None] | None = None
        self.fired = False

    def arm(self, write: Callable[[], None]) -> None:
        self._write = write

    def __enter__(self) -> bool:
        return self._lock.__enter__()

    def __exit__(self, *exc: object) -> None:
        self._lock.__exit__(*exc)
        write, self._write = self._write, None
        if write is not None:
            self.fired = True
            write()


def _gapped_trace() -> tuple[DeterminismTrace, _WriteInTheGap]:
    """A trace holding one scored-leg row, its row lock replaced by the gap."""
    trace = DeterminismTrace(PERIOD_US)
    gap = _WriteInTheGap()
    trace.journal._lock = gap
    trace.record_truth(epoch=1, outcome=TRUTH_RECORDED)
    gap.arm(lambda: trace.record_truth(epoch=1, outcome=TRUTH_RECORDED))
    return trace, gap


def test_the_row_ledger_is_captured_in_one_acquisition() -> None:
    trace, gap = _gapped_trace()

    rows, status = trace.journal.capture()

    assert gap.fired, "the write never landed, so this test says nothing"
    assert len(rows) == status.rows == 1
    rows, status = trace.journal.capture()
    assert len(rows) == status.rows == 2


def test_the_command_ledger_is_captured_in_one_acquisition() -> None:
    log = CommandLoopLog(8)
    gap = _WriteInTheGap()
    log._lock = gap
    log.note_iteration(1)
    log.note_command(1, None)

    def next_pass() -> None:
        log.note_iteration(2)
        log.note_command(2, None)

    gap.arm(next_pass)
    capture = log.capture()

    assert gap.fired, "the write never landed, so this test says nothing"
    assert capture.iterations == len(capture.entries) == 1
    later = log.capture()
    assert later.iterations == len(later.entries) == 2


def test_a_summary_never_pairs_rows_with_a_status_from_another_instant() -> None:
    """Review round 2's probe, kept as a regression.

    Read separately, this summary said one row, two recorded, and complete.
    """
    trace, gap = _gapped_trace()

    summary = summarise(trace, epoch=1)

    assert gap.fired, "the write never landed, so this test says nothing"
    assert summary["rows"] == summary["rows_recorded"] == 1
    assert summary["counts"]["truth"] == {TRUTH_RECORDED: 1}


def test_a_summary_takes_its_command_figures_from_its_verdicts_read() -> None:
    """The command half of the same defect.

    A hole written between two reads of the command ledger would show in the
    counts while the verdict beside them still called the ledger clean.
    """
    trace = DeterminismTrace(PERIOD_US)
    log = trace.command_log
    gap = _WriteInTheGap()
    log._lock = gap
    log.note_iteration(1)
    log.note_command(1, None)

    def holed_pass() -> None:
        log.note_iteration(2)
        log.note_command(2, SimpleNamespace(yaw="not a number"))

    gap.arm(holed_pass)
    summary = summarise(trace, epoch=1)

    assert gap.fired, "the write never landed, so this test says nothing"
    assert summary["complete"] is True
    commands = summary["commands"]
    assert commands["unreadable"] == 0
    assert commands["iterations"] == commands["entries"] == 1


def test_a_capture_is_an_immutable_record_of_both_ledgers() -> None:
    trace = DeterminismTrace(PERIOD_US)
    trace.record_truth(epoch=1, outcome=TRUTH_RECORDED)
    trace.command_log.note_iteration(1)
    # A command object the recorder cannot digest: a HOLE in the ledger.
    trace.command_log.note_command(1, SimpleNamespace(yaw="not a number"))

    capture = trace.capture()

    assert isinstance(capture.rows, tuple)
    assert isinstance(capture.commands.entries, tuple)
    assert capture.period_us == PERIOD_US
    assert capture.commands.unreadable == 1
    # The command ledger's hole is folded into the one verdict.
    assert capture.status.commands_incomplete is True
    assert capture.status.complete is False
    with pytest.raises(dataclasses.FrozenInstanceError):
        capture.rows = ()
    with pytest.raises(dataclasses.FrozenInstanceError):
        capture.commands.iterations = 0

    trace.record_truth(epoch=1, outcome=TRUTH_RECORDED)
    trace.command_log.note_iteration(2)
    trace.command_log.note_command(2, None)

    # Later recording does not reach back into what was captured.
    assert len(capture.rows) == capture.status.rows == 1
    assert capture.commands.iterations == len(capture.commands.entries) == 1
