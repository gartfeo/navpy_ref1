"""The worker thread's ledger: iterations, command bytes, and its own limits."""

from __future__ import annotations

from types import SimpleNamespace

from navpy.modules.navigation.calc_data import CalcData
from navpy.modules.vision.sim.determinism_command_log import (
    COMMAND_RAISED,
    COMMAND_UNREADABLE,
    NULL_COMMAND_LOOP,
    CommandLoopLog,
)
from navpy.modules.vision.sim.determinism_slots import command_digest


def test_an_unstarted_log_has_no_current_iteration() -> None:
    log = CommandLoopLog(8)

    assert log.current_iteration() is None
    assert log.entries() == []
    assert log.iterations == 0
    assert log.failed is False


def test_the_current_iteration_is_the_latest_one_noted() -> None:
    """A dispatch stamps its row from here, so it must be the pass in progress."""
    log = CommandLoopLog(8)

    log.note_iteration(1)
    assert log.current_iteration() == 1
    log.note_iteration(2)
    assert log.current_iteration() == 2
    assert log.iterations == 2


def test_a_command_is_recorded_against_the_pass_that_issued_it() -> None:
    log = CommandLoopLog(8)
    command = CalcData(1.0, 2.0, 3.0, None, 0.5)

    log.note_iteration(4)
    log.note_command(4, command)

    assert log.entries() == [(4, command_digest(command))]


def test_a_command_that_raised_is_not_the_same_as_no_command() -> None:
    """Two runs that differ only in which one crashed are different runs."""
    log = CommandLoopLog(8)

    log.note_command(1, None)
    log.note_command(2, None, True)

    assert log.entries() == [(1, None), (2, COMMAND_RAISED)]


def test_two_commands_differing_only_in_an_absent_channel_differ_in_bytes() -> (
    None
):
    """``None`` and 0.0 on an optional channel are different commands."""
    absent = CalcData(1.0, 2.0, None, 4.0, 5.0)
    zero = CalcData(1.0, 2.0, 0.0, 4.0, 5.0)

    assert command_digest(absent) != command_digest(zero)


def test_bit_identical_negative_zero_is_not_the_same_command_as_zero() -> None:
    """Float equality is the wrong relation for a digest, in both directions."""
    assert command_digest(
        CalcData(-0.0, 2.0, 3.0, 4.0, 5.0)
    ) != command_digest(CalcData(0.0, 2.0, 3.0, 4.0, 5.0))


def test_an_unreadable_command_is_not_the_same_as_no_command() -> None:
    """Three outcomes, three answers.

    "The law produced nothing" and "the recorder could not read what the law
    produced" are different facts about a run, so they cannot share the None
    digest. And neither may raise into the worker: this is a command path.
    """
    log = CommandLoopLog(8)

    log.note_command(1, None)
    log.note_command(2, SimpleNamespace(yaw="not a number"))
    log.note_command(3, None, True)

    assert log.entries() == [
        (1, None),
        (2, COMMAND_UNREADABLE),
        (3, COMMAND_RAISED),
    ]
    assert log.failed is False
    # Only the middle one is a HOLE: the other two are comparable
    # outcomes, and counting them would invalidate honest runs.
    assert log.unreadable == 1


def test_an_out_of_range_command_value_is_unreadable_not_a_fault() -> None:
    """A single bad value must not invalidate the whole ledger.

    float(10**400) raises OverflowError, which is an ArithmeticError and
    so was missed by a hand-named except tuple. It escaped the digest and
    latched ``failed``, throwing away every other command in the run to
    report one that could not be read. Undigestable is an ANSWER.
    """
    log = CommandLoopLog(8)

    log.note_command(1, CalcData(1.0, 2.0, 3.0, None, 0.5))
    log.note_command(2, SimpleNamespace(yaw=10**400, pitch=0.0))
    log.note_command(3, CalcData(4.0, 5.0, None, 6.0, 0.25))

    digests = [digest for _, digest in log.entries()]
    assert digests[1] == COMMAND_UNREADABLE
    assert digests[0] == command_digest(CalcData(1.0, 2.0, 3.0, None, 0.5))
    assert digests[2] == command_digest(CalcData(4.0, 5.0, None, 6.0, 0.25))
    assert log.failed is False
    assert log.unreadable == 1


def test_a_command_whose_getter_raises_is_unreadable_not_a_fault() -> None:
    """The digest reads an object the recorder does not own."""

    class Unreadable:
        pitch = 0.0

        @property
        def yaw(self) -> float:
            raise RuntimeError("no")

    log = CommandLoopLog(8)
    log.note_command(1, Unreadable())

    assert log.entries() == [(1, COMMAND_UNREADABLE)]
    assert log.failed is False


def test_the_ledger_decides_what_counts_as_a_hole_in_its_own_evidence() -> None:
    """One atomic read, and only the three real holes set it.

    A reader that combined three separately-locked properties could observe a
    torn state, and the trace enumerating the ledger internals put that
    knowledge in the wrong object.
    """
    clean = CommandLoopLog(8)
    clean.note_command(1, CalcData(1.0, 2.0, 3.0, None, 0.5))
    clean.note_command(2, None)
    clean.note_command(3, None, True)
    assert clean.incomplete is False

    holed = CommandLoopLog(8)
    holed.note_command(1, SimpleNamespace(yaw="not a number"))
    assert holed.incomplete is True

    budget = CommandLoopLog(1)
    budget.note_command(1, None)
    budget.note_command(2, None)
    assert budget.dropped == 1
    assert budget.incomplete is True

    faulted = CommandLoopLog(8)
    faulted._seal.faulted = True
    assert faulted.incomplete is True


def test_the_unreadable_count_is_derived_and_cannot_disagree() -> None:
    """A counter beside the entries can disagree with them EITHER way.

    Incremented before the append it overcounts when the append fails;
    incremented after, it undercounts when the increment fails. Both break the
    partition the summary reports, and a review demonstrated each in turn. The
    count is now read from the entries, so no fault window exists: the number
    IS the entries, not a running total that has to keep up with them.
    """
    log = CommandLoopLog(8)

    # An iteration that cannot become an int: the fault lands before any
    # mutation, so there is no half-recorded entry.
    log.note_command("not an int", SimpleNamespace(yaw="x"))
    assert log.entries() == []
    assert log.unreadable == 0, "counted a hole it never recorded"
    assert log.failed is True
    assert log.incomplete is True

    # And the count tracks the entries themselves, not a separate tally: a hole
    # placed directly in the ledger is seen without anything being incremented.
    other = CommandLoopLog(8)
    other.note_command(1, CalcData(1.0, 2.0, 3.0, None, 0.5))
    assert other.unreadable == 0
    assert other.incomplete is False
    other._entries.append((2, COMMAND_UNREADABLE))
    assert other.unreadable == 1
    assert other.incomplete is True


def test_the_log_drops_rather_than_grows_past_its_budget() -> None:
    """Bounded like the row log, and for the same reason: no unbounded growth."""
    log = CommandLoopLog(2)
    command = CalcData(1.0, 2.0, 3.0, 4.0, 5.0)

    for iteration in (1, 2, 3, 4):
        log.note_command(iteration, command)

    assert len(log.entries()) == 2
    assert [iteration for iteration, _ in log.entries()] == [1, 2]
    assert log.dropped == 2


def test_a_zero_capacity_log_records_iterations_but_no_commands() -> None:
    """The iteration is what stamps a row, so it survives a zero budget."""
    log = CommandLoopLog(0)

    log.note_iteration(9)
    log.note_command(9, CalcData(1.0, 2.0, 3.0, 4.0, 5.0))

    assert log.current_iteration() == 9
    assert log.entries() == []
    assert log.dropped == 1


def test_entries_are_a_copy_so_a_reader_cannot_edit_the_ledger() -> None:
    log = CommandLoopLog(8)
    log.note_command(1, None)

    log.entries().append((99, None))

    assert log.entries() == [(1, None)]


def test_the_null_observer_accepts_everything_and_records_nothing() -> None:
    """Tracing off: the worker pays two no-op calls and keeps no state."""
    NULL_COMMAND_LOOP.note_iteration(1)
    NULL_COMMAND_LOOP.note_command(1, CalcData(1.0, 2.0, 3.0, 4.0, 5.0))
    NULL_COMMAND_LOOP.note_command(2, None, True)

    assert not hasattr(NULL_COMMAND_LOOP, "__dict__")


def test_a_pass_records_its_sample_and_advances_the_iteration() -> None:
    """``PassObserver``'s note (D3): the pass IS the iteration, and its
    sample is the ATTITUDE ledger's cutoff as the pass began, one per pass,
    None while the ledger held nothing."""
    log = CommandLoopLog(8)

    log.note_pass(1, None, None)
    log.note_pass(2, 5, 4)

    capture = log.capture()
    assert capture.passes == ((1, None, None), (2, 5, 4))
    assert (capture.iterations, log.current_iteration()) == (2, 2)
    assert capture.entries == ()
    assert not capture.incomplete


def test_a_pass_sample_is_stored_as_plain_ints() -> None:
    """A reader joins a pass to the ledger on these numbers, and a 5.0 that
    only compares equal is not the ruling it names."""
    log = CommandLoopLog(8)

    log.note_pass(3.0, 5.0, None)  # type: ignore[arg-type]

    (sample,) = log.capture().passes
    assert sample == (3, 5, None)
    assert [type(value) for value in sample] == [int, int, type(None)]
    assert type(log.current_iteration()) is int


def test_a_pass_dropped_for_budget_is_a_hole_and_still_advances() -> None:
    """Bounded like an entry, and counted in the same ``dropped``: a missing
    sample is a hole. The iteration still advances, because it is what
    stamps the pass's rows."""
    log = CommandLoopLog(1)

    log.note_pass(1, 1, 1)
    log.note_pass(2, 2, 2)

    capture = log.capture()
    assert capture.passes == ((1, 1, 1),)
    assert capture.dropped == 1
    assert (capture.iterations, log.current_iteration()) == (2, 2)
    assert capture.incomplete


def test_a_pass_that_cannot_be_read_is_a_fault_and_changes_nothing() -> None:
    """Built whole before anything moves, so a field that will not convert
    leaves the iteration, the count and the samples as they were, and
    fails the log."""
    log = CommandLoopLog(8)
    log.note_pass(1, 1, 1)

    log.note_pass(2, "not a ruling", 1)  # type: ignore[arg-type]

    capture = log.capture()
    assert capture.passes == ((1, 1, 1),)
    assert (capture.iterations, log.current_iteration()) == (1, 1)
    assert capture.failed and capture.incomplete
