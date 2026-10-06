"""D5: every recording is ONE critical section, and a sealed store is frozen.

Driven through each store's real members on real locks. Both shapes of the
defect D5 removes are pinned, because the two stores had one each: the
journal's recorders latched a fault in a SECOND acquisition of the row lock,
and the command log built its entry before taking its lock at all. Either way
a seal and a capture could come between a fault and its latch, and freeze a
store that had lost a recording as clean. Its review found two more gaps,
pinned here as well: a refusal whose own count failed was settled after the
release, and ``fail`` latched the log before it set the flag. The review of
that fix found an older one: an interrupt that cut a recording short went
on with the flag unset, though the store had changed part-way.

The ATTITUDE admission ledger (D2) records under the same rule, so it is
driven the same way, at the end.
"""

from __future__ import annotations

import threading
from collections.abc import Callable
from dataclasses import replace
from types import SimpleNamespace
from typing import Any

import pytest

from navpy.modules.vision.sim import determinism_journal
from navpy.modules.vision.sim.determinism_admission_ledger import (
    AdmissionLedger,
)
from navpy.modules.vision.sim.determinism_command_log import CommandLoopLog
from navpy.modules.vision.sim.determinism_events import (
    ASSOCIATION_COMMITTED,
    DISCARD_UNBRACKETABLE,
    LIFECYCLE_CLOSED,
    OUTPUT_DELIVERED,
    STAGE_STAGED,
    SUBSCRIPTION_OPENED,
    TRUTH_RECORDED,
)
from navpy.modules.vision.sim.determinism_journal import RowJournal
from navpy.modules.vision.sim.determinism_row_log import BoundedRowLog
from navpy.modules.vision.sim.determinism_seal import SealState
from navpy.modules.vision.sim.determinism_slots import command_digest
from navpy.modules.vision.sim.determinism_trace import DeterminismTrace

PERIOD_US = 20_000  # 50 Hz autopilot scheduler period.
# Bounds a FAILING run only. Every wait below is released by an event the test
# itself sets, so a passing run never waits this long.
WAIT_S = 10.0

# Every journal recorder, with every argument but the epoch. Checked against
# the class, so a recorder added later is either driven here or a failure.
RECORDERS: dict[str, dict[str, Any]] = {
    "record_association": dict(outcome=ASSOCIATION_COMMITTED, source_s=1.0),
    "record_truth": dict(outcome=TRUTH_RECORDED),
    "record_stage": dict(outcome=STAGE_STAGED),
    "record_discard": dict(reason=DISCARD_UNBRACKETABLE, victim=None),
    "record_output": dict(
        outcome=OUTPUT_DELIVERED, taken_at_us=1_000, iteration=1
    ),
    "record_lifecycle": dict(outcome=LIFECYCLE_CLOSED, resulting_epoch=2),
    "record_subscription": dict(outcome=SUBSCRIPTION_OPENED, ruling=3),
}
# And every command-log note, given the iteration it is to convert.
NOTES: dict[str, Callable[[CommandLoopLog, Any], None]] = {
    "note_iteration": lambda log, iteration: log.note_iteration(iteration),
    "note_pass": lambda log, iteration: log.note_pass(iteration, 4, 3),
    "note_command": lambda log, iteration: log.note_command(iteration, None),
}


class _Unconvertible:
    """An epoch or iteration that raises on int(): a fault inside the build."""

    def __index__(self) -> int:
        raise RuntimeError("this cannot be converted")

    __int__ = __index__


class _Paused:
    """An epoch or iteration that stops INSIDE its conversion, then raises.

    ``inside`` is set once a recorder is in there, and it stays until ``go``.
    """

    def __init__(self) -> None:
        self.inside = threading.Event()
        self.go = threading.Event()

    def __index__(self) -> int:
        self.inside.set()
        self.go.wait(WAIT_S)
        raise RuntimeError("converted too late")

    __int__ = __index__


class _AfterRelease:
    """A real lock that runs one callback right after its next release.

    That is the window a delayed latch leaves open: a recording's section has
    released, and its fault is counted already or not at all yet. Disarmed
    before the callback runs, because the callback takes this lock too.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._then: Callable[[], object] | None = None

    def arm(self, then: Callable[[], object]) -> None:
        self._then = then

    def __enter__(self) -> bool:
        return self._lock.__enter__()

    def __exit__(self, *exc: object) -> None:
        self._lock.__exit__(*exc)
        then, self._then = self._then, None
        if then is not None:
            then()


class _Contended:
    """A real lock that says when a second caller starts WAITING for it."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.waiting = threading.Event()

    def __enter__(self) -> bool:
        if not self._lock.acquire(blocking=False):
            self.waiting.set()
            self._lock.acquire()
        return True

    def __exit__(self, *exc: object) -> None:
        self._lock.release()

    def locked(self) -> bool:
        return self._lock.locked()


class _WillNotOpen:
    """A lock whose acquisition raises."""

    def __enter__(self) -> bool:
        raise RuntimeError("this lock will not open")

    def __exit__(self, *exc: object) -> None:
        return None


def _cannot_even_fail(self: object) -> None:
    raise MemoryError("no room even to record the failure")


class _Uncountable(int):
    """A refusal count with no room for the next one."""

    def __add__(self, other: int) -> int:
        raise MemoryError("no room to count this refusal")


class _PausedCommand:
    """A command whose every channel read stops INSIDE the digest until
    ``paused.go``, as ``_Paused`` does, and then reads 0.0."""

    def __init__(self, paused: _Paused) -> None:
        self._paused = paused

    def __getattr__(self, name: str) -> float:
        self._paused.inside.set()
        self._paused.go.wait(WAIT_S)
        return 0.0


class _Watched:
    """A command that counts its channel reads, each of them 0.0."""

    def __init__(self) -> None:
        self.reads = 0

    def __getattr__(self, name: str) -> float:
        self.reads += 1
        return 0.0


# The interrupts a recording must pass on. Neither is an Exception.
INTERRUPTS: dict[str, Callable[[], BaseException]] = {
    "keyboard_interrupt": lambda: KeyboardInterrupt("ctrl+c in a recording"),
    "system_exit": lambda: SystemExit(7),
}


class _InterruptedLock:
    """A lock whose acquisition is interrupted."""

    def __init__(self, interrupt: BaseException) -> None:
        self._interrupt = interrupt

    def __enter__(self) -> bool:
        raise self._interrupt

    def __exit__(self, *exc: object) -> None:
        return None


class _InterruptedCommand:
    """A command whose first channel read, inside the digest, is
    interrupted."""

    def __init__(self, interrupt: BaseException) -> None:
        self._interrupt = interrupt

    def __getattr__(self, name: str) -> float:
        raise self._interrupt


def _escaped(call: Callable[[], object]) -> BaseException | None:
    """What ``call`` raised. Caught as BaseException rather than with
    ``pytest.raises``: an interrupt that escaped would end the whole pytest
    session instead of failing one test."""
    try:
        call()
    except BaseException as escaped:  # noqa: BLE001 - the caller asserts
        return escaped
    return None


def test_every_recorder_and_note_is_driven_here() -> None:
    recorders = {
        name for name in vars(RowJournal) if name.startswith("record_")
    }
    notes = {name for name in vars(CommandLoopLog) if name.startswith("note_")}
    assert (recorders, notes) == (set(RECORDERS), set(NOTES))


def test_a_journal_fault_is_counted_before_its_section_releases() -> None:
    """The delayed-latch regression, for every recorder and for ``fail``.

    The recorders latched a fault in a second acquisition, after the first had
    released. Right after that first release a seal and a capture read the
    journal as clean although a row was lost, and the late latch then landed
    in a sealed store. Here the seal and the capture run at exactly that
    moment, and must already see the fault.
    """
    calls: dict[str, Callable[[RowJournal], object]] = {
        name: lambda journal, name=name: getattr(journal, name)(
            epoch=_Unconvertible(), **RECORDERS[name]
        )
        for name in RECORDERS
    }
    calls["fail"] = lambda journal: journal.fail()
    for name, call in sorted(calls.items()):
        journal = RowJournal(PERIOD_US, 8)
        lock = journal._lock = _AfterRelease()
        seen = []

        def seal_and_read(journal: RowJournal = journal) -> None:
            journal.seal()
            seen.append(journal.capture()[1])

        lock.arm(seal_and_read)
        call(journal)

        assert seen, f"{name} never released the row lock"
        assert seen[0].failed, f"{name} released before counting its fault"
        assert seen[0].refused == 0, f"{name} was refused its own recording"


def test_a_fault_is_counted_before_a_latch_that_fails_as_well(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The flag comes FIRST, before any latch work that can fail.

    A latch run before it, and raising, would release the section with nothing
    counted, and the flag would land after the release: in the window read
    here.
    """
    monkeypatch.setattr(BoundedRowLog, "fail", _cannot_even_fail)
    journal = RowJournal(PERIOD_US, 8)
    lock = journal._lock = _AfterRelease()
    seen = []
    lock.arm(lambda: seen.append(journal.capture()[1]))

    journal.record_truth(epoch=_Unconvertible(), outcome=TRUTH_RECORDED)

    assert seen and seen[0].failed


def test_a_callback_fault_count_that_fails_is_a_fault_at_its_release(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """``count_callback_fault`` is a recording like the others (D3): the
    guard around the source's callback calls it on the bus's reader, and a
    count that fails must still leave the journal failed by its release."""

    def cannot_count(self: BoundedRowLog) -> None:
        raise MemoryError("no room to count the callback's fault")

    monkeypatch.setattr(BoundedRowLog, "note_callback_fault", cannot_count)
    journal = RowJournal(PERIOD_US, 8)
    lock = journal._lock = _AfterRelease()
    seen = []
    lock.arm(lambda: seen.append(journal.capture()[1]))

    journal.count_callback_fault()  # must not raise

    assert seen, "the count never released the row lock"
    assert seen[0].failed, "released before counting the failed count"
    assert seen[0].callback_faults == 0


def test_a_command_log_fault_is_counted_before_its_section_releases() -> None:
    """The same property in the worker's ledger, for each note.

    ``note_iteration`` had the journal's defect: its ``except`` took the lock a
    second time to set the flag. ``note_command`` built its entry before taking
    the lock at all, so its fault never reached a section; the paused test
    below is the one that pins that.
    """
    for name, note in sorted(NOTES.items()):
        log = CommandLoopLog(8)
        lock = log._lock = _AfterRelease()
        seen = []

        def seal_and_read(log: CommandLoopLog = log) -> None:
            log.seal()
            seen.append(log.capture())

        lock.arm(seal_and_read)
        note(log, _Unconvertible())

        assert seen, f"{name} never released the ledger's lock"
        assert seen[0].failed, f"{name} released before counting its fault"
        assert seen[0].refused == 0, f"{name} was refused its own note"


def _paused_then_sealed(
    store: RowJournal | CommandLoopLog | AdmissionLedger,
    record: Callable[[_Paused], object],
) -> None:
    """Pause ``record`` inside its build, seal from a second thread, then let
    the recording finish. Asserts that the build held the store's own lock and
    that the seal waited for it."""
    lock = store._lock = _Contended()
    paused = _Paused()
    recorder = threading.Thread(target=record, args=(paused,))
    sealer = threading.Thread(target=store.seal)
    recorder.start()
    try:
        assert paused.inside.wait(WAIT_S), "the recording never built"
        assert lock.locked(), "the build ran outside the store's lock"
        sealer.start()
        assert lock.waiting.wait(WAIT_S), "the seal never waited for the lock"
        assert sealer.is_alive(), "the seal finished inside a recording"
    finally:
        paused.go.set()
        recorder.join(WAIT_S)
        if sealer.ident is not None:
            sealer.join(WAIT_S)
    assert not recorder.is_alive() and not sealer.is_alive()


def test_a_seal_waits_for_a_paused_recording_then_holds_its_fault() -> None:
    """A seal attempted while a recorder is paused INSIDE its critical section
    waits, and then holds the fault (D5's test). For the command log this is
    the regression: its entry was built before the lock was taken, so the
    build ran unguarded and a seal did not have to wait for it."""
    for name in sorted(RECORDERS):
        journal = RowJournal(PERIOD_US, 8)
        _paused_then_sealed(
            journal,
            lambda paused, journal=journal, name=name: getattr(journal, name)(
                epoch=paused, **RECORDERS[name]
            ),
        )
        rows, status = journal.capture()
        assert rows == (), f"{name} recorded a row after all"
        assert status.failed, f"{name}'s fault is missing from the sealed rows"
        assert status.refused == 0, f"{name} took the lock first"
        journal.record_truth(epoch=1, outcome=TRUTH_RECORDED)
        assert journal.capture()[1].refused == 1, "the seal did not hold"

    for name, note in sorted(NOTES.items()):
        log = CommandLoopLog(8)
        _paused_then_sealed(
            log, lambda paused, log=log, note=note: note(log, paused)
        )
        capture = log.capture()
        assert capture.failed, f"{name}'s fault is missing from the sealed log"
        assert (capture.entries, capture.passes, capture.iterations) == (
            (), (), 0
        )
        assert capture.refused == 0, f"{name} took the lock first"
        log.note_iteration(1)
        assert log.capture().refused == 1, "the seal did not hold"


def test_after_the_seal_every_journal_mutation_is_refused_and_counted() -> (
    None
):
    """Every member that changes the journal, classified from the class.

    Each one, after the seal, must leave the rows and every status field as
    they were but the refusal count, which goes up by exactly one. A refusal
    is an observation, not a hole, so a complete journal stays complete.
    """
    mutations: dict[str, Callable[[RowJournal], object]] = {
        name: lambda journal, name=name: getattr(journal, name)(
            epoch=1, **RECORDERS[name]
        )
        for name in RECORDERS
    }
    mutations["fail"] = lambda journal: journal.fail()
    mutations["drain"] = lambda journal: journal.drain()
    mutations["count_callback_fault"] = (
        lambda journal: journal.count_callback_fault()
    )
    others = {"seal", "capture", "watermark_us", "period_us"}
    public = {name for name in vars(RowJournal) if not name.startswith("_")}
    assert public == set(mutations) | others, (
        f"classify: {sorted(public ^ (set(mutations) | others))}"
    )

    for name, mutate in sorted(mutations.items()):
        journal = RowJournal(PERIOD_US, 8)
        journal.record_association(
            epoch=1, outcome=ASSOCIATION_COMMITTED, source_s=1.25
        )
        journal.seal()
        rows, before = journal.capture()
        assert before.complete, "the journal was not complete to begin with"

        handed_out = mutate(journal)

        assert journal.capture() == (
            rows, replace(before, refused=before.refused + 1)
        ), f"{name} was not refused, or changed more than the count"
        assert journal.watermark_us() == 1_250_000
        assert journal.capture()[1].complete, (
            f"{name}'s refusal read as a hole"
        )
        if name == "drain":
            assert handed_out == [], "a refused drain handed out rows"


def test_after_the_seal_every_command_note_is_refused_and_counted() -> None:
    """The same for the worker's ledger, whose mutations are its notes."""
    others = {
        "seal", "current_iteration", "capture", "entries", "iterations",
        "dropped", "failed", "unreadable", "incomplete",
    }
    public = {
        name for name in vars(CommandLoopLog) if not name.startswith("_")
    }
    assert public == set(NOTES) | others, (
        f"classify: {sorted(public ^ (set(NOTES) | others))}"
    )

    for name, note in sorted(NOTES.items()):
        log = CommandLoopLog(8)
        log.note_iteration(1)
        log.note_command(1, None)
        log.seal()
        before = log.capture()

        note(log, 2)

        assert log.capture() == replace(before, refused=before.refused + 1), (
            f"{name} was not refused, or changed more than the count"
        )
        assert log.current_iteration() == 1
        assert not log.capture().incomplete, f"{name}'s refusal read as a hole"


def test_nothing_clears_a_fault_flag(monkeypatch: pytest.MonkeyPatch) -> None:
    """Monotonic (D5): no later clean recording, drain, seal or refusal reads a
    faulted store as clean again. The journal's log is kept from latching the
    fault, so the flag is the only thing that holds it."""
    journal = RowJournal(PERIOD_US, 8)
    with monkeypatch.context() as patch:
        patch.setattr(BoundedRowLog, "fail", _cannot_even_fail)
        journal.record_truth(epoch=_Unconvertible(), outcome=TRUTH_RECORDED)
    assert journal._log.status().failed is False, "the log latched it anyway"
    for name, later in (
        ("a clean recording", lambda: journal.record_truth(
            epoch=1, outcome=TRUTH_RECORDED
        )),
        ("a drain", journal.drain),
        ("a seal", journal.seal),
        ("a refused recording", lambda: journal.record_truth(
            epoch=1, outcome=TRUTH_RECORDED
        )),
    ):
        later()
        assert journal.capture()[1].failed, f"{name} cleared the fault"

    log = CommandLoopLog(8)
    log.note_command(_Unconvertible(), None)
    for name, later in (
        ("a clean note", lambda: log.note_command(1, None)),
        ("a seal", log.seal),
        ("a refused note", lambda: log.note_iteration(2)),
    ):
        later()
        assert log.capture().failed, f"{name} cleared the fault"


def test_a_refusal_is_counted_in_the_capture_that_holds_the_rows() -> None:
    """One acquisition, like every other figure beside the rows: a refusal
    that lands just after a capture's release belongs to the next capture."""
    journal = RowJournal(PERIOD_US, 8)
    lock = journal._lock = _AfterRelease()
    journal.seal()
    lock.arm(lambda: journal.record_truth(epoch=1, outcome=TRUTH_RECORDED))
    counts = [journal.capture()[1].refused, journal.capture()[1].refused]
    assert counts == [0, 1]

    log = CommandLoopLog(8)
    lock = log._lock = _AfterRelease()
    log.seal()
    lock.arm(lambda: log.note_iteration(1))
    assert [log.capture().refused, log.capture().refused] == [0, 1]


def test_a_recording_whose_lock_will_not_open_still_marks_its_store() -> None:
    """Ordered against nothing, but not lost: the flag is a plain store."""
    journal = RowJournal(PERIOD_US, 8)
    real = journal._lock
    journal._lock = _WillNotOpen()
    journal.record_truth(epoch=1, outcome=TRUTH_RECORDED)  # must not raise
    journal._lock = real
    rows, status = journal.capture()
    assert rows == () and status.failed

    log = CommandLoopLog(8)
    real = log._lock
    log._lock = _WillNotOpen()
    log.note_iteration(1)  # must not raise
    log._lock = real
    assert log.capture().failed


def test_a_seal_state_starts_open_and_takes_no_other_attribute() -> None:
    state = SealState()
    assert (state.sealed, state.refused, state.faulted) == (False, 0, False)
    with pytest.raises(AttributeError):
        state.extra = object()  # type: ignore[attr-defined]


def test_a_refusal_whose_count_fails_is_a_fault_at_its_release() -> None:
    """Counting a refusal is work under the lock too, and it can fail.

    A refusal lost without a word would read as one refusal fewer, so its
    fault is settled inside the section like any other: read here, at the
    release. The store is sealed, so nothing else moves, its log included.
    """
    journal = RowJournal(PERIOD_US, 8)
    journal.record_truth(epoch=1, outcome=TRUTH_RECORDED)
    journal.seal()
    rows = journal.capture()[0]
    log_status = journal._log.status()
    journal._seal.refused = _Uncountable(0)
    lock = journal._lock = _AfterRelease()
    at_release = []
    lock.arm(lambda: at_release.append(journal.capture()))

    journal.record_truth(epoch=1, outcome=TRUTH_RECORDED)

    assert at_release, "the refused recording never released the row lock"
    assert at_release[0][1].failed, "released before counting the lost refusal"
    assert at_release[0][0] == rows, "a sealed journal's rows changed"
    assert journal._log.status() == log_status, "a sealed log was latched"

    log = CommandLoopLog(8)
    log.note_iteration(1)
    log.seal()
    before = log.capture()
    log._seal.refused = _Uncountable(0)
    ledger_lock = log._lock = _AfterRelease()
    ledger_at_release = []
    ledger_lock.arm(lambda: ledger_at_release.append(log.capture()))

    log.note_iteration(2)

    assert ledger_at_release, "the refused note never released its lock"
    after = ledger_at_release[0]
    assert after.failed, "released before counting the lost refusal"
    assert (after.entries, after.iterations) == (
        before.entries, before.iterations
    ), "a sealed ledger changed"


def test_a_facade_fault_is_flagged_before_the_log_latches(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """``fail`` records a fault the facade hit before any row lock: its read
    of the worker iteration (D5). That fault is the recording's content, so
    the flag comes first here as well, and the log's latch finds it set."""
    trace = DeterminismTrace(PERIOD_US, capacity=8)
    journal = trace.journal
    real_fail = BoundedRowLog.fail
    entered = []

    def watched_fail(log: BoundedRowLog) -> None:
        entered.append((journal._lock.locked(), journal._seal.faulted))
        real_fail(log)

    def unreadable(self: CommandLoopLog) -> int | None:
        raise RuntimeError("the worker iteration cannot be read")

    monkeypatch.setattr(CommandLoopLog, "current_iteration", unreadable)
    monkeypatch.setattr(BoundedRowLog, "fail", watched_fail)

    trace.record_output(epoch=1, outcome=OUTPUT_DELIVERED, taken_at_us=None)

    assert entered == [(True, True)], (
        f"the log's latch began as (lock held, flag set) = {entered}"
    )


def test_a_command_is_digested_under_its_ledger_lock() -> None:
    """The digest reads the command's channels, and a getter could do
    anything, so it runs under the lock (D5 moves note_command's building
    there). Paused inside the digest, the lock is held and a seal waits; then
    the note lands whole, with the digest of what was read."""
    log = CommandLoopLog(8)
    _paused_then_sealed(
        log, lambda paused: log.note_command(1, _PausedCommand(paused))
    )
    read = SimpleNamespace(
        yaw=0.0, pitch=0.0, cmd_roll=0.0, cmd_pitch=0.0, cmd_thr=0.0
    )
    capture = log.capture()
    assert capture.entries == ((1, command_digest(read)),)
    assert (capture.failed, capture.refused) == (False, 0)
    log.note_iteration(2)
    assert log.capture().refused == 1, "the seal did not hold"


def test_a_sealed_note_never_reads_its_command() -> None:
    """Refused before anything is built: a digest taken first would run the
    command's getters in a store the seal had closed."""
    log = CommandLoopLog(8)
    log.seal()
    command = _Watched()

    log.note_command(1, command)

    assert command.reads == 0, "a sealed note read its command"
    assert log.capture().refused == 1


@pytest.mark.parametrize("kind", sorted(INTERRUPTS))
def test_an_interrupt_is_flagged_before_its_section_releases(
    monkeypatch: pytest.MonkeyPatch, kind: str
) -> None:
    """The re-review's reproduction: an interrupt lands in the association
    recorder after the watermark moved and before its row was appended. The
    store has changed part-way, so its flag must be set already at the
    release, and the interrupt must come out as itself, even from a log that
    cannot latch: latch work that raised would take the interrupt's place."""
    interrupt = INTERRUPTS[kind]()

    def cut_short(*_args: object) -> None:
        raise interrupt

    monkeypatch.setattr(determinism_journal, "association_row", cut_short)
    monkeypatch.setattr(BoundedRowLog, "fail", _cannot_even_fail)
    journal = RowJournal(PERIOD_US, 8)
    lock = journal._lock = _AfterRelease()
    at_release = []
    lock.arm(lambda: at_release.append(journal.capture()))

    escaped = _escaped(lambda: journal.record_association(
        epoch=1, outcome=ASSOCIATION_COMMITTED, source_s=1.0
    ))

    assert escaped is interrupt, f"the interrupt came out as {escaped!r}"
    assert journal.watermark_us() == 1_000_000, "no part-way change to flag"
    assert at_release, "the recording never released the row lock"
    rows, status = at_release[0]
    assert rows == () and status.failed, "released before the flag was set"

    log = CommandLoopLog(8)
    ledger_lock = log._lock = _AfterRelease()
    ledger_at_release = []
    ledger_lock.arm(lambda: ledger_at_release.append(log.capture()))

    escaped = _escaped(
        lambda: log.note_command(1, _InterruptedCommand(interrupt))
    )

    assert escaped is interrupt, f"the interrupt came out as {escaped!r}"
    assert ledger_at_release, "the note never released the ledger's lock"
    assert ledger_at_release[0].failed, "released before the flag was set"


@pytest.mark.parametrize("kind", sorted(INTERRUPTS))
def test_an_interrupt_while_the_lock_is_taken_still_marks_its_store(
    kind: str,
) -> None:
    """Like a lock that will not open: ordered against nothing, but a
    recording that reached its store was lost, so the flag says so. The
    interrupt goes on."""
    interrupt = INTERRUPTS[kind]()
    journal = RowJournal(PERIOD_US, 8)
    real = journal._lock
    journal._lock = _InterruptedLock(interrupt)
    escaped = _escaped(
        lambda: journal.record_truth(epoch=1, outcome=TRUTH_RECORDED)
    )
    journal._lock = real
    assert escaped is interrupt, f"the interrupt came out as {escaped!r}"
    rows, status = journal.capture()
    assert rows == () and status.failed

    log = CommandLoopLog(8)
    real = log._lock
    log._lock = _InterruptedLock(interrupt)
    escaped = _escaped(lambda: log.note_iteration(1))
    log._lock = real
    assert escaped is interrupt, f"the interrupt came out as {escaped!r}"
    assert log.capture().failed


@pytest.mark.parametrize("kind", sorted(INTERRUPTS))
def test_an_interrupted_refusal_is_flagged_and_changes_nothing_sealed(
    kind: str,
) -> None:
    """A sealed store keeps its contents through an interrupt as well: the
    flag says the refusal went uncounted, and the rows and the log stay as
    they were."""
    interrupt = INTERRUPTS[kind]()

    class _Interrupted(int):
        def __add__(self, other: int) -> int:
            raise interrupt

    journal = RowJournal(PERIOD_US, 8)
    journal.record_truth(epoch=1, outcome=TRUTH_RECORDED)
    journal.seal()
    rows = journal.capture()[0]
    log_status = journal._log.status()
    journal._seal.refused = _Interrupted(0)

    escaped = _escaped(
        lambda: journal.record_truth(epoch=1, outcome=TRUTH_RECORDED)
    )

    assert escaped is interrupt, f"the interrupt came out as {escaped!r}"
    after, status = journal.capture()
    assert status.failed, "the interrupted refusal left no mark"
    assert after == rows, "a sealed journal's rows changed"
    assert journal._log.status() == log_status, "a sealed log was latched"


def _ruling(number: Any, *, accepted: bool = True) -> SimpleNamespace:
    """A ruling with ``AdmissionRuling``'s fields, its number any object: the
    ledger converts every field inside its one section."""
    return SimpleNamespace(
        message_type="ATTITUDE",
        ruling=number,
        time_boot_ms=1_000,
        accepted=accepted,
        reason=None if accepted else "stale_boot",
    )


def test_a_ledger_fault_is_counted_before_its_section_releases() -> None:
    """The ATTITUDE ledger (D2) under the same rule: a ruling whose fields
    will not convert is a fault inside the append's one section, counted
    already when a seal and a capture run at its release, with nothing else
    moved."""
    ledger = AdmissionLedger(8)
    lock = ledger._lock = _AfterRelease()
    seen = []

    def seal_and_read() -> None:
        ledger.seal()
        seen.append(ledger.capture())

    lock.arm(seal_and_read)
    ledger.append(_ruling(_Unconvertible()))

    assert seen, "the append never released the ledger's lock"
    assert seen[0].failed, "the append released before counting its fault"
    assert seen[0].refused == 0, "the append was refused its own ruling"
    assert (seen[0].entries, seen[0].observed, seen[0].admitted) == (
        (), None, None
    ), "a ruling that could not be read moved the ledger"


def test_a_seal_waits_for_a_paused_append_then_holds_its_fault() -> None:
    """Paused inside the append's build, the ledger's lock is held and a
    seal waits for it; then the fault is in the sealed ledger, and the next
    append is refused."""
    ledger = AdmissionLedger(8)
    _paused_then_sealed(ledger, lambda paused: ledger.append(_ruling(paused)))

    capture = ledger.capture()
    assert capture.failed, "the fault is missing from the sealed ledger"
    assert (capture.entries, capture.observed, capture.refused) == (
        (), None, 0
    )
    ledger.append(_ruling(1))
    assert ledger.capture().refused == 1, "the seal did not hold"


def test_after_the_seal_every_ledger_append_is_refused_and_counted() -> None:
    """Its one mutation is ``append``, classified from the class like the
    others'. Refused, it leaves the entries and both live figures as they
    were, and a refusal is not a hole."""
    others = {"cutoff", "seal", "capture"}
    public = {
        name for name in vars(AdmissionLedger) if not name.startswith("_")
    }
    assert public == {"append"} | others, (
        f"classify: {sorted(public ^ ({'append'} | others))}"
    )

    ledger = AdmissionLedger(8)
    ledger.append(_ruling(1))
    ledger.seal()
    before = ledger.capture()

    ledger.append(_ruling(2))
    ledger.append(_ruling(3, accepted=False))

    assert ledger.capture() == replace(before, refused=before.refused + 2), (
        "an append was not refused, or changed more than the count"
    )
    assert ledger.cutoff() == (1, 1)
    assert not ledger.capture().incomplete, "a refusal read as a hole"


def test_nothing_clears_a_ledger_fault_flag() -> None:
    """Monotonic (D5) in the ledger too: no later append, cutoff, seal or
    refusal reads a faulted ledger as clean again."""
    ledger = AdmissionLedger(8)
    ledger.append(_ruling(_Unconvertible()))
    for name, later in (
        ("a clean append", lambda: ledger.append(_ruling(1))),
        ("a cutoff", ledger.cutoff),
        ("a seal", ledger.seal),
        ("a refused append", lambda: ledger.append(_ruling(2))),
    ):
        later()
        assert ledger.capture().failed, f"{name} cleared the fault"


def test_a_ledger_refusal_whose_count_fails_is_a_fault_at_its_release() -> (
    None
):
    """Its refusal is counted under the lock as well, so a count that fails
    is settled there, and the sealed entries stay as they were."""
    ledger = AdmissionLedger(8)
    ledger.append(_ruling(1))
    ledger.seal()
    before = ledger.capture()
    ledger._seal.refused = _Uncountable(0)
    lock = ledger._lock = _AfterRelease()
    at_release = []
    lock.arm(lambda: at_release.append(ledger.capture()))

    ledger.append(_ruling(2))

    assert at_release, "the refused append never released the ledger's lock"
    assert at_release[0].failed, "released before counting the lost refusal"
    assert (at_release[0].entries, at_release[0].observed) == (
        before.entries, before.observed
    ), "a sealed ledger changed"


def test_a_ledger_whose_lock_will_not_open_says_so_and_raises_nothing() -> (
    None
):
    """Both of its callers are command paths: ``append`` on the bus's reader,
    ``cutoff`` on the worker's pass and in the source's recorders. Neither
    raises. The append is lost with the flag set, and a cutoff it could not
    give reads (None, None) with the flag set, so it cannot pass for a clean
    one."""
    appended = AdmissionLedger(8)
    real = appended._lock
    appended._lock = _WillNotOpen()
    appended.append(_ruling(1))  # must not raise
    appended._lock = real
    assert appended.capture().failed
    assert appended.capture().entries == ()

    read = AdmissionLedger(8)
    read.append(_ruling(1))
    real = read._lock
    read._lock = _WillNotOpen()
    assert read.cutoff() == (None, None)
    read._lock = real
    assert read.capture().failed, "a cutoff it could not give read as clean"
    assert read.cutoff() == (1, 1), "the read changed the ledger"


@pytest.mark.parametrize("kind", sorted(INTERRUPTS))
def test_a_ledger_interrupt_is_flagged_and_passed_on(kind: str) -> None:
    """Inside the append's section, and while its lock is taken, the
    interrupt comes out as itself and the ledger is failed by the release.
    A cutoff's is passed on as well: the read changed nothing, and the
    recording it cost belongs to its caller."""
    interrupt = INTERRUPTS[kind]()

    class _Interrupting:
        def __index__(self) -> int:
            raise interrupt

        __int__ = __index__

    ledger = AdmissionLedger(8)
    lock = ledger._lock = _AfterRelease()
    at_release = []
    lock.arm(lambda: at_release.append(ledger.capture()))

    escaped = _escaped(lambda: ledger.append(_ruling(_Interrupting())))

    assert escaped is interrupt, f"the interrupt came out as {escaped!r}"
    assert at_release, "the append never released the ledger's lock"
    assert at_release[0].failed, "released before the flag was set"
    assert at_release[0].entries == ()

    taken = AdmissionLedger(8)
    real = taken._lock
    taken._lock = _InterruptedLock(interrupt)
    escaped = _escaped(lambda: taken.append(_ruling(1)))
    assert escaped is interrupt, f"the interrupt came out as {escaped!r}"
    escaped = _escaped(taken.cutoff)
    taken._lock = real
    assert escaped is interrupt, f"the cutoff's came out as {escaped!r}"
    assert taken.capture().failed
