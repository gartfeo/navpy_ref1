"""Unit tests for the bounded determinism trace and its post-leg summary.

These pin the properties later landings build on: a full or faulted recorder
marks itself incomplete rather than lying, no recorder fault can escape into
the caller (which is a command path), and the source watermark never moves
backwards.
"""

from __future__ import annotations

import ast
import inspect
import textwrap
import threading
from types import SimpleNamespace

from navpy.modules.navigation.calc_data import CalcData
from navpy.modules.vehicle.admission_tap import AdmissionRuling, AdmissionTap
from navpy.modules.vehicle.inbound_router import (
    AUTOPILOT_TELEMETRY_TYPES,
    REJECT_STALE_BOOT,
    message_boot_time_ms,
)
from navpy.modules.vision.sim import determinism_events
from navpy.modules.vision.sim.determinism_admission_ledger import PassObserver
from navpy.modules.vision.sim.determinism_journal import RowJournal
from navpy.modules.vision.sim import determinism_trace as trace_module
from navpy.modules.vision.sim.determinism_events import (
    ASSOCIATION_COMMITTED,
    TRUTH_UNREADABLE,
    TRUTH_RECORDED,
    STAGE_FENCED,
    DISCARD_UNBRACKETABLE,
    ASSOCIATION_FENCED,
    ASSOCIATION_REFUSED,
    DISCARD_PUBLISH_OVERWRITTEN,
    EVENT_ASSOCIATION,
    EVENT_DECIMATE,
    EVENT_LIFECYCLE,
    EVENT_OUTPUT,
    EVENT_STAGE,
    EVENT_SUBSCRIPTION,
    EVENT_TRUTH,
    EVENT_VIOLATION,
    LIFECYCLE_ACTIVATED,
    LIFECYCLE_CLOSED,
    OUTPUT_DISPATCHED,
    OUTPUT_EMPTY,
    STAGE_PAYLOAD_DIGEST,
    STAGE_STAGED,
    SUBSCRIPTION_OPENED,
    VIOLATION_SOURCE_REGRESSED,
    VIOLATION_SOURCE_REPEATED,
)
from navpy.modules.vision.sim.determinism_row_log import BoundedRowLog
from navpy.modules.vision.sim.determinism_trace import (
    DEFAULT_TRACE_CAPACITY,
    DETERMINISM_TRACE_CAPACITY_ENV,
    DETERMINISM_TRACE_ENV,
    DeterminismTrace,
    build_trace,
)
from navpy.modules.vision.sim.determinism_slots import (
    PAYLOAD_UNREADABLE,
    frame_digest,
)
from navpy.modules.vision.sim import determinism_trace_gate as gate
from navpy.modules.vision.sim.determinism_trace_summary import (
    summarise,
)


PERIOD_US = 20_000  # 50 Hz autopilot scheduler period.


def _frame(
    source_timestamp_s: float,
    *,
    u_px: float = 1.5,
    v_px: float = -2.5,
    pitch_deg: float = 3.0,
    roll_deg: float = 4.0,
) -> SimpleNamespace:
    return SimpleNamespace(
        pixel=SimpleNamespace(
            u_px=u_px,
            v_px=v_px,
            aircraft_pitch_deg=pitch_deg,
            aircraft_roll_deg=roll_deg,
            source_timestamp_s=source_timestamp_s,
        )
    )


class _WatchedLock:
    """A real lock that reports whenever both watched locks are held at once.

    ``taken`` records every lock this watcher ever actually acquired. That is
    not bookkeeping: it is the positive control. A watcher installed where
    nothing acquires it reports no overlap forever, and the assertion built on
    it reads as a pass. That happened -- splitting the row lock out onto
    ``RowJournal`` left this test assigning to ``trace._lock``, an attribute the
    facade no longer has, so the row side went unwatched and the whole suite
    still came back green.
    """

    def __init__(
        self,
        name: str,
        held: set[str],
        overlaps: list[tuple],
        taken: set[str],
    ) -> None:
        self._name = name
        self._held = held
        self._overlaps = overlaps
        self._taken = taken
        self._lock = threading.Lock()

    def _took(self) -> None:
        self._taken.add(self._name)
        self._held.add(self._name)
        if len(self._held) > 1:
            self._overlaps.append(tuple(sorted(self._held)))

    def __enter__(self) -> _WatchedLock:
        self._lock.acquire()
        self._took()
        return self

    def __exit__(self, *_: object) -> None:
        self._held.discard(self._name)
        self._lock.release()

    def acquire(self, *args: object, **kwargs: object) -> bool:
        acquired = self._lock.acquire(*args, **kwargs)
        if acquired:
            self._took()
        return acquired

    def release(self) -> None:
        self._held.discard(self._name)
        self._lock.release()


def test_a_frame_whose_payload_could_not_be_read_invalidates_the_verdict() -> (
    None
):
    """An undigestable payload is a HOLE, and holes invalidate a run.

    This regressed once and the regression is the reason the test exists.
    ``frame_digest`` used to RAISE on an out-of-range field, and ``record_stage``
    caught the raise and latched ``failed``. Broadening the digest to answer
    instead of raise removed that signal: the row carried a bare ``None``, which
    is what "there was no payload" already means, and the run read clean. Two
    different unreadable frames left identical evidence.
    """
    trace = DeterminismTrace(PERIOD_US, capacity=8)
    trace.record_stage(
        epoch=1, outcome=STAGE_STAGED, frame=_frame(1.0, u_px=10**400)
    )

    row = list(trace.capture().rows)[0]
    assert row[7] == PAYLOAD_UNREADABLE
    summary = summarise(trace, epoch=1)
    assert summary["payload_unreadable"] is True
    assert summary["failed"] is False, "the recorder is fine; the input was not"
    assert summary["complete"] is False


def test_a_stage_with_no_frame_at_all_is_not_a_hole() -> None:
    """The counterpart: "nothing to digest" must not read as a hole.

    Two runs that both staged nothing HAVE been shown to agree, so this must
    stay complete or the verdict would be useless on any refused staging.
    """
    trace = DeterminismTrace(PERIOD_US, capacity=8)
    trace.record_stage(epoch=1, outcome=STAGE_STAGED, sample=_frame(1.0))

    assert list(trace.capture().rows)[0][7] is None
    summary = summarise(trace, epoch=1)
    assert summary["payload_unreadable"] is False
    assert summary["complete"] is True


def test_a_pixel_getter_that_raises_is_an_unreadable_payload() -> None:
    """The pixel read used to sit OUTSIDE the try, so a getter escaped."""

    class Unreadable:
        @property
        def pixel(self) -> object:
            raise RuntimeError("no pixel for you")

    trace = DeterminismTrace(PERIOD_US, capacity=8)
    trace.record_stage(epoch=1, outcome=STAGE_STAGED, frame=Unreadable())

    assert frame_digest(Unreadable()) == PAYLOAD_UNREADABLE
    assert summarise(trace, epoch=1)["complete"] is False


def _declared_outcomes(prefix: str) -> tuple[str, ...]:
    """Every string outcome the events vocabulary declares under one prefix.

    READ FROM THE MODULE, not listed here. The runtime lock test can only see a
    branch it takes, and a review found that gap for real: a nesting placed on
    the rejected-output branch went unmeasured because the test drove only
    delivered and empty. Adding those values by hand would have moved the hole
    to whatever outcome is declared next, not closed it. Deriving them means a
    new outcome is either driven or the coverage assertion fails.

    Filtered to ``str`` on purpose: two trailing COLUMN INDEXES share these
    prefixes (``STAGE_PAYLOAD_DIGEST``, ``OUTPUT_WORKER_ITERATION``) and are
    ints, not outcomes.
    """
    return tuple(
        value
        for name, value in vars(determinism_events).items()
        if name.startswith(prefix) and isinstance(value, str)
    )


def test_no_two_names_of_one_kind_share_a_value() -> None:
    """A row carries its event and its outcome as VALUES, so two names that
    spell one value merge in every row and every count. A test comparing a
    constant with itself passes either way: a mutation battery set
    LIFECYCLE_CLOSED to "activated" and all 200 focused tests still passed.

    Grouped by the name's first word (EVENT, ASSOCIATION, ..., LIFECYCLE) and
    read from the module, so a group declared later is covered without being
    listed here. A value may repeat ACROSS groups: ASSOCIATION_FENCED and
    STAGE_FENCED are outcomes of different events.
    """
    groups: dict[str, list[str]] = {}
    for name, value in vars(determinism_events).items():
        if name.isupper() and isinstance(value, str):
            groups.setdefault(name.split("_", 1)[0], []).append(value)
    # The control: a filter that matched nothing would pass vacuously.
    assert {"EVENT", "OUTPUT", "LIFECYCLE"} <= set(groups), sorted(groups)
    for group, values in groups.items():
        assert len(set(values)) == len(values), (
            f"{group}_* spells one value twice: {sorted(values)}"
        )


def test_no_member_can_record_a_row_without_latching_a_hole_in_it() -> None:
    """Driven, not read. Two AST versions of this claimed more than they checked.

    The first looked for ``self._rows.append`` by name, so ``extend`` walked past
    it. The second matched any mutation of ``self._rows``, which fixed that but
    still could not see a hole added inside ``drain``, and rejected a harmless
    ``self._rows.copy()`` as a mutation. Reading source for "does this add a row"
    keeps answering a question about SYNTAX when the question is about BEHAVIOUR.

    So every public member is CALLED. Anything that takes a row is handed a
    staging hole and must latch it; anything else must not grow the row list.
    A local alias, an ``extend``, a module-level helper, an addition inside
    ``drain`` -- none of them can satisfy this by being spelled differently,
    because none of them is read.

    The member list is asserted COMPLETE against the class, so a new public
    member cannot quietly opt out of the check: it fails here until it is
    classified.

    Scope: this is the class's own API. Python cannot stop outside code reaching
    ``_rows`` directly, and a review confirmed that it can. No production code
    does -- ``BoundedRowLog`` is constructed only by ``DeterminismTrace``.
    """
    stage_hole = (
        EVENT_STAGE, 1, STAGE_STAGED, 0, 0, None, None, PAYLOAD_UNREADABLE
    )
    clean_row = (EVENT_OUTPUT, 1, OUTPUT_DISPATCHED, 0, 0, None, None, 1)

    takes_a_row = {"append", "note_violation"}
    takes_nothing = {
        "status", "drain", "snapshot", "fail", "note_callback_fault",
    }
    properties = {"capacity"}
    public = {
        name
        for name in vars(BoundedRowLog)
        if not name.startswith("_")
    }
    assert public == takes_a_row | takes_nothing | properties, (
        "a public member is unclassified, so this test does not cover it: "
        f"{sorted(public - takes_a_row - takes_nothing - properties)}"
    )

    for name in sorted(takes_a_row):
        log = BoundedRowLog(8)
        getattr(log, name)(stage_hole)
        assert log.status().payload_unreadable is True, (
            f"{name} recorded a staging hole without latching it"
        )

    def is_stage_hole(row: tuple) -> bool:
        return (
            len(row) > determinism_events.STAGE_PAYLOAD_DIGEST
            and row[0] == EVENT_STAGE
            and row[determinism_events.STAGE_PAYLOAD_DIGEST] == PAYLOAD_UNREADABLE
        )

    for name in sorted(takes_nothing):
        log = BoundedRowLog(8)
        log.append(clean_row)
        held = len(log.snapshot())
        returned = getattr(log, name)()
        assert len(log.snapshot()) <= held, f"{name} added a row"
        # What it HOLDS and what it HANDS BACK, both. A row count alone cannot
        # see a hole added inside ``drain``, because ``drain`` empties the list
        # on its way out -- a review put one there and passed.
        produced = list(log.snapshot())
        if isinstance(returned, list):
            produced += returned
        assert not [row for row in produced if is_stage_hole(row)], (
            f"{name} produced a staging hole out of nowhere"
        )
        assert log.status().payload_unreadable is False, (
            f"{name} latched a hole out of nowhere"
        )

    # And the latch outlives the row: applied BEFORE the capacity check, for the
    # same reason note_violation latches before appending -- an overflow that
    # drops the row must not also erase the fact that the row was holed.
    full = BoundedRowLog(1)
    full.append(clean_row)
    full.append(stage_hole)
    assert full.status().overflowed is True
    assert stage_hole not in full.snapshot(), "the row was not dropped"
    assert full.status().payload_unreadable is True, (
        "the dropped row took its own hole with it"
    )


def test_the_payload_latch_reads_column_seven_only_on_a_stage_row() -> None:
    """The event is checked first, and the check is load-bearing here.

    ``STAGE_PAYLOAD_DIGEST`` is 7 and ``OUTPUT_WORKER_ITERATION`` is 7: the same
    column carries a payload digest on one row type and a loop count on another.
    Reading it without checking the event was safe only by luck -- an int never
    equals ``PAYLOAD_UNREADABLE``. No event produces a non-stage row carrying
    those bytes today, which is exactly why luck could stand in for a contract,
    so the row is built here by hand. The next row type to end at index 7 is
    covered by a check rather than by coincidence.
    """
    assert STAGE_PAYLOAD_DIGEST == 7

    log = BoundedRowLog(8)
    log.append((EVENT_OUTPUT, 1, OUTPUT_DISPATCHED, 0, 0, None, None, 7))
    assert log.status().payload_unreadable is False

    log.append(
        (EVENT_OUTPUT, 1, OUTPUT_DISPATCHED, 0, 0, None, None, PAYLOAD_UNREADABLE)
    )
    assert log.status().payload_unreadable is False, (
        "latched a payload hole off a row that carries no payload digest"
    )

    # A short row cannot be indexed at all, and must not raise into a recorder.
    log.append((EVENT_STAGE, 1, STAGE_STAGED))
    assert log.status().payload_unreadable is False


def test_no_recording_call_ever_holds_both_locks_at_once() -> None:
    """The lock-ordering claim, MEASURED rather than read off the source.

    Two locks held at once is the shape a future deadlock grows from. The
    source-level check in test_direct_poi_pixel_source can only reject
    shapes it knows to look for, and a review found five nesting forms it
    missed: a nested function, a chained alias, an annotated alias, the public
    ``command_log`` property, and a bare ``acquire()``. This test does not care
    how a nesting is written. Every lock is replaced by a real lock sharing one
    ``held`` set -- the rows', the command log's, the ATTITUDE ledger's and the
    router tap's that feeds it -- so ANY overlap is recorded whatever route
    reached it.

    It cannot see a branch it never takes, though, and that gap was real TWICE.
    An early version made ONE association call, so a nesting after the watermark
    early-return went unmeasured. The version after that drove the refusal and
    regression paths and claimed "every recorder" with them -- an overstatement:
    it drove two of the five declared output outcomes, and a review nested the
    locks on the rejected branch and passed.

    What is claimed now is what is checked below. Every outcome the events
    vocabulary DECLARES is driven, read off the module instead of listed here,
    and the coverage is asserted from the recorded rows. A new outcome added to
    the vocabulary is therefore either driven or a failure, and every recorder's
    event is asserted present in its own right, because two of them were being
    driven without being checked. Both epochs are driven for every outcome, since
    a review hid a nesting behind an epoch AND outcome pair that the sweep never
    visited. That is still not every branch -- one keyed on a frame shape, a
    watermark or a capacity is covered only by the inputs varied here, which is
    why the sweep runs against an overflowed log as well -- but it is no longer a
    claim wider than its evidence.
    """
    held: set[str] = set()
    overlaps: list[tuple] = []
    taken: set[str] = set()

    def watched(capacity: int) -> DeterminismTrace:
        """A trace whose three locks share ONE held-set with every other one.

        Sharing it is what makes an overlap visible no matter which trace or
        which thread reached it. That is not by itself enough: sharing preserves
        instrumentation for the calls that RUN, not the paths they take, and a
        review nested the locks on a discard that only happens once the log has
        overflowed. So the full sweep is run against BOTH traces below, and the
        split costs no coverage only because of that.
        """
        built = DeterminismTrace(PERIOD_US, capacity=capacity)
        # The row lock lives on the JOURNAL, not on the facade. Assigning to
        # ``built._lock`` instead silently instruments nothing, which is exactly
        # what this test did until ``taken`` below caught it.
        built.journal._lock = _WatchedLock("row", held, overlaps, taken)
        built.command_log._lock = _WatchedLock(
            "command", held, overlaps, taken
        )
        built.ledger._lock = _WatchedLock("ledger", held, overlaps, taken)
        return built

    def tapped(target: DeterminismTrace) -> AdmissionTap:
        """A real router tap feeding ``target``'s ledger, its own lock
        watched with the rest. Watched BEFORE the subscription, which keeps
        the lock it was given: the tap numbers a ruling under that lock and
        hands it on outside it, so an append that nested the ledger's lock
        inside the tap's would show here."""
        tap = AdmissionTap(message_boot_time_ms, AUTOPILOT_TELEMETRY_TYPES)
        tap._lock = _WatchedLock("tap", held, overlaps, taken)
        tap.subscribe("ATTITUDE", target.ledger.append)
        return tap

    # Two traces, because the two goals fight over one row budget. The sweep
    # needs its rows to SURVIVE so the coverage assertion can read them back;
    # the overflow branch needs the budget exceeded. Running the sweep in a tiny
    # buffer silently evicted an outcome and reported it as never driven -- which
    # is the coverage assertion doing its job, and the reason for the split.
    trace = watched(256)
    tiny = watched(2)
    trace_tap, tiny_tap = tapped(trace), tapped(tiny)

    frame = _frame(1.0)
    trace.command_log.note_iteration(1)
    trace.command_log.note_command(1, CalcData(1.0, 2.0, 3.0, None, 0.5))
    trace.command_log.note_command(2, None)
    trace.command_log.note_command(3, None, True)
    trace.command_log.note_command(4, SimpleNamespace(yaw="unreadable"))

    # Association: advance, then REPEAT the same source time, then REGRESS it,
    # then refuse with no stamp at all. The middle two take the early returns
    # and latch violations; the last records no source time.
    trace.record_association(
        epoch=1, outcome=ASSOCIATION_COMMITTED, source_s=1.0
    )
    trace.record_association(
        epoch=1, outcome=ASSOCIATION_COMMITTED, source_s=1.0
    )
    trace.record_association(
        epoch=1, outcome=ASSOCIATION_COMMITTED, source_s=0.5
    )
    trace.record_association(epoch=1, outcome=ASSOCIATION_REFUSED)

    # Then EVERY remaining declared outcome for every recorder, so no branch
    # keyed on an outcome value can hide from this. The digest inputs are
    # varied across the sweep: a real frame, nothing at all, and a payload that
    # cannot be packed.
    unreadable = _frame(1.0, u_px=10**400)

    def sweep(target: DeterminismTrace) -> None:
        """Drive every declared outcome of every recorder, in BOTH epochs.

        Both epochs, because a review nested the locks behind
        ``epoch == 2 and outcome == "delivered"`` and passed: the sweep varied
        the epoch WITH the outcome index, so each outcome was seen in ONE epoch
        only and the pair it hid behind was among those never visited. (An
        earlier wording here said no pair was visited at all, which overstated
        it -- a subset was.)
        """
        for epoch in (1, 2):
            for outcome in _declared_outcomes("ASSOCIATION_"):
                target.record_association(
                    epoch=epoch, outcome=outcome, source_s=2.0 + epoch
                )
            for outcome in _declared_outcomes("TRUTH_"):
                target.record_truth(epoch=epoch, outcome=outcome)
            for index, outcome in enumerate(_declared_outcomes("STAGE_")):
                target.record_stage(
                    epoch=epoch,
                    outcome=outcome,
                    sample=frame if index % 2 else None,
                    frame=(frame, None, unreadable)[index % 3],
                )
            for index, outcome in enumerate(_declared_outcomes("OUTPUT_")):
                target.record_output(
                    epoch=epoch,
                    outcome=outcome,
                    taken_at_us=None if index % 2 else 1_000 * (index + 1),
                    frame=(frame, None, unreadable)[index % 3],
                )
            for index, reason in enumerate(_declared_outcomes("DISCARD_")):
                target.record_discard(
                    epoch=epoch,
                    reason=reason,
                    victim=frame if index % 2 else None,
                )
            for outcome in _declared_outcomes("LIFECYCLE_"):
                target.record_lifecycle(
                    epoch=epoch, outcome=outcome, resulting_epoch=epoch + 1
                )
            for outcome in _declared_outcomes("SUBSCRIPTION_"):
                target.record_subscription(epoch=epoch, outcome=outcome)

    def admissions(target: DeterminismTrace, tap: AdmissionTap) -> None:
        """The ledger's side, in both epochs: rulings through the real tap,
        admitted, stampless and rejected; each pass sampled by the worker's
        observer, and a command noted; and the source's callback guarded,
        once returning and once raising, which the guard counts."""
        observer = PassObserver(target.ledger, target.command_log)

        def raising(message: object) -> None:
            raise RuntimeError("the source's callback failed")

        for epoch in (1, 2):
            tap.rule(
                "ATTITUDE", SimpleNamespace(time_boot_ms=1_000 * epoch),
                True, None,
            )
            tap.rule("ATTITUDE", SimpleNamespace(), True, None)
            tap.rule(
                "ATTITUDE", SimpleNamespace(time_boot_ms=500),
                False, REJECT_STALE_BOOT,
            )
            observer.note_iteration(epoch)
            observer.note_command(epoch, None)
            target.guard_callback(lambda message: None)(epoch)
            try:
                target.guard_callback(raising)(epoch)
            except RuntimeError:
                pass

    # Cleared first, so the coverage assertion below reads ONLY sweep rows. A
    # review supplied a missing committed-association outcome from the four
    # setup calls above and passed while the sweep itself never produced it.
    trace.journal.drain()
    sweep(trace)
    admissions(trace, trace_tap)

    # The WHOLE sweep again on the tiny buffer, so every recorder is also driven
    # in the overflowed state. Driving only two of them there was a real gap: a
    # review nested the locks in ``record_discard`` behind a check on
    # ``overflowed`` and passed both tests, because nothing ever discarded while
    # the log was full.
    sweep(tiny)
    admissions(tiny, tiny_tap)
    sweep(tiny)
    admissions(tiny, tiny_tap)
    # And a ruling the ATTITUDE ledger cannot read, so its fault branch runs
    # under the watched locks too.
    tiny.ledger.append(SimpleNamespace(
        ruling=object(), time_boot_ms=None, accepted=True, reason=None
    ))
    # And once more after every seal, so every recorder's REFUSAL
    # branch runs under the watched locks as well: a sealed ledger
    # returns early, and an early return is where a nesting hid before.
    tiny.journal.seal()
    tiny.command_log.seal()
    tiny.ledger.seal()
    sweep(tiny)
    admissions(tiny, tiny_tap)
    tiny.command_log.note_iteration(9)
    tiny.command_log.note_command(9, None)

    _ = trace.capture().status
    _ = tiny.capture().status
    _ = summarise(trace, epoch=1)
    _ = summarise(trace, epoch=2)
    _ = summarise(tiny, epoch=1)
    list(trace.capture().rows)
    rows = trace.journal.drain()
    tiny.journal.drain()
    _ = trace.capture().status

    # The branches this is meant to reach were actually reached.
    assert tiny.capture().status.overflowed, "the overflow branch never ran"
    assert not trace.capture().status.overflowed, (
        "the sweep buffer overflowed, so its rows no longer prove coverage"
    )
    assert trace.capture().status.first_violation is not None, "no violation branch ran"
    assert trace.capture().status.payload_unreadable, "the unreadable-payload branch never ran"
    assert tiny.capture().status.payload_unreadable, "overflow lost the payload latch"
    assert tiny.capture().status.refused > 0, "the journal never refused anything"
    # Two notes an epoch from the sealed admissions, and the two after them.
    assert tiny.command_log.capture().refused == 6, (
        "the command ledger did not refuse the six notes made after its seal"
    )
    # The ATTITUDE ledger's branches, each shown to have run.
    assert tiny.ledger.capture().dropped > 0, "the ledger never overflowed"
    assert tiny.ledger.capture().failed, "the ledger's fault branch never ran"
    # Three rulings an epoch reached the sealed ledger.
    assert tiny.ledger.capture().refused == 6, (
        "the ledger did not refuse the six rulings made after its seal"
    )
    ledger = trace.ledger.capture()
    assert (ledger.stampless, ledger.rejections) == (
        2, {REJECT_STALE_BOOT: 2}
    ), "the tap's rulings did not all reach the ledger"
    assert trace.capture().status.callback_faults == 2, (
        "the guard never counted the raising callback"
    )
    assert len(trace.capture().commands.passes) == 2, "no pass was sampled"

    # And the sweep really covered the vocabulary. Asserted against the rows
    # rather than against the loop that wrote them, because a recorder that
    # swallowed an outcome would otherwise be counted as driven. The row budget
    # is deliberately small, so this reads the rows the trace SAW, which
    # ``drain`` returns in full only up to capacity -- hence the sweep is
    # checked per event against what that event actually emitted.
    # Asserted per EVENT, OUTCOME and EPOCH together. Aggregating them was a
    # real gap and a review measured it three ways: suppressing all epoch-2
    # truth rows passed, suppressing all tiny-trace truth rows passed, and
    # suppressing the sweep's own committed associations passed because setup
    # rows covered for them. A triple that is missing is now named.
    seen = {(row[0], row[2], row[1]) for row in rows}
    for event, prefix in (
        (EVENT_ASSOCIATION, "ASSOCIATION_"),
        (EVENT_TRUTH, "TRUTH_"),
        (EVENT_STAGE, "STAGE_"),
        (EVENT_OUTPUT, "OUTPUT_"),
        (EVENT_DECIMATE, "DISCARD_"),
        (EVENT_LIFECYCLE, "LIFECYCLE_"),
        (EVENT_SUBSCRIPTION, "SUBSCRIPTION_"),
    ):
        for outcome in _declared_outcomes(prefix):
            for epoch in (1, 2):
                assert (event, outcome, epoch) in seen, (
                    f"never driven: {event}/{outcome} in epoch {epoch}"
                )

    # Every watcher must have been REACHED, or "no overlap" is the answer an
    # uninstrumented lock gives. This is the control the test lacked when the
    # row lock moved out from under it.
    assert taken == {"row", "command", "ledger", "tap"}, (
        f"a watched lock was never acquired, so it watched nothing: {taken}"
    )
    assert overlaps == [], f"both locks were held at once: {overlaps}"

    # And the watcher itself has to be capable of firing, or this proves
    # nothing either. Two controls, because they fail for different reasons.
    control: set[str] = set()
    row, command = (
        _WatchedLock("row", held, overlaps, control),
        _WatchedLock("command", held, overlaps, control),
    )
    with row, command:
        pass
    assert overlaps == [("command", "row")]


class _AccessRecord:
    """Every touch of the journal's protected state, and whether it was locked."""

    def __init__(self) -> None:
        self.touches: list[tuple[str, bool]] = []
        self.armed = False

    def note(self, what: str, locked: bool) -> None:
        if self.armed:
            self.touches.append((what, locked))

    @property
    def unlocked(self) -> list[str]:
        return [what for what, locked in self.touches if not locked]


class _GuardedLog:
    """The row log, reporting whether the lock was held at each touch.

    Instrumenting the LOG rather than the lock is the point. A counter on the
    lock records that a member acquired it SOMETIME during the call, and a
    review showed that is not the property: replacing each critical section with
    ``with self._lock: pass`` followed by the original body left every member
    acquiring the lock, doing its work entirely unsynchronised, and passing.

    Two things a later review found and this now handles.

    Writes are FORWARDED. Without ``__setattr__`` the wrapper swallowed
    ``log._failed = True`` into itself, leaving the real log unfailed -- a probe
    that changes the state it is measuring is worse than no probe.

    And reading a non-callable attribute off the log is reported too. That is a
    POLICY, not an observed locking defect, and the message says so: the journal
    is required to only ever CALL this object. The policy exists because a member
    that instead takes ``self._log._rows`` under the lock and copies it
    afterwards has let mutable state escape the critical section, and no
    per-access check downstream can see that. Reporting every read is a blunt
    way to close it -- a review showed a locked read of the immutable
    ``capacity`` is reported the same way -- and it is kept blunt deliberately,
    because distinguishing safe reads from unsafe ones means tracking what
    happens to the value afterwards, which is the kind of approximation this
    workstream has already had to abandon twice.
    """

    def __init__(self, real: object, lock: object, record: _AccessRecord):
        object.__setattr__(self, "_real", real)
        object.__setattr__(self, "_lock", lock)
        object.__setattr__(self, "_record", record)

    def __getattr__(self, name: str) -> object:
        real = object.__getattribute__(self, "_real")
        lock = object.__getattribute__(self, "_lock")
        record = object.__getattribute__(self, "_record")
        attribute = getattr(real, name)
        if not callable(attribute):
            record.note(f"_log.{name} (read, not called)", False)
            return attribute

        def wrapped(*args: object, **kwargs: object) -> object:
            record.note(f"_log.{name}()", lock.locked())
            return attribute(*args, **kwargs)

        return wrapped

    def __setattr__(self, name: str, value: object) -> None:
        real = object.__getattribute__(self, "_real")
        lock = object.__getattribute__(self, "_lock")
        record = object.__getattribute__(self, "_record")
        record.note(f"_log.{name} = ...", lock.locked())
        setattr(real, name, value)


class _GuardedState:
    """The journal's seal state, reporting whether the lock was held at each
    read and write.

    Unlike the log's, its members ARE plain attributes, so a read under the
    lock is what is required rather than a breach of policy. Instrumented
    because ``seal`` touches nothing else, and a member that touched nothing
    would pass this test vacuously.
    """

    def __init__(self, real: object, lock: object, record: _AccessRecord):
        object.__setattr__(self, "_real", real)
        object.__setattr__(self, "_lock", lock)
        object.__setattr__(self, "_record", record)

    def __getattr__(self, name: str) -> object:
        locked = object.__getattribute__(self, "_lock").locked()
        object.__getattribute__(self, "_record").note(
            f"_seal.{name} read", locked
        )
        return getattr(object.__getattribute__(self, "_real"), name)

    def __setattr__(self, name: str, value: object) -> None:
        locked = object.__getattribute__(self, "_lock").locked()
        object.__getattribute__(self, "_record").note(
            f"_seal.{name} = ...", locked
        )
        setattr(object.__getattribute__(self, "_real"), name, value)


def _watched_journal(record: _AccessRecord) -> type:
    """A RowJournal whose watermark reads and writes are observed too.

    ``watermark_us`` is the one member that touches no log, so the guard above
    would have nothing to say about it. The watermark is a plain slot, so a
    subclass property shadows the slot descriptor and reports each access while
    still storing through it.
    """

    class Watched(RowJournal):
        __slots__ = ()

        @property
        def _watermark_us(self) -> int | None:
            record.note("_watermark_us read", self._lock.locked())
            return RowJournal._watermark_us.__get__(self)

        @_watermark_us.setter
        def _watermark_us(self, value: int | None) -> None:
            record.note("_watermark_us write", self._lock.locked())
            RowJournal._watermark_us.__set__(self, value)

    return Watched


class _Unconvertible:
    """An epoch that explodes on int(), to drive each recorder's fault path."""

    def __index__(self) -> int:
        raise RuntimeError("this epoch cannot be converted")

    __int__ = __index__


_UNCONVERTIBLE = _Unconvertible()


def test_every_journal_member_holds_the_row_lock_while_it_works() -> None:
    """Each member INDIVIDUALLY, and HELD rather than merely taken.

    Two review findings live in this test. The first: an aggregate check asked
    only whether RowJournal locks somewhere, so deleting the ``with`` from
    ``record_output`` alone left all 98 focused tests passing with that row
    appended unsynchronised. The second: counting acquisitions is still not the
    property -- ``with self._lock: pass`` followed by the unguarded body
    acquires the lock ten times out of ten and protects nothing.

    So the protected state is instrumented instead of the lock, and every access
    to it must happen while the lock is held. A member that touched nothing is a
    failure too: it would pass an "everything was locked" assertion vacuously.

    The member list is derived from the class and asserted complete, so a new
    public member is either classified here or a failure -- the same tripwire
    the one-writer guard uses, and for the same reason.
    """
    frame = _frame(1.0)
    calls = {
        "record_association": dict(
            epoch=1, outcome=ASSOCIATION_COMMITTED, source_s=1.0
        ),
        "record_truth": dict(epoch=1, outcome=TRUTH_RECORDED),
        "record_stage": dict(epoch=1, outcome=STAGE_STAGED, frame=frame),
        "record_discard": dict(
            epoch=1, reason=DISCARD_UNBRACKETABLE, victim=frame
        ),
        "record_output": dict(
            epoch=1, outcome=OUTPUT_DISPATCHED, taken_at_us=1_000, iteration=1
        ),
        "record_lifecycle": dict(
            epoch=1, outcome=LIFECYCLE_CLOSED, resulting_epoch=2
        ),
        "record_subscription": dict(
            epoch=1, outcome=SUBSCRIPTION_OPENED, ruling=3
        ),
        "count_callback_fault": {},
        "capture": {},
        "fail": {},
        "seal": {},
        "drain": {},
        "watermark_us": {},
    }
    # Reads one int, set once in __init__ and never written again: the only
    # member with nothing to synchronise.
    lock_free = {"period_us"}

    # Public surface only: vars() also lists the dunders and the
    # ``__slots__`` field descriptors, neither of them a member to classify.
    public = {name for name in vars(RowJournal) if not name.startswith("_")}
    assert public == set(calls) | lock_free, (
        "RowJournal's public surface changed; classify the new member as "
        f"lock-taking or lock-free: {sorted(public ^ (set(calls) | lock_free))}"
    )

    for name in sorted(calls):
        record = _AccessRecord()
        journal = _watched_journal(record)(PERIOD_US, 8)
        journal._log = _GuardedLog(journal._log, journal._lock, record)
        journal._seal = _GuardedState(journal._seal, journal._lock, record)
        record.armed = True  # construction itself runs before any lock exists
        getattr(journal, name)(**calls[name])

        assert record.touches, (
            f"RowJournal.{name} touched neither the log nor the watermark, so "
            "this test says nothing about it"
        )
        assert not record.unlocked, (
            f"RowJournal.{name} either worked outside the row lock or read an "
            f"attribute off the log instead of calling it: {record.unlocked}"
        )

    # The FAULT path, which the table above cannot reach because every argument
    # in it converts cleanly. A review found the recorders latching the failure
    # with the lock already dropped -- the critical section releases it on the
    # way out of the ``with``, so ``except: self._log.fail()`` ran unguarded in
    # all five. An unreachable branch is where the last three of these have
    # been hiding, so it is driven rather than reasoned about.
    for name in sorted(calls):
        if not name.startswith("record_"):
            continue
        record = _AccessRecord()
        journal = _watched_journal(record)(PERIOD_US, 8)
        log = journal._log
        journal._log = _GuardedLog(log, journal._lock, record)
        journal._seal = _GuardedState(journal._seal, journal._lock, record)
        record.armed = True
        getattr(journal, name)(**{**calls[name], "epoch": _UNCONVERTIBLE})

        assert record.touches, f"RowJournal.{name} did not fault as intended"
        assert not record.unlocked, (
            f"RowJournal.{name} either touched the log outside the row lock "
            f"while handling a fault, or read an attribute off it instead of "
            f"calling it: {record.unlocked}"
        )
        assert journal.capture()[1].failed, (
            f"RowJournal.{name} swallowed a fault without latching it"
        )
        # The log latched it too, not only the journal's flag: the flag
        # alone satisfies the assertion above.
        assert log.status().failed, (
            f"RowJournal.{name} flagged a fault and never latched the log"
        )


def test_a_fault_while_latching_a_fault_still_invalidates_the_trace() -> None:
    """The double fault: recording failed, and recording the failure failed too.

    A review injected exactly this -- an epoch that raises on ``int()`` while
    ``BoundedRowLog.fail`` raises as well -- and measured "rows: 0 failed:
    False". The recorder had swallowed both, so a trace that had lost a row
    reported COMPLETE, which is the single outcome this whole instrument exists
    to prevent. Nothing may escape into the command path, but nothing may be
    silently forgiven either.

    Both halves are asserted here, because they pull in opposite directions and
    fixing one by breaking the other is the obvious wrong turn.
    """
    original = BoundedRowLog.fail

    def cannot_even_fail(self: object) -> None:
        raise MemoryError("no room even to record the failure")

    frame = _frame(1.0)
    faults = {
        "record_association": dict(outcome=ASSOCIATION_COMMITTED, source_s=1.0),
        "record_truth": dict(outcome=TRUTH_RECORDED),
        "record_stage": dict(outcome=STAGE_STAGED, frame=frame),
        "record_discard": dict(reason=DISCARD_UNBRACKETABLE, victim=frame),
        "record_output": dict(
            outcome=OUTPUT_DISPATCHED, taken_at_us=1_000, iteration=1
        ),
        "record_lifecycle": dict(outcome=LIFECYCLE_CLOSED, resulting_epoch=2),
        "record_subscription": dict(outcome=SUBSCRIPTION_OPENED, ruling=3),
    }
    # EVERY recorder, derived from the class. Driving one of them was not
    # enough: dropping the latch result in record_association alone passed a
    # version of this test that only drove record_truth, which is the
    # per-item gap behind an aggregate that this workstream keeps re-finding.
    recorders = {
        name for name in vars(RowJournal) if name.startswith("record_")
    }
    assert recorders == set(faults), (
        f"a recorder is not covered by this test: {recorders ^ set(faults)}"
    )

    for name in sorted(faults):
        journal = RowJournal(PERIOD_US, 8)
        BoundedRowLog.fail = cannot_even_fail
        try:
            # Must not raise: the caller is a command path.
            getattr(journal, name)(epoch=_UNCONVERTIBLE, **faults[name])
        finally:
            BoundedRowLog.fail = original

        assert list(journal.capture()[0]) == [], f"{name} recorded a row after all"
        assert journal.capture()[1].failed is True, (
            f"{name} lost a row and the trace still reads complete"
        )

    # And through the facade, which is where a real caller sits.
    trace = DeterminismTrace(PERIOD_US, capacity=8)
    BoundedRowLog.fail = cannot_even_fail
    try:
        trace.record_truth(epoch=_UNCONVERTIBLE, outcome=TRUTH_RECORDED)
    finally:
        BoundedRowLog.fail = original
    assert trace.capture().status.failed is True


def test_committed_association_advances_the_watermark() -> None:
    trace = DeterminismTrace(PERIOD_US, capacity=16)
    trace.record_association(
        epoch=1, outcome=ASSOCIATION_COMMITTED, source_s=12.5
    )
    row = list(trace.capture().rows)[0]
    assert row[0] == EVENT_ASSOCIATION
    assert row[2] == ASSOCIATION_COMMITTED
    assert row[3] == 12_500_000
    assert row[4] == 625
    # A refusal carries no stamp, so the watermark cannot advance on it.
    trace.record_association(epoch=1, outcome=ASSOCIATION_REFUSED)
    refusal = list(trace.capture().rows)[1]
    assert refusal[3] is None and refusal[4] is None
    trace.record_output(
        epoch=1, outcome=OUTPUT_EMPTY, taken_at_us=trace.watermark_us()
    )
    assert list(trace.capture().rows)[2][5] == 12_500_000


def test_regressing_and_repeating_source_stamps_latch_violations() -> None:
    trace = DeterminismTrace(PERIOD_US, capacity=16)
    trace.record_association(
        epoch=1, outcome=ASSOCIATION_COMMITTED, source_s=12.5
    )
    trace.record_association(
        epoch=1, outcome=ASSOCIATION_COMMITTED, source_s=12.4
    )
    trace.record_association(
        epoch=1, outcome=ASSOCIATION_COMMITTED, source_s=12.5
    )
    reasons = [
        row[2] for row in list(trace.capture().rows) if row[0] == EVENT_VIOLATION
    ]
    assert reasons == [VIOLATION_SOURCE_REGRESSED, VIOLATION_SOURCE_REPEATED]
    status = trace.capture().status
    # Only the FIRST is latched; the rest are recorded, not promoted.
    assert status.first_violation is not None
    assert status.first_violation[2] == VIOLATION_SOURCE_REGRESSED
    # A regression never drags the watermark backwards.
    trace.record_output(
        epoch=1, outcome=OUTPUT_EMPTY, taken_at_us=trace.watermark_us()
    )
    assert list(trace.capture().rows)[-1][5] == 12_500_000


def test_overflow_keeps_the_earliest_rows_and_invalidates_the_trace() -> None:
    trace = DeterminismTrace(PERIOD_US, capacity=2)
    for index in range(5):
        trace.record_association(
            epoch=1,
            outcome=ASSOCIATION_COMMITTED,
            source_s=12.5 + 0.02 * index,
        )
    rows = list(trace.capture().rows)
    assert len(rows) == 2
    # The scored leg starts at activation, so the EARLIEST rows are the ones
    # worth keeping when the budget runs out.
    assert [row[3] for row in rows] == [12_500_000, 12_520_000]
    status = trace.capture().status
    assert status.dropped == 3
    assert status.overflowed is True
    assert status.complete is False


def test_a_recorder_fault_latches_invalid_without_raising() -> None:
    trace = DeterminismTrace(PERIOD_US, capacity=16)
    # int("epoch") raises: the recorder must swallow it and mark itself
    # unusable, because its caller is a command path.
    trace.record_association(
        epoch="epoch",  # type: ignore[arg-type]
        outcome=ASSOCIATION_COMMITTED,
        source_s=12.5,
    )
    status = trace.capture().status
    assert status.failed is True
    assert status.complete is False


def test_zero_period_records_rows_without_inventing_slots() -> None:
    trace = DeterminismTrace(0, capacity=8)
    trace.record_association(
        epoch=1, outcome=ASSOCIATION_COMMITTED, source_s=12.5
    )
    row = list(trace.capture().rows)[0]
    assert row[3] == 12_500_000
    assert row[4] is None


def test_drain_empties_the_rows_and_snapshot_does_not() -> None:
    trace = DeterminismTrace(PERIOD_US, capacity=8)
    trace.record_output(epoch=1, outcome=OUTPUT_EMPTY, taken_at_us=None)
    assert len(list(trace.capture().rows)) == 1
    assert len(list(trace.capture().rows)) == 1
    assert len(trace.journal.drain()) == 1
    assert list(trace.capture().rows) == []


def test_summarise_measures_stage_and_dispatch_lag_in_slots() -> None:
    trace = DeterminismTrace(PERIOD_US, capacity=32)
    frame = _frame(12.5)
    trace.record_association(
        epoch=1, outcome=ASSOCIATION_COMMITTED, source_s=12.5
    )
    trace.record_stage(epoch=1, outcome=STAGE_STAGED, frame=frame)
    # The worker is on its seventh pass when it takes the frame.
    trace.command_log.note_iteration(7)
    # The input stream advances three slots before this frame is dispatched.
    trace.record_association(
        epoch=1, outcome=ASSOCIATION_COMMITTED, source_s=12.56
    )
    trace.record_discard(
        epoch=1, reason=DISCARD_PUBLISH_OVERWRITTEN, victim=frame
    )
    trace.record_output(
        epoch=1,
        outcome=OUTPUT_DISPATCHED,
        frame=frame,
        taken_at_us=trace.watermark_us(),
    )
    summary = summarise(trace)
    assert summary["period_us"] == PERIOD_US
    assert summary["complete"] is True
    assert summary["counts"][EVENT_STAGE] == {STAGE_STAGED: 1}
    assert summary["counts"][EVENT_DECIMATE] == {DISCARD_PUBLISH_OVERWRITTEN: 1}
    assert summary["counts"][EVENT_OUTPUT] == {OUTPUT_DISPATCHED: 1}
    # Staged in the slot it was captured in, taken three slots later.
    assert summary["stage_lag_slots"] == {"count": 1, "min": 0, "max": 0}
    assert summary["dispatch_lag_slots"] == {"count": 1, "min": 3, "max": 3}
    # The worker pass that first had it. An ITERATION, in its own unit: it is
    # never combined with the slot numbers above, because the worker skips a
    # missed deadline and the two grids drift apart the first time it does.
    assert summary["ready_iterations"] == {"count": 1, "min": 7, "max": 7}
    assert summary["commands"]["iterations"] == 1
    assert summary["commands"]["entries"] == 0


def test_summarise_reports_no_lag_when_nothing_was_delivered() -> None:
    trace = DeterminismTrace(PERIOD_US, capacity=8)
    trace.record_output(epoch=1, outcome=OUTPUT_EMPTY, taken_at_us=None)
    summary = summarise(trace)
    empty = {"count": 0, "min": None, "max": None}
    assert summary["dispatch_lag_slots"] == empty
    assert summary["stage_lag_slots"] == empty
    # An empty slot was still handed to a worker pass, but no frame was
    # READY on one, so the iteration series stays empty too.
    assert summary["ready_iterations"] == empty


def test_a_scoped_violation_survives_the_drain_that_ends_the_leg() -> None:
    """The owner drains after the leg. The verdict must outlive the rows.

    Derived from the rows, a leg-scoped violation vanished the moment the
    owner drained, and the leg summarised as clean AND complete -- the worst
    possible failure for a determinism verdict.
    """
    trace = DeterminismTrace(PERIOD_US, capacity=32)
    trace.record_association(
        epoch=1, outcome=ASSOCIATION_COMMITTED, source_s=99.0
    )
    trace.record_association(
        epoch=1, outcome=ASSOCIATION_COMMITTED, source_s=98.0
    )
    before = summarise(trace, epoch=1)
    assert before["first_violation"][2] == VIOLATION_SOURCE_REGRESSED

    assert trace.journal.drain()
    after = summarise(trace, epoch=1)
    assert after["rows"] == 0
    assert after["first_violation"][2] == VIOLATION_SOURCE_REGRESSED
    assert summarise(trace)["first_violation"] is not None


def test_a_cruise_leg_violation_does_not_invalidate_the_scored_leg() -> None:
    """The latch is trace-wide; the verdict is per leg.

    activate() bumps the epoch, so a stamp that regressed before the scored
    leg opened would otherwise decide that leg's verdict.
    """
    trace = DeterminismTrace(PERIOD_US, capacity=32)
    trace.record_association(
        epoch=0, outcome=ASSOCIATION_COMMITTED, source_s=99.0
    )
    trace.record_association(
        epoch=0, outcome=ASSOCIATION_COMMITTED, source_s=98.0
    )
    trace.record_association(
        epoch=1, outcome=ASSOCIATION_COMMITTED, source_s=100.0
    )
    assert trace.capture().status.first_violation is not None

    scored = summarise(trace, epoch=1)
    assert scored["first_violation"] is None
    assert scored["counts"][EVENT_ASSOCIATION] == {ASSOCIATION_COMMITTED: 1}
    assert scored["rows"] == 1
    assert scored["rows_recorded"] == 4

    cruise = summarise(trace, epoch=0)
    assert cruise["first_violation"][2] == VIOLATION_SOURCE_REGRESSED

    # Unscoped still answers for the whole trace.
    every = summarise(trace)
    assert every["first_violation"][2] == VIOLATION_SOURCE_REGRESSED
    assert every["counts"][EVENT_ASSOCIATION] == {ASSOCIATION_COMMITTED: 3}


def test_build_trace_follows_the_module_gate(monkeypatch) -> None:
    monkeypatch.setattr(trace_module, "ENABLED", False)
    assert build_trace(PERIOD_US) is None
    monkeypatch.setattr(trace_module, "ENABLED", True)
    built = build_trace(PERIOD_US)
    assert isinstance(built, DeterminismTrace)
    assert built.period_us == PERIOD_US


def test_environment_gate_and_capacity_parsing() -> None:
    assert gate.enabled_from_env({}) is False
    assert gate.enabled_from_env({DETERMINISM_TRACE_ENV: "0"}) is False
    assert (
        gate.enabled_from_env({DETERMINISM_TRACE_ENV: "off"}) is False
    )
    assert gate.enabled_from_env({DETERMINISM_TRACE_ENV: "1"}) is True
    assert gate.capacity_from_env({}) == DEFAULT_TRACE_CAPACITY
    assert (
        gate.capacity_from_env({DETERMINISM_TRACE_CAPACITY_ENV: "40"})
        == 40
    )
    # A nonsense or non-positive budget falls back rather than disabling the
    # bound, which would make the trace unbounded.
    assert (
        gate.capacity_from_env({DETERMINISM_TRACE_CAPACITY_ENV: "x"})
        == DEFAULT_TRACE_CAPACITY
    )
    assert (
        gate.capacity_from_env({DETERMINISM_TRACE_CAPACITY_ENV: "0"})
        == DEFAULT_TRACE_CAPACITY
    )


def test_tracing_is_off_by_default() -> None:
    assert trace_module.ENABLED is False


def test_a_drained_trace_does_not_report_itself_complete() -> None:
    """A verdict has to be taken from a trace that still holds its rows.

    Draining is the intended end-of-leg step, so this is not a fault -- but
    after one, `rows` is 0 and every count is empty, and a reader who trusted
    `complete` would read that as a clean run with nothing in it.
    """
    trace = DeterminismTrace(PERIOD_US, capacity=8)
    trace.record_association(
        epoch=1, outcome=ASSOCIATION_COMMITTED, source_s=12.5
    )
    assert summarise(trace, epoch=1)["complete"] is True

    trace.journal.drain()
    after = summarise(trace, epoch=1)
    assert after["rows"] == 0
    assert after["counts"][EVENT_ASSOCIATION] == {}
    assert after["drained"] is True
    assert after["complete"] is False


def test_a_command_ledger_that_overflowed_invalidates_the_verdict() -> None:
    """A command sequence with a hole cannot show two runs issued the same."""
    trace = DeterminismTrace(PERIOD_US, capacity=1)
    log = trace.command_log
    log.note_command(1, None)
    log.note_command(2, None)

    assert log.dropped == 1
    summary = summarise(trace)
    assert summary["commands"]["dropped"] == 1
    assert summary["complete"] is False


def test_a_command_ledger_that_faulted_invalidates_the_verdict() -> None:
    trace = DeterminismTrace(PERIOD_US, capacity=8)
    trace.command_log._seal.faulted = True

    assert summarise(trace)["complete"] is False


def test_a_command_whose_bytes_were_never_read_invalidates_the_verdict() -> (
    None
):
    """An unreadable entry is a HOLE, and holes are what invalidate a run.

    The marker is non-None, so both the completeness verdict and the summary
    count treated it as a digested command and reported bytes that were never
    obtained. Two runs cannot be compared on a command neither of them
    recorded.
    """
    trace = DeterminismTrace(PERIOD_US, capacity=8)
    trace.command_log.note_command(1, SimpleNamespace(yaw="not a number"))

    summary = summarise(trace)
    assert summary["commands"]["unreadable"] == 1
    assert summary["commands"]["digested"] == 0
    assert summary["complete"] is False


def test_the_comparable_command_outcomes_do_not_invalidate_the_verdict() -> (
    None
):
    """The counter must not be so broad that an honest run reads as holed.

    "Produced nothing" and "crashed" are both facts two runs can be compared
    on, so neither is a hole. Only an entry whose bytes were never obtained is.
    """
    trace = DeterminismTrace(PERIOD_US, capacity=8)
    trace.command_log.note_command(1, None)
    trace.command_log.note_command(2, None, True)

    summary = summarise(trace)
    assert summary["commands"]["no_command"] == 1
    assert summary["commands"]["raised"] == 1
    assert summary["commands"]["digested"] == 0
    assert summary["commands"]["unreadable"] == 0
    assert summary["complete"] is True


def test_digested_counts_only_entries_that_carry_real_command_bytes() -> None:
    """One number, one meaning. Both markers are non-None; neither is bytes."""
    trace = DeterminismTrace(PERIOD_US, capacity=8)
    trace.command_log.note_command(1, CalcData(1.0, 2.0, 3.0, None, 0.5))
    trace.command_log.note_command(2, None)
    trace.command_log.note_command(3, None, True)
    trace.command_log.note_command(4, SimpleNamespace(yaw="not a number"))

    commands = summarise(trace)["commands"]
    assert commands["entries"] == 4
    assert commands["digested"] == 1
    assert commands["no_command"] == 1
    assert commands["raised"] == 1
    assert commands["unreadable"] == 1


def test_the_row_ledger_and_the_command_ledger_are_one_verdict() -> None:
    """Neither ledger can report a clean run while the other has a hole."""
    trace = DeterminismTrace(PERIOD_US, capacity=8)

    assert trace.capture().status.complete is True
    assert trace.capture().status.commands_incomplete is False
    assert trace.capture().status.drained is False


def test_a_truth_row_carries_the_watermark_it_was_recorded_under() -> None:
    """A TRUTH row keeps the newest ATTITUDE stamp known when its sample
    arrived, and that stamp's slot: the lower bound an offline label is
    read from. A mutation that dropped both survived every test before."""
    from navpy.modules.vision.sim.determinism_truth_sample import UNAVAILABLE
    journal = RowJournal(PERIOD_US, 8)
    journal.record_truth(epoch=1, outcome=TRUTH_UNREADABLE)
    journal.record_association(
        epoch=1, outcome=ASSOCIATION_COMMITTED, source_s=1.25
    )
    journal.record_truth(epoch=1, outcome=TRUTH_RECORDED)

    rows = list(journal.capture()[0])
    assert [row for row in rows if row[0] == EVENT_TRUTH] == [
        (EVENT_TRUTH, 1, TRUTH_UNREADABLE, None, None, UNAVAILABLE),
        (EVENT_TRUTH, 1, TRUTH_RECORDED, 1_250_000, 62, UNAVAILABLE),
    ]


def test_a_lifecycle_row_names_both_epochs_and_the_watermark_it_saw() -> None:
    """D4: one row per boundary, at the ENDING epoch, with the epoch the
    boundary made current and the newest ATTITUDE stamp known at that moment,
    and its slot: the lower bound an offline label is read from."""
    journal = RowJournal(PERIOD_US, 8)
    journal.record_lifecycle(
        epoch=0, outcome=LIFECYCLE_ACTIVATED, resulting_epoch=1
    )
    journal.record_association(
        epoch=1, outcome=ASSOCIATION_COMMITTED, source_s=1.25
    )
    journal.record_lifecycle(
        epoch=1, outcome=LIFECYCLE_CLOSED, resulting_epoch=2
    )

    rows = list(journal.capture()[0])
    assert [row for row in rows if row[0] == EVENT_LIFECYCLE] == [
        (EVENT_LIFECYCLE, 0, LIFECYCLE_ACTIVATED, 1, None, None),
        (EVENT_LIFECYCLE, 1, LIFECYCLE_CLOSED, 2, 1_250_000, 62),
    ]


def test_a_callback_fault_is_a_hole_the_status_reports() -> None:
    """D3: the callback registry swallows what the source's callback raised,
    so this count is the only mark a half-recorded message leaves, and it
    invalidates a verdict like any other hole. It is not a recorder fault."""
    log = BoundedRowLog(8)
    assert log.status().complete
    log.note_callback_fault()
    status = log.status()
    assert (status.callback_faults, status.complete, status.failed) == (
        1, False, False
    )

    trace = DeterminismTrace(PERIOD_US, capacity=8)
    trace.journal.count_callback_fault()
    summary = summarise(trace)
    assert (summary["callback_faults"], summary["complete"]) == (1, False)


def test_a_hole_in_the_attitude_ledger_invalidates_the_capture() -> None:
    """The rows and the commands can be whole while the ledger is not: an
    entry dropped for budget, or a ruling missing between two it holds, and
    the window (D8.9) can no longer be checked. ``TraceStatus`` has no room
    for it, so it folds into ``TraceCapture.complete``."""

    def ruling(number: int) -> AdmissionRuling:
        return AdmissionRuling("ATTITUDE", number, 1_000 * number, True, None)

    whole = DeterminismTrace(PERIOD_US, capacity=8)
    whole.ledger.append(ruling(1))
    whole.ledger.append(ruling(2))
    assert whole.capture().complete
    assert summarise(whole)["complete"] is True

    overflowed = DeterminismTrace(PERIOD_US, capacity=1)
    overflowed.ledger.append(ruling(1))
    overflowed.ledger.append(ruling(2))
    gapped = DeterminismTrace(PERIOD_US, capacity=8)
    gapped.ledger.append(ruling(1))
    gapped.ledger.append(ruling(3))
    for name, holed in (("overflowed", overflowed), ("gapped", gapped)):
        capture = holed.capture()
        assert capture.status.complete, f"{name}: the rows had a hole"
        assert capture.complete is False, f"{name} read as complete"
        assert summarise(holed)["complete"] is False, name
