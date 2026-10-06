"""D3: the association rows correlate with the ATTITUDE ledger, end to end.

Through the REAL router, message store and callback registry, on the bus's
one reader thread: every ATTITUDE ruling ADMITTED strictly between the
OPENED row's ruling (s0) and the CLOSED row's ruling (s1) reached the
source's callback, so it has exactly one association row naming it, whatever
the outcome. s0 and s1 themselves are not claimed: each races the
subscription it bounds, and rows may name rulings outside the window. A
ruling inside it without its row is a hole (D8.9 of
the LANDING2 step-1 plan), and the ledger is
what finds it when neither the rows nor the status can.
"""

from __future__ import annotations

import itertools
import threading
from collections import Counter
from collections.abc import Callable
from dataclasses import replace
from types import SimpleNamespace
from typing import Any

import pytest

from navpy.modules.vehicle import inbound_router
from navpy.modules.vehicle.inbound_router import REJECT_STALE_BOOT
from navpy.modules.vision.sim import determinism_trace
from navpy.modules.vision.sim.determinism_eligibility import (
    INSPECTABLE,
    Verdict,
    judge,
)
from navpy.modules.vision.sim.determinism_events import (
    ASSOCIATION_COMMITTED,
    ASSOCIATION_REFUSED,
    EVENT_ASSOCIATION,
    EVENT_SUBSCRIPTION,
    SUBSCRIPTION_CLOSED,
    SUBSCRIPTION_OPENED,
)
from navpy.modules.vision.sim.determinism_evidence import sealed_capture
from navpy.modules.vision.sim.determinism_labels import case_labels
from navpy.modules.vision.sim.determinism_row_log import BoundedRowLog
from navpy.modules.vision.sim.direct_poi_pixel_source import (
    DirectPoiPixelSource,
)
from tests.modules.vision import determinism_case_factory as factory
from tests.modules.vision.direct_pixel_router_bench import (
    POI,
    Message,
    RouterVehicle,
    attitude_message,
    sim_state_message,
)

# The pairing gate's skew bound is not under test here: wide enough that no
# scheduling stall between two ingests can refuse a pair.
GENEROUS_SKEW_S = 60.0
# Bounds a FAILING run only: every wait below is released by the test itself.
WAIT_S = 10.0


def _stamp(ruling: int) -> int:
    """The boot stamp of the ATTITUDE ruled ``ruling``. Every ATTITUDE is
    ruled on and numbered in turn, so the tap's number and this agree."""
    return 10_000 + 20 * ruling


def _source(
    monkeypatch: pytest.MonkeyPatch,
    vehicle: RouterVehicle,
    deliver: Callable[[Callable[[Any], None]], Callable[[Any], None]] = (
        lambda append: append
    ),
) -> tuple[DirectPoiPixelSource, Any]:
    """A traced source, and the ledger's subscription made as the child
    makes it: BEFORE the source starts (D8.5). ``deliver`` stands between
    the tap and the ledger's ``append``, to lose or alter one delivery."""
    monkeypatch.setattr(determinism_trace, "ENABLED", True)
    source = DirectPoiPixelSource(
        vehicle,
        POI,
        SimpleNamespace(
            wall_period_for_scheduler_period=lambda _: GENEROUS_SKEW_S
        ),
        aircraft_sequence="ZYX",
        aircraft_degrees=True,
        deliver=lambda detection: True,
        wall_now_s=lambda: 100.0,
    )
    ledger = source.determinism_trace.ledger
    return source, vehicle.on_admission("ATTITUDE", deliver(ledger.append))


def _window(capture: Any) -> tuple[int | None, int | None, list[int]]:
    """s0 and s1, the rulings the OPENED and CLOSED rows name, and the
    ADMITTED rulings strictly between them: the ones the claim covers."""
    bounds = [row for row in capture.rows if row[0] == EVENT_SUBSCRIPTION]
    assert [row[2] for row in bounds] == [
        SUBSCRIPTION_OPENED, SUBSCRIPTION_CLOSED
    ]
    s0, s1 = bounds[0][3], bounds[1][3]
    if s1 is None:
        return s0, s1, []
    low = 0 if s0 is None else s0
    return s0, s1, [
        ruling
        for ruling, _, accepted, _ in capture.ledger.entries
        if accepted and low < ruling < s1
    ]


def _unmatched(capture: Any) -> dict[int, int]:
    """Each ruling the claim covers whose association rows are not exactly
    one, with how many it has."""
    named = Counter(
        row[5] for row in capture.rows if row[0] == EVENT_ASSOCIATION
    )
    _, _, window = _window(capture)
    return {ruling: named[ruling] for ruling in window if named[ruling] != 1}


def _escaped(call: Callable[[], object]) -> BaseException | None:
    """What ``call`` raised. Caught as BaseException rather than with
    ``pytest.raises``: an interrupt that escaped would end the whole pytest
    session instead of failing one test."""
    try:
        call()
    except BaseException as escaped:  # noqa: BLE001 - the caller asserts
        return escaped
    return None


def _driven_receipts(monkeypatch: pytest.MonkeyPatch) -> None:
    """Every receipt the router stamps, 5 ms after the one before, so the
    pairing's newness test turns on the schedule alone, never on how fast
    the host ingests."""
    receipts = itertools.count(100.0, 0.005)
    monkeypatch.setattr(
        inbound_router, "time", SimpleNamespace(time=lambda: next(receipts))
    )


def _verdict(
    source: DirectPoiPixelSource, subscription: Any, ending: int | None
) -> Verdict:
    """D8's verdict on the case this leg leaves: the capture sealed as the
    writer seals it, the subscription's own figures, and the epoch close()
    returned, as the teardown reports them."""
    return judge(factory.case_of(
        sealed_capture(source.determinism_trace),
        admission=factory.admission(
            first=subscription.first,
            end=subscription.end,
            faults=subscription.faults,
            faulted=subscription.faulted,
        ),
        epoch=ending,
    ))


def _reasons(verdict: Verdict, rule: int) -> list[str]:
    return [reason.text for reason in verdict.reasons if reason.rule == rule]


def test_the_rows_name_the_rulings_the_source_was_subscribed_for(
    monkeypatch,
) -> None:
    """In order, on one thread: a ruling before start() has no row; the
    OPENED row names the latest ruling observed; each admission after it
    has one row naming it, whatever its outcome, and a rejection none; the
    CLOSED row names the latest observed; nothing after close() is rowed."""
    vehicle = RouterVehicle()
    source, subscription = _source(monkeypatch, vehicle)

    vehicle.ingest(attitude_message(10_000))  # 1: before the source subscribed
    source.start()
    vehicle.ingest(sim_state_message())
    vehicle.ingest(attitude_message(10_020))  # 2: pairs with that truth
    vehicle.ingest(attitude_message(10_010))  # 3: stale, so rejected
    vehicle.ingest(attitude_message(10_040))  # 4: no new truth, so refused
    ending = source.close()
    vehicle.ingest(attitude_message(10_060))  # 5: after the close
    subscription.cancel()

    capture = source.determinism_trace.capture()
    assert [
        (row[2], row[5]) for row in capture.rows
        if row[0] == EVENT_ASSOCIATION
    ] == [(ASSOCIATION_COMMITTED, 2), (ASSOCIATION_REFUSED, 4)]
    assert [row for row in capture.rows if row[0] == EVENT_SUBSCRIPTION] == [
        (EVENT_SUBSCRIPTION, 0, SUBSCRIPTION_OPENED, 1),
        (EVENT_SUBSCRIPTION, 0, SUBSCRIPTION_CLOSED, 4),
    ]
    assert capture.ledger.entries == (
        (1, 10_000, True, None),
        (2, 10_020, True, None),
        (3, 10_010, False, REJECT_STALE_BOOT),
        (4, 10_040, True, None),
        (5, 10_060, True, None),
    )
    assert (
        subscription.first, subscription.end,
        subscription.faults, subscription.faulted,
    ) == (1, 5, 0, False)
    # s1 is not claimed, though here it has its row.
    assert _window(capture) == (1, 4, [2])
    assert _unmatched(capture) == {}
    assert capture.complete
    verdict = _verdict(source, subscription, ending)
    # These legacy fixtures have no source-stamped truth: v2 adds rule 14.
    assert verdict.status == INSPECTABLE
    assert verdict.failed_rules == (14,), verdict.reasons


# How many admitted rulings the window holds, and which one is lost.
WINDOWS = {"first": (3, 2), "last": (3, 4), "only": (1, 2)}


@pytest.mark.parametrize("lost", sorted(WINDOWS))
def test_a_source_callback_that_raises_leaves_a_counted_hole(
    monkeypatch, lost
) -> None:
    """The attitude read raises inside the source's callback: that ruling
    has no row, the guard counted it, and the registry logged the SAME
    object it would have without the guard. First, last or only, the
    window names exactly the lost ruling."""
    inside, failed = WINDOWS[lost]
    vehicle = RouterVehicle()
    source, subscription = _source(monkeypatch, vehicle)
    failure = RuntimeError(f"attitude unreadable at ruling {failed}")
    vehicle.failures[_stamp(failed)] = failure

    vehicle.ingest(attitude_message(_stamp(1)))
    source.start()
    for ruling in range(2, inside + 3):  # the window, then s1
        vehicle.ingest(attitude_message(_stamp(ruling)))
    ending = source.close()
    subscription.cancel()

    capture = source.determinism_trace.capture()
    assert _window(capture) == (1, inside + 2, list(range(2, inside + 2)))
    assert _unmatched(capture) == {failed: 0}
    assert len(vehicle.logger.handling) == 1
    assert vehicle.logger.handling[0] is failure
    assert (capture.status.callback_faults, capture.status.failed) == (
        1, False
    )
    assert capture.complete is False
    # Ineligible twice over: the fault counted (D8.4), the hole (D8.9).
    assert _verdict(source, subscription, ending).failed_rules == (4, 9, 14)


def test_a_row_lost_before_the_source_was_called_is_seen_only_by_the_window(
    monkeypatch,
) -> None:
    """The registry's gap. An EARLIER callback raises, and the logger raises
    reporting it, so the dispatch stops before the source's callback: the
    source never ran, the guard counted nothing, and the rows and the status
    both read whole. Only the ledger shows the ruling the source was owed."""
    vehicle = RouterVehicle()
    source, subscription = _source(monkeypatch, vehicle)
    earlier = RuntimeError("an earlier subscriber failed")
    logging_failed = RuntimeError("the bus logger failed")

    def earlier_callback(message: Message) -> None:
        if message.time_boot_ms == _stamp(3):
            raise earlier

    vehicle.on_message("ATTITUDE", earlier_callback)  # ahead of the source
    vehicle.ingest(attitude_message(_stamp(1)))
    source.start()
    vehicle.ingest(attitude_message(_stamp(2)))
    vehicle.logger.failure = logging_failed
    escaped = _escaped(lambda: vehicle.ingest(attitude_message(_stamp(3))))
    vehicle.logger.failure = None
    vehicle.ingest(attitude_message(_stamp(4)))
    vehicle.ingest(attitude_message(_stamp(5)))
    ending = source.close()
    subscription.cancel()

    assert escaped is logging_failed
    assert escaped.__context__ is earlier
    capture = source.determinism_trace.capture()
    assert _window(capture) == (1, 5, [2, 3, 4])
    assert _unmatched(capture) == {3: 0}
    assert capture.status.callback_faults == 0
    assert capture.complete, "the rows and the status saw no loss at all"
    # So the case is ineligible through the missing row alone (D8.9).
    assert _verdict(source, subscription, ending).failed_rules == (9, 14)


def test_a_fault_count_that_fails_flags_the_journal_and_raises_the_same(
    monkeypatch,
) -> None:
    """The guard's count cannot be made: the journal is failed instead, and
    what the callback raised still reaches the registry, the same object."""
    vehicle = RouterVehicle()
    source, subscription = _source(monkeypatch, vehicle)
    failure = RuntimeError("attitude unreadable")
    vehicle.failures[_stamp(2)] = failure

    def count_fails(self: BoundedRowLog) -> None:
        raise MemoryError("the fault count failed")

    monkeypatch.setattr(BoundedRowLog, "note_callback_fault", count_fails)
    vehicle.ingest(attitude_message(_stamp(1)))
    source.start()
    vehicle.ingest(attitude_message(_stamp(2)))
    vehicle.ingest(attitude_message(_stamp(3)))
    ending = source.close()
    subscription.cancel()

    capture = source.determinism_trace.capture()
    assert len(vehicle.logger.handling) == 1
    assert vehicle.logger.handling[0] is failure
    assert (capture.status.callback_faults, capture.status.failed) == (
        0, True
    )
    assert capture.complete is False
    assert _unmatched(capture) == {2: 0}
    assert _verdict(source, subscription, ending).failed_rules == (4, 9, 14)


@pytest.mark.parametrize("original", ["exception", "interrupt"])
def test_an_interrupt_in_the_fault_count_is_passed_on_as_itself(
    monkeypatch, original
) -> None:
    """The count is a recording, and D5 passes on an interrupt that lands in
    one as itself, with the journal failed. So it escapes here, with what the
    callback raised as its context. Raising the callback's exception instead
    would swallow the interrupt, which the untraced path never does: the
    registry catches Exception alone. The window still shows the lost row."""
    vehicle = RouterVehicle()
    source, subscription = _source(monkeypatch, vehicle)
    failure: BaseException = (
        RuntimeError("attitude unreadable")
        if original == "exception"
        else SystemExit("attitude read stopped")
    )
    vehicle.failures[_stamp(2)] = failure
    interrupt = KeyboardInterrupt("ctrl+c while the fault is counted")

    def count_interrupted(self: BoundedRowLog) -> None:
        raise interrupt

    monkeypatch.setattr(
        BoundedRowLog, "note_callback_fault", count_interrupted
    )
    vehicle.ingest(attitude_message(_stamp(1)))
    source.start()
    escaped = _escaped(lambda: vehicle.ingest(attitude_message(_stamp(2))))
    vehicle.ingest(attitude_message(_stamp(3)))
    ending = source.close()
    subscription.cancel()

    assert escaped is interrupt
    assert escaped.__context__ is failure
    assert vehicle.logger.handling == [], "the registry saw no exception"
    capture = source.determinism_trace.capture()
    assert (capture.status.callback_faults, capture.status.failed) == (
        0, True
    )
    assert capture.complete is False
    assert _unmatched(capture) == {2: 0}
    assert _verdict(source, subscription, ending).failed_rules == (4, 9, 14)


class _Feeder:
    """The bus's ONE reader thread: SIM_STATE then ATTITUDE each cycle, every
    fifth ATTITUDE stale. It runs free up to what the test has granted, so
    start() and close() land wherever the thread happens to be."""

    def __init__(self, vehicle: RouterVehicle) -> None:
        self._vehicle = vehicle
        self._condition = threading.Condition()
        self._granted = 0
        self._done = 0
        self._stopped = False
        self.error: BaseException | None = None
        self._thread = threading.Thread(
            target=self._run, name="bus-reader", daemon=True
        )

    def start(self) -> None:
        self._thread.start()

    def grant(self, cycles: int) -> None:
        with self._condition:
            self._granted += cycles
            self._condition.notify_all()

    def wait_done(self, cycles: int) -> None:
        with self._condition:
            reached = self._condition.wait_for(
                lambda: self._done >= cycles or self.error is not None,
                WAIT_S,
            )
            done, error = self._done, self.error
        assert reached and error is None, (
            f"the reader stopped at cycle {done}: {error!r}"
        )

    def stop(self) -> None:
        with self._condition:
            self._stopped = True
            self._condition.notify_all()
        self._thread.join(WAIT_S)
        assert not self._thread.is_alive(), "the reader did not stop"

    def _run(self) -> None:
        try:
            for cycle in itertools.count(1):
                with self._condition:
                    self._condition.wait_for(
                        lambda: self._granted >= cycle or self._stopped,
                        WAIT_S,
                    )
                    if self._stopped or self._granted < cycle:
                        return
                self._vehicle.ingest(sim_state_message())
                stale = cycle % 5 == 0
                # 30 ms under this cycle's stamp is 10 ms under the last
                # admitted one: the router rejects it as stale.
                self._vehicle.ingest(
                    attitude_message(_stamp(cycle) - (30 if stale else 0))
                )
                with self._condition:
                    self._done = cycle
                    self._condition.notify_all()
        except BaseException as error:  # noqa: BLE001 - reported by wait
            with self._condition:
                self.error = error
                self._condition.notify_all()


def test_every_ruling_admitted_inside_the_window_has_exactly_one_row(
    monkeypatch,
) -> None:
    """start() and close() race the reader thread: whichever ruling each
    boundary lands on, every admission strictly between them has one row,
    and no row names a rejection. The grants bound where the boundaries
    can land: s0 in 10..30 and s1 in 40..60, so the window holds 31..39
    at least, the stale ruling 35 among them."""
    vehicle = RouterVehicle()
    source, subscription = _source(monkeypatch, vehicle)
    feeder = _Feeder(vehicle)
    feeder.start()
    try:
        feeder.grant(30)
        feeder.wait_done(10)
        source.start()
        feeder.grant(30)
        feeder.wait_done(40)
        ending = source.close()
        feeder.grant(30)
        feeder.wait_done(90)
    finally:
        feeder.stop()
    subscription.cancel()

    capture = source.determinism_trace.capture()
    s0, s1, window = _window(capture)
    assert 10 <= s0 <= 30 and 40 <= s1 <= 60, (s0, s1)
    assert set(range(31, 40)) - {35} <= set(window)
    assert _unmatched(capture) == {}
    rejected = {
        ruling for ruling, _, accepted, _ in capture.ledger.entries
        if not accepted
    }
    assert 35 in rejected
    named = {row[5] for row in capture.rows if row[0] == EVENT_ASSOCIATION}
    assert not rejected & named, "a row named a rejected ruling"
    assert capture.ledger.entries[0][0] == 1
    assert capture.ledger.entries[-1][0] == 90
    assert capture.status.callback_faults == 0
    assert (
        subscription.first, subscription.end,
        subscription.faults, subscription.faulted,
    ) == (1, 90, 0, False)
    assert not capture.ledger.incomplete
    verdict = _verdict(source, subscription, ending)
    assert verdict.status == INSPECTABLE
    assert verdict.failed_rules == (14,), verdict.reasons


# The plan review's round-1 probe (the LANDING2 step-1 plan, "Tests and
# verification"): admissions 100 and 120 in both, and the source subscribed
# across a different one, so its one refused association acted on 100 in one
# schedule and on 120 in the other. The rows then read the same: a refusal
# carries no stamp. Now it names the ruling it acted on.
ROUND_1_SCHEDULES = {
    "refuses 100": ("start", 100, "close", 120),
    "refuses 120": (100, "start", 120, "close"),
}


def _round_1(
    monkeypatch: pytest.MonkeyPatch, name: str
) -> tuple[Any, Verdict]:
    """One round-1 schedule through the real router: its capture, and D8's
    verdict on it."""
    vehicle = RouterVehicle()
    source, subscription = _source(monkeypatch, vehicle)
    ending = None
    for step in ROUND_1_SCHEDULES[name]:
        if step == "start":
            source.start()
        elif step == "close":
            ending = source.close()
        else:
            vehicle.ingest(attitude_message(step))
    subscription.cancel()
    verdict = _verdict(source, subscription, ending)
    return source.determinism_trace.capture(), verdict


def _association_labels(capture: Any) -> list[tuple]:
    labels = case_labels(capture).rows
    return [
        labels[index]
        for index, row in enumerate(capture.rows)
        if row[0] == EVENT_ASSOCIATION
    ]


def test_the_round_1_schedules_leave_different_evidence(monkeypatch) -> None:
    """Same ledger and no truth samples, but told apart: each refusal names
    its ruling and the window its bounds, so offline the refusal is placed by
    the ATTITUDE it acted on, slot 5 against slot 6 of the 20 ms grid."""
    early, early_verdict = _round_1(monkeypatch, "refuses 100")
    late, late_verdict = _round_1(monkeypatch, "refuses 120")

    assert early_verdict.failed_rules == (14,), early_verdict.reasons
    assert late_verdict.failed_rules == (14,), late_verdict.reasons
    assert early.ledger.entries == late.ledger.entries == (
        (1, 100, True, None),
        (2, 120, True, None),
    )
    assert [
        [row for row in capture.rows if row[0] == EVENT_ASSOCIATION]
        for capture in (early, late)
    ] == [
        [(EVENT_ASSOCIATION, 0, ASSOCIATION_REFUSED, None, None, 1)],
        [(EVENT_ASSOCIATION, 0, ASSOCIATION_REFUSED, None, None, 2)],
    ]
    assert [_window(capture)[:2] for capture in (early, late)] == [
        (None, 1), (1, 2)
    ]
    assert [_association_labels(capture) for capture in (early, late)] == [
        [(("attitude_slot", 5),)],
        [(("attitude_slot", 6),)],
    ]


def _dropping(
    number: int,
) -> Callable[[Callable[[Any], None]], Callable[[Any], None]]:
    """A delivery of ruling ``number`` that never reaches the ledger, and
    that the tap does not see fail."""

    def deliver(append: Callable[[Any], None]) -> Callable[[Any], None]:
        return lambda ruling: None if ruling.ruling == number else append(ruling)

    return deliver


def test_a_delivery_lost_between_equal_stamps_fails_one_to_one_alone(
    monkeypatch,
) -> None:
    """ATTITUDE 3 repeats ATTITUDE 2's stamp, and its ruling never reaches
    the ledger. Its row names the latest ruling the ledger admitted, 2, whose
    stamp is its own, so stamp consistency cannot see the loss; one-to-one
    does, ruling 2 named twice (D8.9)."""
    _driven_receipts(monkeypatch)
    vehicle = RouterVehicle()
    source, subscription = _source(monkeypatch, vehicle, _dropping(3))
    vehicle.ingest(attitude_message(_stamp(1)))
    source.start()
    for stamp in (_stamp(2), _stamp(2), _stamp(4)):  # 3 repeats 2's stamp
        vehicle.ingest(sim_state_message())
        vehicle.ingest(attitude_message(stamp))
    ending = source.close()
    subscription.cancel()

    capture = source.determinism_trace.capture()
    assert [
        (row[2], row[5]) for row in capture.rows
        if row[0] == EVENT_ASSOCIATION
    ] == [
        (ASSOCIATION_COMMITTED, 2),
        (ASSOCIATION_COMMITTED, 2),
        (ASSOCIATION_COMMITTED, 4),
    ]
    reasons = _reasons(_verdict(source, subscription, ending), 9)
    assert [text for text in reasons if "stamp consistent" in text] == []
    assert (
        "rulings admitted inside the window without exactly one row: 1, "
        "the first (2, 2)"
    ) in reasons, reasons


def test_a_forced_stamp_mismatch_fails_stamp_consistency(monkeypatch) -> None:
    """The ledger is handed ruling 2 with its stamp 20 ms late. The committed
    row for it carries the ATTITUDE's own stamp, which no longer agrees, and
    that alone makes the case ineligible (D8.9)."""
    _driven_receipts(monkeypatch)

    def late(append: Callable[[Any], None]) -> Callable[[Any], None]:
        def deliver(ruling: Any) -> None:
            if ruling.ruling == 2:
                ruling = replace(ruling, time_boot_ms=ruling.time_boot_ms + 20)
            append(ruling)

        return deliver

    vehicle = RouterVehicle()
    source, subscription = _source(monkeypatch, vehicle, late)
    vehicle.ingest(attitude_message(_stamp(1)))
    source.start()
    for ruling in (2, 3):
        vehicle.ingest(sim_state_message())
        vehicle.ingest(attitude_message(_stamp(ruling)))
    ending = source.close()
    subscription.cancel()

    verdict = _verdict(source, subscription, ending)
    assert verdict.failed_rules == (9, 14), verdict.reasons
    [reason] = _reasons(verdict, 9)
    assert reason.startswith(
        "committed or fenced rows not stamp consistent: 1,"
    ), reason


# The last ruling before cancel, as the router rules it: admitted, rejected
# as stale, or admitted on a repeated stamp.
LAST_RULINGS = {
    "admitted": _stamp(4),
    "rejected": _stamp(1),
    "repeated": _stamp(3),
}


@pytest.mark.parametrize("last", sorted(LAST_RULINGS))
def test_a_failed_delivery_of_the_last_ruling_leaves_the_case_ineligible(
    monkeypatch, last
) -> None:
    """D1's test list: the ledger raises taking the last ruling before
    cancel. The tap contains it, and the subscription was owed that ruling
    and lost it, so D8.6 refuses the case whatever the ruling was."""
    _driven_receipts(monkeypatch)
    failure = RuntimeError("the ledger could not take the last ruling")

    def failing(append: Callable[[Any], None]) -> Callable[[Any], None]:
        def deliver(ruling: Any) -> None:
            if ruling.ruling == 4:
                raise failure
            append(ruling)

        return deliver

    vehicle = RouterVehicle()
    source, subscription = _source(monkeypatch, vehicle, failing)
    vehicle.ingest(attitude_message(_stamp(1)))
    source.start()
    for ruling in (2, 3):
        vehicle.ingest(sim_state_message())
        vehicle.ingest(attitude_message(_stamp(ruling)))
    ending = source.close()
    vehicle.ingest(attitude_message(LAST_RULINGS[last]))  # ruling 4, the last
    subscription.cancel()

    assert (
        subscription.end, subscription.faults, subscription.faulted
    ) == (4, 1, True)
    verdict = _verdict(source, subscription, ending)
    assert verdict.status == INSPECTABLE
    assert 6 in verdict.failed_rules, verdict.reasons
