"""The ATTITUDE ledger's subscription, as the direct-pixel child makes it.

Record-only: a vehicle without the router's admission tap, or one whose tap
raises, costs the determinism verdict and never the flight. The evidence the
teardown reads is the subscription's own: the rulings it was owed, first..end,
and whether one was lost, counted or not; and the link's own, the window rows
the journal already held when it subscribed (D8.5 and D8.6 of
the LANDING2 step-1 plan).
"""

from __future__ import annotations

from collections.abc import Callable
from types import SimpleNamespace
from typing import Any

from navpy.modules.vehicle.admission_tap import AdmissionTap
from navpy.modules.vehicle.inbound_router import (
    AUTOPILOT_TELEMETRY_TYPES,
    message_boot_time_ms,
)
from navpy.modules.vision.sim.determinism_events import SUBSCRIPTION_OPENED
from navpy.modules.vision.sim.determinism_trace import DeterminismTrace
from scripts.pixel_pn_admission_ledger import (
    ADMISSION_MESSAGE_TYPE,
    NO_ADMISSION_TAP,
    AdmissionLedgerLink,
    subscribe_admission_ledger,
)
from scripts.pixel_pn_determinism_summary import REPR_FAILED

PERIOD_US = 20_000  # 50 Hz autopilot scheduler period.


def _escaped(call: Callable[[], object]) -> BaseException | None:
    """What ``call`` raised. Caught as BaseException rather than with
    ``pytest.raises``: an interrupt that escaped would end the whole pytest
    session instead of failing one test."""
    try:
        call()
    except BaseException as escaped:  # noqa: BLE001 - the caller asserts
        return escaped
    return None


def _evidence(**fields: Any) -> dict[str, Any]:
    return {
        "live": True,
        "first": None,
        "end": None,
        "faults": None,
        "faulted": None,
        "error": None,
        "window_rows": None,
        **fields,
    }


def _trace() -> DeterminismTrace:
    return DeterminismTrace(PERIOD_US, capacity=8)


def _attitude(boot_ms: int) -> SimpleNamespace:
    return SimpleNamespace(time_boot_ms=boot_ms)


def test_tracing_off_subscribes_nothing() -> None:
    """No trace, no ledger to fill, and the vehicle is not even asked."""
    asked: list[tuple] = []
    vehicle = SimpleNamespace(on_admission=lambda *args: asked.append(args))

    assert subscribe_admission_ledger(vehicle, None) is None
    assert asked == []


def test_a_live_subscription_reports_the_rulings_it_was_owed() -> None:
    """Through a real tap: the ledger is subscribed to ATTITUDE, fills as the
    rulings are made, and once cancelled the evidence names first..end."""
    trace = _trace()
    tap = AdmissionTap(message_boot_time_ms, AUTOPILOT_TELEMETRY_TYPES)
    tap.rule("ATTITUDE", _attitude(900), True, None)  # before: not owed
    link = subscribe_admission_ledger(
        SimpleNamespace(on_admission=tap.subscribe), trace
    )
    tap.rule("ATTITUDE", _attitude(1_000), True, None)
    tap.rule("VFR_HUD", SimpleNamespace(), True, None)  # another type
    tap.rule("ATTITUDE", _attitude(1_020), True, None)

    assert link.evidence() == _evidence(
        first=2, end=None, faults=0, faulted=False, window_rows=0
    ), "a live subscription has no end yet"
    link.cancel()
    tap.rule("ATTITUDE", _attitude(1_040), True, None)  # after: not owed

    assert ADMISSION_MESSAGE_TYPE == "ATTITUDE"
    assert link.evidence() == _evidence(
        first=2, end=3, faults=0, faulted=False, window_rows=0
    )
    assert [entry[0] for entry in trace.ledger.capture().entries] == [2, 3]


def test_a_vehicle_without_the_tap_is_reported_and_not_live() -> None:
    link = subscribe_admission_ledger(
        SimpleNamespace(on_admission=lambda *_: None), _trace()
    )

    link.cancel()  # nothing to cancel, and not an error

    assert link.evidence() == _evidence(live=False, error=NO_ADMISSION_TAP)


def test_a_tap_that_raises_is_reported_by_its_error() -> None:
    error = ValueError("the router does not rule on 'ATTITUDE'")

    def on_admission(*_: object) -> None:
        raise error

    link = subscribe_admission_ledger(
        SimpleNamespace(on_admission=on_admission), _trace()
    )

    assert link.evidence() == _evidence(live=False, error=repr(error))


def test_an_interrupt_while_subscribing_is_passed_on() -> None:
    interrupt = KeyboardInterrupt("ctrl+c while subscribing")

    def on_admission(*_: object) -> None:
        raise interrupt

    escaped = _escaped(lambda: subscribe_admission_ledger(
        SimpleNamespace(on_admission=on_admission), _trace()
    ))

    assert escaped is interrupt


class _Unreadable:
    """A subscription whose ``end`` cannot be read."""

    first = 1
    faults = 0
    faulted = False

    def __init__(self, failure: BaseException) -> None:
        self._failure = failure

    @property
    def end(self) -> int:
        raise self._failure

    def cancel(self) -> None:
        return None


def test_an_evidence_read_that_raises_claims_nothing_else() -> None:
    """The artifact is still written: the read's error, and no bound or
    flag it could not vouch for. Only an interrupt passes."""
    failure = RuntimeError("the tap's lock would not open")
    link = AdmissionLedgerLink(_Unreadable(failure), None)

    assert link.evidence() == _evidence(error=repr(failure))

    interrupt = KeyboardInterrupt("ctrl+c reading the evidence")
    interrupted = AdmissionLedgerLink(_Unreadable(interrupt), None)
    assert _escaped(interrupted.evidence) is interrupt


class _Unrepresentable:
    """An exception argument whose repr raises ``failure``, so repr() of the
    exception that carries it raises too."""

    def __init__(self, failure: BaseException) -> None:
        self._failure = failure

    def __repr__(self) -> str:
        raise self._failure


def _tap_raising(error: BaseException) -> SimpleNamespace:
    def on_admission(*_: object) -> None:
        raise error

    return SimpleNamespace(on_admission=on_admission)


def test_an_unrepresentable_tap_error_costs_only_the_evidence() -> None:
    """The error's text is record-only too: when repr() of what the tap
    raised itself raises, the link reports the artifact's fixed text, and
    nothing reaches the child, whose source would otherwise never start."""
    error = RuntimeError(_Unrepresentable(ValueError("no repr")))
    links: list[AdmissionLedgerLink | None] = []

    escaped = _escaped(lambda: links.append(
        subscribe_admission_ledger(_tap_raising(error), _trace())
    ))

    assert escaped is None
    assert links[0].evidence() == _evidence(live=False, error=REPR_FAILED)


def test_an_interrupt_describing_a_tap_error_is_passed_on() -> None:
    interrupt = KeyboardInterrupt("ctrl+c while describing the error")
    error = RuntimeError(_Unrepresentable(interrupt))

    escaped = _escaped(
        lambda: subscribe_admission_ledger(_tap_raising(error), _trace())
    )

    assert escaped is interrupt


def test_an_unrepresentable_evidence_error_is_a_fixed_text() -> None:
    """The same for a read of the subscription that raises. Only an
    interrupt passes."""
    link = AdmissionLedgerLink(
        _Unreadable(RuntimeError(_Unrepresentable(ValueError("no repr")))),
        None,
    )
    evidence: list[dict[str, Any]] = []

    assert _escaped(lambda: evidence.append(link.evidence())) is None
    assert evidence == [_evidence(error=REPR_FAILED)]

    interrupt = KeyboardInterrupt("ctrl+c while describing the error")
    interrupted = AdmissionLedgerLink(
        _Unreadable(RuntimeError(_Unrepresentable(interrupt))), None
    )
    assert _escaped(interrupted.evidence) is interrupt


def test_a_counted_fault_is_carried_into_the_evidence() -> None:
    """A ruling the tap could not build is owed and lost: counted and
    flagged on the subscription, and the evidence carries both."""
    trace = _trace()

    def unreadable_stamp(message: object) -> int:
        raise RuntimeError("no stamp to read")

    tap = AdmissionTap(unreadable_stamp, AUTOPILOT_TELEMETRY_TYPES)
    link = subscribe_admission_ledger(
        SimpleNamespace(on_admission=tap.subscribe), trace
    )
    tap.rule("ATTITUDE", _attitude(1_000), True, None)
    link.cancel()

    assert link.evidence() == _evidence(
        first=1, end=1, faults=1, faulted=True, window_rows=0
    )
    assert trace.ledger.capture().entries == ()


def test_an_uncounted_loss_is_carried_by_the_flag_alone() -> None:
    """D8.6, amended: an interrupt that cost a ruling flags the subscription
    and is passed on uncounted, so ``faults`` stays 0 and only ``faulted``
    says a ruling was lost. The evidence has to carry the flag, or that
    loss reads as none."""
    tap = AdmissionTap(message_boot_time_ms, AUTOPILOT_TELEMETRY_TYPES)
    interrupt = KeyboardInterrupt("ctrl+c while delivering a ruling")

    def interrupted(ruling: object) -> None:
        raise interrupt

    subscription = tap.subscribe("ATTITUDE", interrupted)
    escaped = _escaped(
        lambda: tap.rule("ATTITUDE", _attitude(1_000), True, None)
    )
    subscription.cancel()

    assert escaped is interrupt
    assert AdmissionLedgerLink(subscription, None).evidence() == _evidence(
        first=1, end=1, faults=0, faulted=True
    )


def _vehicle() -> SimpleNamespace:
    return SimpleNamespace(
        on_admission=AdmissionTap(
            message_boot_time_ms, AUTOPILOT_TELEMETRY_TYPES
        ).subscribe
    )


def test_a_window_already_open_is_counted_when_the_ledger_subscribes() -> None:
    """D8.5's evidence is the link's own: the window rows the journal held
    once the subscription was in place. The source writes its OPENED row as
    it starts, so a ledger subscribed before start() finds none (the tests
    above), and one subscribed after it finds the row it came too late for."""
    trace = _trace()
    trace.record_subscription(epoch=0, outcome=SUBSCRIPTION_OPENED)

    link = subscribe_admission_ledger(_vehicle(), trace)

    assert link.evidence() == _evidence(
        first=1, end=None, faults=0, faulted=False, window_rows=1
    )


def _unjournaled(failure: BaseException) -> SimpleNamespace:
    """A trace's ledger, beside a journal whose read raises ``failure``."""

    def capture() -> None:
        raise failure

    return SimpleNamespace(
        ledger=_trace().ledger, journal=SimpleNamespace(capture=capture)
    )


def test_a_journal_that_cannot_be_read_leaves_the_order_unknown() -> None:
    """None, never 0: a read that failed says nothing about the order. The
    subscription stands and is reported; only an interrupt passes."""
    failure = RuntimeError("the journal's lock would not open")

    link = subscribe_admission_ledger(_vehicle(), _unjournaled(failure))

    assert link.evidence() == _evidence(
        first=1, end=None, faults=0, faulted=False, window_rows=None
    )
    interrupt = KeyboardInterrupt("ctrl+c reading the journal")
    escaped = _escaped(
        lambda: subscribe_admission_ledger(_vehicle(), _unjournaled(interrupt))
    )
    assert escaped is interrupt


def test_the_window_rows_stand_when_the_subscription_cannot_be_read() -> None:
    """The window rows are the link's own, read as it subscribed: a later
    read of the subscription that fails leaves them as they were read."""
    failure = RuntimeError("the tap's lock would not open")
    link = AdmissionLedgerLink(_Unreadable(failure), None, 0)

    assert link.evidence() == _evidence(error=repr(failure), window_rows=0)


def test_a_window_opened_while_the_ledger_subscribes_is_counted() -> None:
    """The count is read once the subscription is in place, so a window row
    written while it was being made, as a source started on another thread
    would write it, is counted rather than missed."""
    trace = _trace()
    tap = AdmissionTap(message_boot_time_ms, AUTOPILOT_TELEMETRY_TYPES)

    def on_admission(message_type: str, callback: Any) -> object:
        subscription = tap.subscribe(message_type, callback)
        trace.record_subscription(epoch=0, outcome=SUBSCRIPTION_OPENED)
        return subscription

    link = subscribe_admission_ledger(
        SimpleNamespace(on_admission=on_admission), trace
    )

    assert link.evidence() == _evidence(
        first=1, end=None, faults=0, faulted=False, window_rows=1
    )
