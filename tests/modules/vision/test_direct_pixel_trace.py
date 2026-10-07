"""Tests for the direct-pixel source's trace adapter and its off switch.

The off switch is the PRODUCTION path: tracing is disabled by default, so the
source holds ``DisabledPixelTrace`` on every real flight. A method that exists
on the recorder but not on the no-op would crash the message handler with
tracing off -- which is why the call surfaces are compared here rather than
assumed.
"""

from __future__ import annotations

import inspect
from collections.abc import Callable
from types import SimpleNamespace

from navpy.modules.vehicle.admission_tap import AdmissionRuling
from navpy.modules.vision.sim import determinism_trace as trace_module
from navpy.modules.vision.sim.determinism_admission_ledger import PassObserver
from navpy.modules.vision.sim.determinism_command_log import NULL_COMMAND_LOOP
from navpy.modules.vision.sim.determinism_events import (
    ASSOCIATION_COMMITTED,
    ASSOCIATION_FENCED,
    ASSOCIATION_REFUSED,
    DISCARD_PENDING_LEG_ENDED,
    DISCARD_PENDING_OVERWRITTEN,
    DISCARD_PUBLISH_LEG_ENDED,
    DISCARD_PUBLISH_OVERWRITTEN,
    EVENT_ASSOCIATION,
    EVENT_DECIMATE,
    EVENT_LIFECYCLE,
    EVENT_SUBSCRIPTION,
    LIFECYCLE_ACTIVATED,
    LIFECYCLE_CLOSED,
    OUTPUT_DISPATCHED,
    OUTPUT_REJECTED,
    STAGE_FENCED,
    STAGE_STAGED,
    SUBSCRIPTION_CLOSED,
    SUBSCRIPTION_OPENED,
)
from navpy.modules.vision.sim.direct_dispatch_metrics import LegBoundary
from navpy.modules.vision.sim.direct_pixel_trace import (
    DirectPixelTrace,
    DisabledPixelTrace,
    build_direct_pixel_trace,
    command_loop_observer,
    dispatch_outcome,
)


SCHEDULER_RATE_HZ = 50.0


def _associated(source_s: float) -> SimpleNamespace:
    return SimpleNamespace(attitude_timestamp_s=source_s)


def _ruling(number: int, *, accepted: bool = True) -> AdmissionRuling:
    """An ATTITUDE ruling as the router's tap hands one to the ledger."""
    return AdmissionRuling(
        "ATTITUDE",
        number,
        1_000 * number,
        accepted,
        None if accepted else "stale_boot",
    )


def _escaped(call: Callable[[], object]) -> BaseException | None:
    """What ``call`` raised. Caught as BaseException rather than with
    ``pytest.raises``: an interrupt that escaped would end the whole pytest
    session instead of failing one test."""
    try:
        call()
    except BaseException as escaped:  # noqa: BLE001 - the caller asserts
        return escaped
    return None


def _public_calls(cls: type) -> dict[str, inspect.Signature]:
    return {
        name: inspect.signature(member)
        for name, member in vars(cls).items()
        if not name.startswith("_") and inspect.isfunction(member)
    }


def test_the_off_switch_answers_every_call_the_recorder_does() -> None:
    """Disabled is the default, so a missing no-op breaks real flights."""
    assert _public_calls(DisabledPixelTrace) == _public_calls(DirectPixelTrace)


def test_disabled_by_default_and_enabled_by_the_module_gate(
    monkeypatch,
) -> None:
    monkeypatch.setattr(trace_module, "ENABLED", False)
    disabled = build_direct_pixel_trace(SCHEDULER_RATE_HZ)
    assert isinstance(disabled, DisabledPixelTrace)
    assert disabled.trace is None
    assert disabled.watermark_us() is None

    monkeypatch.setattr(trace_module, "ENABLED", True)
    enabled = build_direct_pixel_trace(SCHEDULER_RATE_HZ)
    assert isinstance(enabled, DirectPixelTrace)
    assert enabled.trace is not None


def test_the_grid_period_comes_from_the_scheduler_rate(monkeypatch) -> None:
    """W1's grid is the SCHEDULER period, not the 0.8x pose period."""
    monkeypatch.setattr(trace_module, "ENABLED", True)
    assert build_direct_pixel_trace(50.0).trace.period_us == 20_000
    assert build_direct_pixel_trace(400.0).trace.period_us == 2_500


def test_every_no_op_accepts_the_arguments_the_source_passes(
    monkeypatch,
) -> None:
    """Exercise the disabled surface exactly as the message handler does."""
    monkeypatch.setattr(trace_module, "ENABLED", False)
    trace = build_direct_pixel_trace(SCHEDULER_RATE_HZ)
    associated = _associated(12.5)
    trace.leg_ended(LegBoundary(LIFECYCLE_ACTIVATED, 0, 1, associated, None))
    trace.truth(1, True)
    trace.truth(1, False)
    trace.unbracketable(1, associated)
    trace.association_refused(1)
    trace.association_fenced(1, associated)
    trace.association_committed(1, associated, None)
    trace.association_committed(1, associated, associated)
    trace.stage(1, STAGE_FENCED, associated, None, None)
    trace.stage(1, STAGE_STAGED, associated, None, associated)
    trace.output(1, OUTPUT_DISPATCHED, None, None)
    trace.subscription_opened(0)
    trace.subscription_closed(1)
    assert trace.trace is None


def test_the_off_switch_registers_the_callback_itself(monkeypatch) -> None:
    """With tracing off the stream link subscribes exactly the callback it
    was given, as it did before there was a guard to put around it."""
    monkeypatch.setattr(trace_module, "ENABLED", False)
    trace = build_direct_pixel_trace(SCHEDULER_RATE_HZ)

    def callback(message: object) -> None:
        return None

    assert trace.guard_callback(callback) is callback


def test_the_recorder_reports_the_watermark_its_caller_must_capture(
    monkeypatch,
) -> None:
    """The source stamps an output row with the watermark it read at TAKE
    time, so the adapter has to expose it before delivery runs unlocked."""
    monkeypatch.setattr(trace_module, "ENABLED", True)
    trace = build_direct_pixel_trace(SCHEDULER_RATE_HZ)
    assert trace.watermark_us() is None
    trace.association_committed(1, _associated(12.510), None)
    assert trace.watermark_us() == 12_510_000


def test_a_displaced_clock_is_attributed_after_the_watermark_advances(
    monkeypatch,
) -> None:
    """Order matters: the newer sample must already be the watermark, because
    the older clock was displaced BY it."""
    monkeypatch.setattr(trace_module, "ENABLED", True)
    trace = build_direct_pixel_trace(SCHEDULER_RATE_HZ)
    trace.association_committed(1, _associated(12.510), None)
    trace.association_committed(1, _associated(12.535), _associated(12.510))

    rows = list(trace.trace.capture().rows)
    assert [row[0] for row in rows] == [
        EVENT_ASSOCIATION,
        EVENT_ASSOCIATION,
        EVENT_DECIMATE,
    ]
    discard = rows[2]
    assert discard[2] == DISCARD_PENDING_OVERWRITTEN
    assert discard[3] == 12_510_000
    assert discard[5] == 12_535_000


def test_a_displaced_frame_is_recorded_before_the_frame_that_replaced_it(
    monkeypatch,
) -> None:
    monkeypatch.setattr(trace_module, "ENABLED", True)
    trace = build_direct_pixel_trace(SCHEDULER_RATE_HZ)
    older = SimpleNamespace(pixel=SimpleNamespace(source_timestamp_s=12.510))
    trace.stage(1, STAGE_STAGED, _associated(12.535), None, older)

    rows = list(trace.trace.capture().rows)
    assert rows[0][0] == EVENT_DECIMATE
    assert rows[0][2] == DISCARD_PUBLISH_OVERWRITTEN
    assert rows[0][3] == 12_510_000
    assert rows[1][0] == "stage"


def test_an_output_row_carries_the_watermark_its_caller_captured(
    monkeypatch,
) -> None:
    """Delivery runs unlocked. A stamp that arrives DURING delivery must not
    make the frame look staler than it was when the dispatch chose it."""
    monkeypatch.setattr(trace_module, "ENABLED", True)
    trace = build_direct_pixel_trace(SCHEDULER_RATE_HZ)
    frame = SimpleNamespace(pixel=SimpleNamespace(source_timestamp_s=12.500))
    taken_at_us = trace.watermark_us()
    # An ATTITUDE lands while the consumer is still working.
    trace.association_committed(1, _associated(12.600), None)
    trace.output(1, OUTPUT_DISPATCHED, frame, taken_at_us)

    row = list(trace.trace.capture().rows)[-1]
    assert row[3] == 12_500_000
    assert row[5] is None and row[6] is None
    assert trace.trace.watermark_us() == 12_600_000


def test_a_boundary_row_comes_first_and_needs_no_emptied_slot(
    monkeypatch,
) -> None:
    """W5: the boundary is visible even when both slots were empty, and a
    reader meets it before the discards it caused, all at the ENDING epoch,
    with the epoch it made current and the watermark at that moment."""
    monkeypatch.setattr(trace_module, "ENABLED", True)
    trace = build_direct_pixel_trace(SCHEDULER_RATE_HZ)
    trace.association_committed(0, _associated(12.510), None)
    held = _associated(12.535)
    shown = SimpleNamespace(pixel=SimpleNamespace(source_timestamp_s=12.490))
    trace.leg_ended(LegBoundary(LIFECYCLE_ACTIVATED, 0, 1, held, shown))
    trace.leg_ended(LegBoundary(LIFECYCLE_CLOSED, 1, 2, None, None))

    assert list(trace.trace.capture().rows)[1:] == [
        (EVENT_LIFECYCLE, 0, LIFECYCLE_ACTIVATED, 1, 12_510_000, 625),
        (EVENT_DECIMATE, 0, DISCARD_PENDING_LEG_ENDED, 12_535_000, 626,
         12_510_000, 625),
        (EVENT_DECIMATE, 0, DISCARD_PUBLISH_LEG_ENDED, 12_490_000, 624,
         12_510_000, 625),
        (EVENT_LIFECYCLE, 1, LIFECYCLE_CLOSED, 2, 12_510_000, 625),
    ]


def test_a_subscription_row_names_the_latest_ruling_observed(
    monkeypatch,
) -> None:
    """OBSERVED, admitted or not: the two rows bound the rulings made while
    the source was subscribed (D3), and a rejection is one of them. None
    while the ledger held no ruling at all."""
    monkeypatch.setattr(trace_module, "ENABLED", True)
    trace = build_direct_pixel_trace(SCHEDULER_RATE_HZ)
    ledger = trace.trace.ledger
    trace.subscription_opened(0)
    ledger.append(_ruling(1))
    ledger.append(_ruling(2, accepted=False))
    trace.subscription_closed(1)

    assert [
        row for row in trace.trace.capture().rows
        if row[0] == EVENT_SUBSCRIPTION
    ] == [
        (EVENT_SUBSCRIPTION, 0, SUBSCRIPTION_OPENED, None),
        (EVENT_SUBSCRIPTION, 1, SUBSCRIPTION_CLOSED, 2),
    ]


def test_an_association_row_names_the_latest_ruling_admitted(
    monkeypatch,
) -> None:
    """ADMITTED only: the associator reads the newest ATTITUDE the router
    stored, and a rejected one never reaches the store (D3). A refusal and
    a fence name it as well, since they ruled on that ATTITUDE too."""
    monkeypatch.setattr(trace_module, "ENABLED", True)
    trace = build_direct_pixel_trace(SCHEDULER_RATE_HZ)
    ledger = trace.trace.ledger
    trace.association_refused(0)
    ledger.append(_ruling(1))
    ledger.append(_ruling(2, accepted=False))
    trace.association_committed(0, _associated(12.5), None)
    trace.association_refused(0)
    ledger.append(_ruling(3))
    trace.association_fenced(0, _associated(12.52))

    assert [
        (row[2], row[5]) for row in trace.trace.capture().rows
        if row[0] == EVENT_ASSOCIATION
    ] == [
        (ASSOCIATION_REFUSED, None),
        (ASSOCIATION_COMMITTED, 1),
        (ASSOCIATION_REFUSED, 1),
        (ASSOCIATION_FENCED, 3),
    ]


def test_a_guarded_callback_counts_each_raise_and_raises_the_same_object(
    monkeypatch,
) -> None:
    """The callback registry catches what a message callback raises, so
    without this count a message the source stopped part-way through leaves
    no mark (D3). The registry must still see what it saw before: the same
    object, an interrupt too, and nothing counted for a callback that
    returned."""
    monkeypatch.setattr(trace_module, "ENABLED", True)
    trace = build_direct_pixel_trace(SCHEDULER_RATE_HZ)
    failures: dict[str, BaseException] = {
        "fault": RuntimeError("the source's callback failed"),
        "interrupt": KeyboardInterrupt(),
    }
    handed: list[str] = []

    def callback(message: str) -> None:
        handed.append(message)
        if message in failures:
            raise failures[message]

    guarded = trace.guard_callback(callback)
    guarded("clean")
    assert trace.trace.capture().status.callback_faults == 0

    assert _escaped(lambda: guarded("fault")) is failures["fault"]
    assert _escaped(lambda: guarded("interrupt")) is failures["interrupt"]

    status = trace.trace.capture().status
    assert handed == ["clean", "fault", "interrupt"]
    assert status.callback_faults == 2
    assert status.complete is False
    assert status.failed is False, "a callback's fault is not the recorder's"


def test_the_worker_observer_samples_the_ledger_on_every_pass(
    monkeypatch,
) -> None:
    """Tracing on, every pass records the ATTITUDE ledger's cutoff as it
    began, and a command goes to the command log as before; off, the worker
    gets the no-op."""
    assert command_loop_observer(None) is NULL_COMMAND_LOOP
    monkeypatch.setattr(trace_module, "ENABLED", True)
    trace = build_direct_pixel_trace(SCHEDULER_RATE_HZ).trace
    observer = command_loop_observer(trace)
    assert isinstance(observer, PassObserver)

    observer.note_iteration(1)
    trace.ledger.append(_ruling(1))
    trace.ledger.append(_ruling(2, accepted=False))
    observer.note_iteration(2)
    observer.note_command(2, None)

    commands = trace.capture().commands
    assert commands.passes == ((1, None, None), (2, 2, 1))
    assert commands.entries == ((2, None),)
    assert commands.iterations == 2
    assert trace.command_log.current_iteration() == 2


def test_dispatch_outcome_separates_refusal_from_success() -> None:
    assert dispatch_outcome(True) == OUTPUT_DISPATCHED
    assert dispatch_outcome(False) == OUTPUT_REJECTED
