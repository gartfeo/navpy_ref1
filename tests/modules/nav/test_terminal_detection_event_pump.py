"""Exclusive terminal source pump lifecycle and routing tests."""

from __future__ import annotations

import threading
import time
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest

from navpy.modules.navigation.navigation_command_slot import NavigationCommandSlot
from navpy.modules.navigation.nav.vision_nav.frame import TerminalVisionFrame
from navpy.modules.navigation.nav.vision_nav.source_epoch import (
    FrameAdmission,
    SourceEpochLedger,
)
from navpy.modules.nav.terminal_detection_event_pump import (
    TerminalDetectionEventPump,
    TerminalDetectionEventPumpPorts,
    TerminalPumpFailure,
)
from navpy.modules.nav.terminal_command_dispatch import (
    TerminalCommandDispatch,
    TerminalCommandPorts,
)
from navpy.modules.nav.terminal_nav_workflow import (
    TerminalNavPorts,
    TerminalNavWorkflow,
)
from navpy.modules.nav.terminal_source_event_handler import (
    TerminalSourceEventHandler,
    TerminalSourceEventHandlerPorts,
)
from navpy.modules.nav.terminal_source_reset_fence import (
    TerminalSourceResetFence,
)
from navpy.modules.nav.terminal_publication_admission import (
    TerminalPublicationAdmission,
    TerminalPublicationAdmissionPorts,
)
from navpy.modules.nav.terminal_source_contracts import NavSourceBatch
from navpy.modules.vehicle.flight_mode import FlightMode
from navpy.modules.vision.sim.detection_publication_store import (
    DetectionPublicationStore,
)
from tests.detection_factory import make_detected_target


def _capture_error(action, errors) -> None:
    try:
        action()
    except BaseException as error:
        errors.append(error)


def _publish(store: DetectionPublicationStore, timestamp_s: float) -> None:
    slot = store.reserve(threading.Event(), threading.Event())
    assert slot is not None
    target = make_detected_target(
        obj_id=7,
        x_error=0.0,
        y_error=0.0,
        k=None,
        timestamp=timestamp_s,
    )
    assert store.publish(
        slot,
        [target],
        primary_target=target,
        source_timestamp_s=timestamp_s,
        source_receipt_timestamp_s=None,
        source_name="ideal_360",
        source_discontinuity=False,
    )


def _pump(
    store,
    handler,
    *,
    failed=None,
    wall_s=lambda: 0.0,
    receipt_max_wall_age_s=lambda: 1.0,
):
    failed = failed or Mock()
    if not hasattr(handler, "handle_reset"):
        handler = SimpleNamespace(
            handle=handler.handle,
            handle_reset=lambda: None,
        )
    pump = TerminalDetectionEventPump(
        TerminalDetectionEventPumpPorts(
            open_lease=lambda pump_reset: store.open_event_lease(pump_reset),
            report_failure=failed,
            wall_s=wall_s,
            receipt_max_wall_age_s=receipt_max_wall_age_s,
        ),
        handler,
    )
    return pump, failed


def _activate_pump(pump: TerminalDetectionEventPump) -> bool:
    return pump.prepare() and pump.activate()


def _dispatch_available(pump: TerminalDetectionEventPump) -> bool:
    return pump.dispatch_available()


def test_half_speed_silence_before_first_event_expires_after_scaled_window():
    store = DetectionPublicationStore(source_driven=True, capacity=1)
    wall_s = [1000.0]
    handler = Mock()
    handler.handle_reset = Mock()
    pump, failed = _pump(
        store,
        handler,
        wall_s=lambda: wall_s[0],
        receipt_max_wall_age_s=lambda: 1.0 / 0.5,
    )

    assert _activate_pump(pump)
    wall_s[0] = 1001.99
    assert pump.fail_if_source_receipt_expired() is False
    wall_s[0] = 1002.01
    assert pump.fail_if_source_receipt_expired() is True

    assert pump.is_open is False
    handler.handle.assert_not_called()
    handler.handle_reset.assert_called_once_with()
    failed.assert_called_once_with(TerminalPumpFailure("source_receipt_timeout"))


def test_expiry_transition_cannot_be_reactivated_during_cleanup() -> None:
    store = DetectionPublicationStore(source_driven=True, capacity=1)
    wall_s = [0.0]
    cleanup_entered = threading.Event()
    release_cleanup = threading.Event()
    handler = Mock()
    handler.handle_reset = Mock()
    pump, _failed = _pump(
        store,
        handler,
        wall_s=lambda: wall_s[0],
        receipt_max_wall_age_s=lambda: 0.1,
    )
    assert _activate_pump(pump)
    wall_s[0] = 0.11
    original_stop = pump._stop_session

    def stop_after_release(session) -> None:
        cleanup_entered.set()
        assert release_cleanup.wait(timeout=1.0)
        original_stop(session)

    expired: list[bool] = []
    errors: list[BaseException] = []
    with patch.object(pump, "_stop_session", side_effect=stop_after_release):
        caller = threading.Thread(
            target=lambda: _capture_error(
                lambda: expired.append(pump.fail_if_source_receipt_expired()),
                errors,
            )
        )
        caller.start()
        assert cleanup_entered.wait(timeout=1.0)

        assert pump.activate() is False
        assert pump._dispatch.handle_publication(Mock()) is False
        release_cleanup.set()
        caller.join(timeout=1.0)

    assert not caller.is_alive()
    assert not errors
    assert expired == [True]
    assert pump.is_open is False


def test_ten_times_speed_reset_rearms_silence_window_without_replay():
    store = DetectionPublicationStore(source_driven=True, capacity=1)
    wall_s = [1000.0]
    handled = threading.Event()
    handler = Mock()
    handler.handle.side_effect = lambda _publication: handled.set() or True
    pump, failed = _pump(
        store,
        handler,
        wall_s=lambda: wall_s[0],
        receipt_max_wall_age_s=lambda: 1.0 / 10.0,
    )

    assert _activate_pump(pump)
    wall_s[0] = 1000.05
    _publish(store, 1.0)
    assert _dispatch_available(pump) is True
    assert handled.wait(1.0)
    store.clear(mark_discontinuity=True)

    wall_s[0] = 1000.14
    assert pump.fail_if_source_receipt_expired() is False
    wall_s[0] = 1000.16
    assert pump.fail_if_source_receipt_expired() is True

    assert handler.handle.call_count == 1
    assert handler.handle_reset.call_count == 2
    failed.assert_called_once_with(TerminalPumpFailure("source_receipt_timeout"))


class _ThrowingCloseLease:
    def __init__(self) -> None:
        self._closed = threading.Event()

    @property
    def closed(self) -> bool:
        return self._closed.is_set()

    def wait_and_dispatch(self, _handler):
        self._closed.wait(1.0)
        return None

    def close(self) -> None:
        self._closed.set()
        raise RuntimeError("lease close failed")


def test_source_timeout_reports_failure_when_lease_close_raises():
    wall_s = [0.0]
    failed = Mock()
    lease = _ThrowingCloseLease()
    handler = Mock()
    pump = TerminalDetectionEventPump(
        TerminalDetectionEventPumpPorts(
            open_lease=lambda _reset: lease,
            report_failure=failed,
            wall_s=lambda: wall_s[0],
            receipt_max_wall_age_s=lambda: 0.1,
        ),
        handler,
    )
    assert _activate_pump(pump)

    wall_s[0] = 0.11
    assert pump.fail_if_source_receipt_expired() is True

    failure = failed.call_args.args[0]
    assert failure.reason == "source_receipt_timeout"
    assert isinstance(failure.error, RuntimeError)
    assert str(failure.error) == "lease close failed"
    handler.handle_reset.assert_called_once_with()


def test_prepare_does_not_start_a_source_dispatch_thread(monkeypatch):
    started_names: list[str] = []
    original_start = threading.Thread.start

    def record_start(thread) -> None:
        started_names.append(thread.name)
        original_start(thread)

    monkeypatch.setattr(threading.Thread, "start", record_start)
    store = DetectionPublicationStore(source_driven=True, capacity=1)
    pump, failed = _pump(store, Mock())

    assert pump.prepare() is True
    assert "terminal-detection-dispatch" not in started_names
    pump.close()
    failed.assert_not_called()


def test_source_timeout_reports_failure_when_reset_cleanup_raises():
    store = DetectionPublicationStore(source_driven=True, capacity=1)
    wall_s = [0.0]
    failed = Mock()
    handler = Mock()
    handler.handle_reset.side_effect = RuntimeError("reset failed")
    pump, _ = _pump(
        store,
        handler,
        failed=failed,
        wall_s=lambda: wall_s[0],
        receipt_max_wall_age_s=lambda: 0.1,
    )
    assert _activate_pump(pump)

    wall_s[0] = 0.11
    assert pump.fail_if_source_receipt_expired() is True

    failure = failed.call_args.args[0]
    assert failure.reason == "source_receipt_timeout"
    assert isinstance(failure.error, RuntimeError)
    assert str(failure.error) == "reset failed"


def test_pump_routes_latest_backlogged_publication_and_releases_lease():
    store = DetectionPublicationStore(source_driven=True, capacity=2)
    handled = []
    done = threading.Event()

    def handle(publication) -> bool:
        handled.append(publication.source_timestamp_s)
        done.set()
        return True

    pump, failed = _pump(store, SimpleNamespace(handle=handle))
    assert pump.prepare()
    _publish(store, 1.0)
    _publish(store, 2.0)
    assert pump.activate()
    assert _dispatch_available(pump) is True
    assert done.wait(1.0)

    pump.close()

    assert handled == [2.0]
    assert pump.is_open is False
    failed.assert_not_called()


def test_prepared_pump_does_not_dispatch_until_activated():
    store = DetectionPublicationStore(source_driven=True, capacity=1)
    handled = threading.Event()
    handler = SimpleNamespace(
        handle=lambda _publication: handled.set() or True,
        handle_reset=lambda: None,
    )
    pump, failed = _pump(store, handler)

    assert pump.prepare()
    _publish(store, 1.0)
    assert not handled.wait(0.05)
    assert _dispatch_available(pump) is False

    assert pump.activate()
    assert _dispatch_available(pump) is True
    assert handled.wait(1.0)
    pump.close()
    failed.assert_not_called()


def test_handler_failure_keeps_stream_claimed_until_lifecycle_close():
    store = DetectionPublicationStore(source_driven=True, capacity=2)
    rejected = threading.Event()

    def reject(_publication) -> bool:
        rejected.set()
        return False

    pump, _failed = _pump(store, SimpleNamespace(handle=reject))
    assert _activate_pump(pump)
    _publish(store, 1.0)
    assert _dispatch_available(pump) is True
    assert rejected.wait(1.0)
    _publish(store, 2.0)
    assert _dispatch_available(pump) is False

    assert pump.is_open is True
    assert store.drain() == []
    pump.close()
    assert [event.source_timestamp_s for event in store.drain()] == [2.0]


def test_close_waits_for_in_flight_dispatch_before_returning():
    store = DetectionPublicationStore(source_driven=True, capacity=1)
    entered = threading.Event()
    release = threading.Event()

    def handle(_publication) -> bool:
        entered.set()
        assert release.wait(1.0)
        return True

    pump, _failed = _pump(store, SimpleNamespace(handle=handle))
    assert _activate_pump(pump)
    _publish(store, 1.0)
    dispatcher = threading.Thread(target=lambda: _dispatch_available(pump))
    dispatcher.start()
    assert entered.wait(1.0)
    _publish(store, 2.0)
    closed = threading.Event()
    closer = threading.Thread(
        target=lambda: (pump.close(), closed.set()),
    )
    closer.start()
    assert not closed.wait(0.05)

    release.set()
    assert closed.wait(1.0)
    closer.join(1.0)
    dispatcher.join(1.0)
    assert not dispatcher.is_alive()
    assert pump.is_open is False
    assert [
        publication.source_timestamp_s for publication in store.drain()
    ] == [2.0]


def test_blocked_handler_close_is_bounded_retained_and_retryable():
    store = DetectionPublicationStore(source_driven=True, capacity=1)
    entered = threading.Event()
    release = threading.Event()
    handler = Mock()

    def block(_publication) -> bool:
        entered.set()
        assert release.wait(timeout=1.0)
        return True

    handler.handle.side_effect = block
    pump, _failed = _pump(store, handler)
    assert _activate_pump(pump)
    _publish(store, 1.0)
    dispatcher = threading.Thread(target=lambda: _dispatch_available(pump))
    dispatcher.start()
    assert entered.wait(timeout=1.0)

    with patch(
        "navpy.modules.nav.terminal_detection_event_pump."
        "TERMINAL_PUMP_STOP_TIMEOUT_S",
        0.01,
    ):
        started_s = time.perf_counter()
        with pytest.raises(TimeoutError, match="dispatch fence"):
            pump.close()
        assert time.perf_counter() - started_s < 0.2

    assert pump.is_open is False
    handler.handle_reset.assert_not_called()
    release.set()
    dispatcher.join(timeout=1.0)
    assert not dispatcher.is_alive()
    pump.close()
    handler.handle_reset.assert_called_once_with()


def test_concurrent_close_resets_commands_and_closes_lease_once():
    class Lease:
        def __init__(self):
            self.close_calls = 0

        def wait_and_dispatch(self, _handler):
            return None

        def close(self):
            self.close_calls += 1

    lease = Lease()
    reset_entered = threading.Event()
    release_reset = threading.Event()
    handler = Mock()

    def reset() -> None:
        reset_entered.set()
        assert release_reset.wait(timeout=1.0)

    handler.handle_reset.side_effect = reset
    pump = TerminalDetectionEventPump(
        TerminalDetectionEventPumpPorts(
            open_lease=lambda _reset: lease,
            report_failure=Mock(),
            wall_s=lambda: 0.0,
            receipt_max_wall_age_s=lambda: 1.0,
        ),
        handler,
    )
    assert pump.prepare()
    errors = []
    first = threading.Thread(target=lambda: _capture_error(pump.close, errors))
    second = threading.Thread(target=lambda: _capture_error(pump.close, errors))
    first.start()
    assert reset_entered.wait(timeout=1.0)
    second.start()
    release_reset.set()
    first.join(timeout=1.0)
    second.join(timeout=1.0)

    assert not first.is_alive()
    assert not second.is_alive()
    assert errors == []
    assert handler.handle_reset.call_count == 1
    assert lease.close_calls == 1
    assert pump._session is None


def test_concurrent_close_cannot_hide_failed_lease_cleanup():
    failure = RuntimeError("lease close failed")

    class Lease:
        def __init__(self):
            self.close_calls = 0

        def wait_and_dispatch(self, _handler):
            return None

        def close(self):
            self.close_calls += 1
            raise failure

    lease = Lease()
    handler = Mock()
    pump = TerminalDetectionEventPump(
        TerminalDetectionEventPumpPorts(
            open_lease=lambda _reset: lease,
            report_failure=Mock(),
            wall_s=lambda: 0.0,
            receipt_max_wall_age_s=lambda: 1.0,
        ),
        handler,
    )
    assert pump.prepare()
    start = threading.Barrier(3)
    errors = []

    def close() -> None:
        start.wait(timeout=1.0)
        _capture_error(pump.close, errors)

    callers = [threading.Thread(target=close) for _ in range(2)]
    for caller in callers:
        caller.start()
    start.wait(timeout=1.0)
    for caller in callers:
        caller.join(timeout=1.0)

    assert all(not caller.is_alive() for caller in callers)
    assert errors == [failure, failure]
    assert lease.close_calls == 2
    assert handler.handle_reset.call_count == 1
    assert pump._session is not None


def test_source_clear_invalidates_pre_reset_pending_worker_frame():
    store = DetectionPublicationStore(source_driven=True, capacity=1)
    slot = NavigationCommandSlot(threading.RLock(), threading.Event())
    handled = threading.Event()

    class SlotHandler:
        @staticmethod
        def handle(publication) -> bool:
            slot.replace(publication)
            slot.signal_pending()
            handled.set()
            return True

        @staticmethod
        def handle_reset() -> None:
            slot.invalidate()

    pump, failed = _pump(store, SlotHandler())
    assert _activate_pump(pump)
    _publish(store, 1.0)
    assert _dispatch_available(pump) is True
    assert handled.wait(1.0)
    assert slot.peek().source_timestamp_s == 1.0

    store.clear(mark_discontinuity=True)

    assert slot.take_or_else(lambda: None) is None
    handled.clear()
    _publish(store, 2.0)
    assert _dispatch_available(pump) is True
    assert handled.wait(1.0)
    work = slot.take_or_else(lambda: None)
    assert work is not None
    assert work.payload.source_timestamp_s == 2.0
    assert work.payload.source_discontinuity is True
    slot.finish(work)
    pump.close()
    failed.assert_not_called()


def test_source_stop_invalidates_pending_worker_frame_before_returning():
    store = DetectionPublicationStore(source_driven=True, capacity=1)
    slot = NavigationCommandSlot(threading.RLock(), threading.Event())
    handled = threading.Event()

    class SlotHandler:
        @staticmethod
        def handle(publication) -> bool:
            slot.replace(publication)
            slot.signal_pending()
            handled.set()
            return True

        @staticmethod
        def handle_reset() -> None:
            slot.invalidate()

    pump, failed = _pump(store, SlotHandler())
    assert _activate_pump(pump)
    _publish(store, 1.0)
    assert _dispatch_available(pump) is True
    assert handled.wait(1.0)

    store.stop()

    assert slot.take_or_else(lambda: None) is None
    pump.close()
    assert failed.call_count <= 1


def test_source_reset_waits_for_bootstrap_then_invalidates_queued_work():
    order = []
    entered = threading.Event()
    release = threading.Event()
    fence = TerminalSourceResetFence(
        lambda: order.append("discard"),
        lambda: order.append("invalidate"),
    )

    def bootstrap() -> None:
        with fence.bootstrap():
            entered.set()
            assert release.wait(1.0)
            order.append("dispatch")

    bootstrap_thread = threading.Thread(target=bootstrap)
    bootstrap_thread.start()
    assert entered.wait(1.0)
    reset_done = threading.Event()
    reset_thread = threading.Thread(
        target=lambda: (fence.reset(), reset_done.set()),
    )
    reset_thread.start()
    assert not reset_done.wait(0.05)

    release.set()
    bootstrap_thread.join(1.0)
    reset_thread.join(1.0)

    assert order == ["dispatch", "discard", "invalidate"]
    assert reset_done.is_set()


def test_reset_between_bootstrap_dispatch_and_activation_invalidates_work():
    store = DetectionPublicationStore(source_driven=True, capacity=2)
    _publish(store, 1.0)
    slot = NavigationCommandSlot(threading.RLock(), threading.Event())
    fence = TerminalSourceResetFence(
        lambda: None,
        lambda: slot.invalidate(),
    )
    handler = SimpleNamespace(
        handle=lambda _publication: True,
        handle_reset=fence.reset,
    )
    pump, failed = _pump(store, handler)
    target = Mock()
    batch = NavSourceBatch(True, Mock(), (), False)
    source = Mock()
    source.find_active_target_detection.return_value = target
    source.collect.return_value = batch
    source.frame_ready.return_value = True
    source.consume.return_value = True
    record = Mock()
    record.commit.return_value = True
    dispatch_entered = threading.Event()
    release_dispatch = threading.Event()
    commands = Mock()

    def dispatch(*_args) -> bool:
        slot.replace("bootstrap-command")
        slot.signal_pending()
        dispatch_entered.set()
        assert release_dispatch.wait(1.0)
        return True

    commands.dispatch.side_effect = dispatch
    workflow = TerminalNavWorkflow(
        TerminalNavPorts(
            vehicle_mode=lambda: FlightMode.GUIDED,
            request_guided=Mock(),
            mark_guided_session=Mock(),
            active_target=lambda: target,
            vision_nav_active=lambda: True,
            command_liveness_failed=lambda: False,
        ),
        source,
        commands,
        record,
        Mock(),
        pump,
        fence.bootstrap,
    )
    workflow_thread = threading.Thread(target=workflow.act_nav)
    workflow_thread.start()
    assert dispatch_entered.wait(1.0)

    reset_done = threading.Event()
    reset_thread = threading.Thread(
        target=lambda: (
            store.clear(mark_discontinuity=True),
            reset_done.set(),
        ),
    )
    reset_thread.start()
    assert not reset_done.wait(0.05)

    release_dispatch.set()
    workflow_thread.join(1.0)
    reset_thread.join(1.0)

    assert not workflow_thread.is_alive()
    assert not reset_thread.is_alive()
    assert reset_done.is_set()
    assert slot.take_or_else(lambda: None) is None
    pump.close()
    failed.assert_not_called()


def test_unavailable_lease_fails_closed_without_starting_thread():
    failed = Mock()
    pump = TerminalDetectionEventPump(
        TerminalDetectionEventPumpPorts(
            open_lease=lambda _reset: None,
            report_failure=failed,
            wall_s=lambda: 0.0,
            receipt_max_wall_age_s=lambda: 1.0,
        ),
        Mock(),
    )

    assert _activate_pump(pump) is False
    assert pump.is_open is False
    failed.assert_called_once_with(TerminalPumpFailure("lease_unavailable"))


def test_command_failure_latch_survives_diagnostic_logger_exceptions():
    active = make_detected_target(obj_id=7, task_id=17)
    failed = Mock()
    logger = Mock()
    logger.info.side_effect = RuntimeError("logger unavailable")
    commands = TerminalCommandDispatch(
        TerminalCommandPorts(Mock(), failed, logger),
        Mock(),
    )

    commands.mark_no_detection(active)
    commands.mark_navigation_rejected(active)
    commands.mark_source_liveness_expired(active)

    assert failed.call_count == 3


def test_regular_event_matches_active_task_before_source_admission():
    active = make_detected_target(
        obj_id=7,
        x_error=0.0,
        y_error=0.0,
        k=None,
        timestamp=1.0,
    )
    commands = Mock()
    commands.source_publication_target.return_value = None
    admission = Mock()
    failed = Mock()
    reset_source = Mock()
    handler = TerminalSourceEventHandler(
        TerminalSourceEventHandlerPorts(
            active_target=lambda: active,
            session_active=lambda: True,
            reset_source=reset_source,
            admission=admission,
            mark_failed=failed,
            logger=Mock(),
            commands=commands,
        )
    )
    publication = Mock(source_discontinuity=False, source_name="ideal_360")

    assert handler.handle(publication) is False
    commands.mark_navigation_rejected.assert_called_once_with(active)
    admission.admit_event.assert_not_called()
    commands.dispatch_target.assert_not_called()
    reset_source.assert_called_once_with()


def test_empty_named_discontinuity_allows_lower_clock_in_new_epoch():
    active = make_detected_target(obj_id=7, task_id=17, timestamp=100.0)
    restarted = make_detected_target(obj_id=7, task_id=17, timestamp=0.1)
    ledger = SourceEpochLedger()
    admitted = []

    def dispatch(target) -> bool:
        source_name = target.pixel.source_name
        generation = ledger.generation(source_name)
        # KEYWORDS, not positions. `TerminalVisionFrame` has grown four
        # aircraft-state fields since this was written, so the positional form
        # silently shifted: the 25.0 meant as the airspeed landed on control_y
        # and `air_speed_mps` inherited 0.0, which the frame rejects outright
        # ("terminal frame airspeed must be positive"). What this test actually
        # exercises is the epoch ledger's use of source name and generation --
        # the ray and the aircraft state are filler that only has to be valid.
        frame = TerminalVisionFrame(
            source_name=source_name,
            source_generation=generation,
            task_id=target.identity.task_id,
            obj_id=target.identity.obj_id,
            source_timestamp_s=target.pixel.source_timestamp_s,
            body_x=1.0,
            body_y=0.0,
            body_z=0.0,
            control_x=1.0,
            control_y=0.0,
            control_z=0.0,
            aircraft_roll_deg=0.0,
            air_speed_mps=25.0,
            aircraft_yaw_rate_rad_s=0.0,
            aircraft_pitch_deg=0.0,
        )
        outcome = ledger.admit(frame)
        admitted.append((frame.source_generation, outcome))
        return outcome is FrameAdmission.FRESH

    commands = Mock()
    commands.source_publication_target.side_effect = [active, None, restarted]
    commands.dispatch_target.side_effect = dispatch
    admission = TerminalPublicationAdmission(
        TerminalPublicationAdmissionPorts(
            clear_discontinuity=ledger.note_discontinuities,
            mark_failed=Mock(),
            logger=Mock(),
        )
    )
    handler = TerminalSourceEventHandler(
        TerminalSourceEventHandlerPorts(
            active_target=lambda: active,
            session_active=lambda: True,
            reset_source=Mock(),
            admission=admission,
            mark_failed=Mock(),
            logger=Mock(),
            commands=commands,
        )
    )

    assert handler.handle(
        Mock(source_discontinuity=False, source_name="test")
    ) is True
    assert handler.handle(
        Mock(source_discontinuity=True, source_name="test")
    ) is True
    assert handler.handle(
        Mock(source_discontinuity=False, source_name="test")
    ) is True

    assert admitted == [
        (0, FrameAdmission.FRESH),
        (1, FrameAdmission.FRESH),
    ]


def test_pump_stays_open_across_empty_named_discontinuity():
    store = DetectionPublicationStore(source_driven=True, capacity=3)
    active = make_detected_target(obj_id=7, task_id=17, timestamp=100.0)
    active.replace_pixel(replace(active.pixel, source_name="ideal_360"))
    restarted = make_detected_target(obj_id=7, task_id=17, timestamp=0.1)
    restarted.replace_pixel(replace(restarted.pixel, source_name="ideal_360"))
    ledger = SourceEpochLedger()
    admitted = []
    first_dispatched = threading.Event()
    second_dispatched = threading.Event()
    discontinuity_cleared = threading.Event()

    def publish(target, timestamp_s, *, discontinuity=False):
        slot = store.reserve(threading.Event(), threading.Event())
        assert slot is not None
        targets = [] if target is None else [target]
        assert store.publish(
            slot,
            targets,
            primary_target=target,
            source_timestamp_s=timestamp_s,
            source_receipt_timestamp_s=None,
            source_name="ideal_360",
            source_discontinuity=discontinuity,
        )

    def resolve(publication, selected):
        return next(
            (
                target
                for target in publication.detected_targets
                if target.identity.task_id == selected.identity.task_id
            ),
            None,
        )

    def dispatch(target) -> bool:
        source_name = target.pixel.source_name
        frame = TerminalVisionFrame(
            source_name,
            ledger.generation(source_name),
            target.identity.task_id,
            target.identity.obj_id,
            target.pixel.source_timestamp_s,
            1.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
        )
        outcome = ledger.admit(frame)
        admitted.append((frame.source_generation, outcome))
        (first_dispatched if len(admitted) == 1 else second_dispatched).set()
        return outcome is FrameAdmission.FRESH

    def clear(names):
        ledger.note_discontinuities(names)
        discontinuity_cleared.set()

    commands = Mock()
    commands.source_publication_target.side_effect = resolve
    commands.dispatch_target.side_effect = dispatch
    handler = TerminalSourceEventHandler(
        TerminalSourceEventHandlerPorts(
            active_target=lambda: active,
            session_active=lambda: True,
            reset_source=Mock(),
            admission=TerminalPublicationAdmission(
                TerminalPublicationAdmissionPorts(clear, Mock(), Mock())
            ),
            mark_failed=Mock(),
            logger=Mock(),
            commands=commands,
        )
    )
    pump, pump_failed = _pump(store, handler)
    assert _activate_pump(pump)

    publish(active, 100.0)
    assert _dispatch_available(pump) is True
    assert first_dispatched.wait(1.0)
    publish(None, 0.0, discontinuity=True)
    assert _dispatch_available(pump) is True
    assert discontinuity_cleared.wait(1.0)
    publish(restarted, 0.1)
    assert _dispatch_available(pump) is True
    assert second_dispatched.wait(1.0)

    assert pump.is_open is True
    assert admitted == [
        (0, FrameAdmission.FRESH),
        (1, FrameAdmission.FRESH),
    ]
    commands.mark_navigation_rejected.assert_not_called()
    pump_failed.assert_not_called()
    pump.close()


def test_inactive_source_session_rejects_without_false_navigation_failure():
    reset_source = Mock()
    mark_failed = Mock()
    logger = Mock()
    commands = Mock()
    handler = TerminalSourceEventHandler(
        TerminalSourceEventHandlerPorts(
            active_target=Mock(),
            session_active=lambda: False,
            reset_source=reset_source,
            admission=Mock(),
            mark_failed=mark_failed,
            logger=logger,
            commands=commands,
        )
    )

    assert handler.handle(Mock()) is False

    reset_source.assert_called_once_with()
    mark_failed.assert_not_called()
    logger.error.assert_not_called()
    commands.source_publication_target.assert_not_called()


def test_rejected_later_event_invalidates_earlier_pending_work():
    active = make_detected_target(
        obj_id=7,
        x_error=0.0,
        y_error=0.0,
        k=None,
        timestamp=1.0,
    )
    slot = NavigationCommandSlot(threading.RLock(), threading.Event())
    commands = Mock()
    commands.source_publication_target.side_effect = [active, None]
    commands.dispatch_target.side_effect = (
        lambda target: slot.replace(target) is None
    )
    handler = TerminalSourceEventHandler(
        TerminalSourceEventHandlerPorts(
            active_target=lambda: active,
            session_active=lambda: True,
            reset_source=lambda: slot.invalidate(),
            admission=Mock(admit_event=Mock(return_value=True)),
            mark_failed=Mock(),
            logger=Mock(),
            commands=commands,
        )
    )

    assert handler.handle(Mock()) is True
    assert slot.peek() is active
    assert handler.handle(Mock()) is False

    assert slot.take_or_else(lambda: None) is None


def test_named_discontinuity_resets_epoch_before_receipt_and_dispatch():
    active = make_detected_target(
        obj_id=7,
        x_error=0.0,
        y_error=0.0,
        k=None,
        timestamp=1.0,
    )
    order = []
    commands = Mock()
    commands.source_publication_target.return_value = active
    commands.dispatch_target.side_effect = (
        lambda _target: order.append("dispatch") or True
    )
    admission = TerminalPublicationAdmission(
        TerminalPublicationAdmissionPorts(
            clear_discontinuity=lambda _names: order.append("clear"),
            mark_failed=Mock(),
            logger=Mock(),
        )
    )
    handler = TerminalSourceEventHandler(
        TerminalSourceEventHandlerPorts(
            active_target=lambda: active,
            session_active=lambda: True,
            reset_source=Mock(),
            admission=admission,
            mark_failed=Mock(),
            logger=Mock(),
            commands=commands,
        )
    )
    publication = Mock(
        source_discontinuity=True,
        source_name="ideal_360",
    )

    assert handler.handle(publication) is True
    assert order == ["clear", "dispatch"]


def test_unnamed_discontinuity_fails_without_receipt_or_dispatch():
    active = make_detected_target(
        obj_id=7,
        x_error=0.0,
        y_error=0.0,
        k=None,
        timestamp=1.0,
    )
    commands = Mock()
    commands.source_publication_target.return_value = active
    failed = Mock()
    admission = TerminalPublicationAdmission(
        TerminalPublicationAdmissionPorts(
            clear_discontinuity=Mock(),
            mark_failed=failed,
            logger=Mock(),
        )
    )
    handler = TerminalSourceEventHandler(
        TerminalSourceEventHandlerPorts(
            active_target=lambda: active,
            session_active=lambda: True,
            reset_source=Mock(),
            admission=admission,
            mark_failed=Mock(),
            logger=Mock(),
            commands=commands,
        )
    )
    publication = Mock(source_discontinuity=True, source_name=None)

    assert handler.handle(publication) is False
    failed.assert_called_once_with()
    commands.dispatch_target.assert_not_called()
