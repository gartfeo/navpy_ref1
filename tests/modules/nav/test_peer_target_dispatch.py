from __future__ import annotations

import threading
import time
from unittest.mock import Mock, patch

import pytest

from navpy.modules.common.models.location import Location
from navpy.modules.nav.peer_target_dispatch import (
    PeerTargetDispatchPorts,
    PeerTargetDispatchWorker,
)
from navpy.modules.vision.models.detect_data import DetectedObject
from tests.detection_factory import make_detected_target


def _target(task_id: int) -> DetectedObject:
    return make_detected_target(obj_id=task_id, task_id=task_id)


def _start_worker(ports: PeerTargetDispatchPorts) -> PeerTargetDispatchWorker:
    worker = PeerTargetDispatchWorker(ports)
    worker.start()
    return worker


class _Harness:
    def __init__(self) -> None:
        self.active = None
        self.statuses: set[int] = set()
        self.notified: list[list[DetectedObject]] = []
        self.notification = threading.Event()
        self.location = Location(40.0, 44.0, 100.0)
        self.resolve = Mock(return_value=self.location)
        self.warnings: list[str] = []
        self.errors: list[tuple[str, Exception]] = []

    def ports(self) -> PeerTargetDispatchPorts:
        return PeerTargetDispatchPorts(
            active_target=lambda: self.active,
            resolve_location=self.resolve,
            status_exists=lambda target: target.identity.task_id in self.statuses,
            mark_notified=lambda target: self.statuses.add(target.identity.task_id),
            notify_targets=self._notify,
            warn=self.warnings.append,
            error=lambda message, exc: self.errors.append((message, exc)),
        )

    def _notify(self, targets: list[DetectedObject]) -> None:
        self.notified.append(targets)
        self.notification.set()


def test_submit_returns_without_waiting_for_geo_or_network() -> None:
    entered = threading.Event()
    release = threading.Event()
    harness = _Harness()

    def blocking_resolve(target):
        entered.set()
        assert release.wait(timeout=1.0)
        return harness.location

    harness.resolve.side_effect = blocking_resolve
    worker = _start_worker(harness.ports())
    try:
        submit_returned = threading.Event()

        def submit() -> None:
            worker.submit([_target(1)])
            submit_returned.set()

        submit_thread = threading.Thread(target=submit)
        submit_thread.start()
        assert submit_returned.wait(timeout=0.2)
        assert entered.wait(timeout=0.2)
        assert harness.notified == []
        release.set()
        submit_thread.join(timeout=1.0)
        assert harness.notification.wait(timeout=1.0)
    finally:
        release.set()
        worker.stop()


def test_worker_coalesces_to_two_peer_targets_for_three_uav_demo() -> None:
    harness = _Harness()
    worker = _start_worker(harness.ports())
    try:
        worker.submit([_target(1), _target(2), _target(3)])
        assert harness.notification.wait(timeout=1.0)
        assert [
            [target.identity.task_id for target in batch]
            for batch in harness.notified
        ] == [
            [1, 2]
        ]
    finally:
        worker.stop()


def test_worker_excludes_active_and_already_handled_targets() -> None:
    harness = _Harness()
    active = _target(1)
    harness.active = active
    harness.statuses.add(2)
    worker = _start_worker(harness.ports())
    try:
        worker.submit([active, _target(2)])
        worker.stop()
        assert harness.notified == []
        harness.resolve.assert_not_called()
    finally:
        worker.stop()


def test_reset_cancels_running_preparation_and_discards_queued_generation() -> None:
    first_entered = threading.Event()
    release_first = threading.Event()
    reset_returned = threading.Event()
    harness = _Harness()

    def resolve(target):
        if target.identity.task_id == 1:
            first_entered.set()
            assert release_first.wait(timeout=1.0)
        return harness.location

    harness.resolve.side_effect = resolve
    worker = _start_worker(harness.ports())
    try:
        first = _target(1)
        worker.submit([first])
        assert first_entered.wait(timeout=1.0)
        worker.submit([_target(2)])

        def reset() -> None:
            worker.reset()
            reset_returned.set()

        reset_thread = threading.Thread(target=reset)
        reset_thread.start()
        assert reset_returned.wait(timeout=0.2)
        worker.submit([_target(3)])
        release_first.set()
        reset_thread.join(timeout=1.0)
        assert harness.notification.wait(timeout=1.0)
        worker.stop()

        notified_ids = [
            target.identity.task_id
            for batch in harness.notified
            for target in batch
        ]
        assert notified_ids == [3]
        assert first.geo.projected_target_location is None
        assert 2 not in harness.statuses
    finally:
        release_first.set()
        worker.stop()


def test_reset_fails_fast_when_running_effect_cannot_quiesce() -> None:
    entered = threading.Event()
    release = threading.Event()
    harness = _Harness()

    def blocking_notify(targets):
        entered.set()
        release.wait()
        harness._notify(targets)

    ports = harness.ports()
    worker = _start_worker(PeerTargetDispatchPorts(
        active_target=ports.active_target,
        resolve_location=ports.resolve_location,
        status_exists=ports.status_exists,
        mark_notified=ports.mark_notified,
        notify_targets=blocking_notify,
        warn=ports.warn,
        error=ports.error,
    ))
    try:
        worker.submit([_target(1)])
        assert entered.wait(timeout=1.0)

        started_s = time.perf_counter()
        with pytest.raises(TimeoutError) as raised:
            worker.reset(timeout_s=0.01)
        assert time.perf_counter() - started_s < 0.5

        for _ in range(2):
            with pytest.raises(TimeoutError) as persisted:
                worker.raise_if_failed()
            assert persisted.value is raised.value

        worker.submit([_target(2)])
        release.set()
        worker.stop()
        assert [
            target.identity.task_id
            for batch in harness.notified
            for target in batch
        ] == [1]
    finally:
        release.set()
        worker.stop()


def test_stop_timeout_includes_a_running_effect_fence() -> None:
    entered = threading.Event()
    release = threading.Event()
    harness = _Harness()

    def blocking_resolve(_target):
        entered.set()
        release.wait()
        return harness.location

    harness.resolve.side_effect = blocking_resolve
    worker = _start_worker(harness.ports())
    try:
        worker.submit([_target(1)])
        assert entered.wait(timeout=1.0)

        started_s = time.perf_counter()
        with pytest.raises(TimeoutError) as raised:
            worker.stop(timeout_s=0.01)
        assert time.perf_counter() - started_s < 0.5
        assert worker.is_alive

        release.set()
        worker.stop(timeout_s=1.0)
        assert not worker.is_alive
        with pytest.raises(TimeoutError) as persisted:
            worker.raise_if_failed()
        assert persisted.value is raised.value
    finally:
        release.set()
        if worker.is_alive:
            worker.stop(timeout_s=1.0)


def test_reset_waits_for_an_inflight_effect_commit() -> None:
    entered = threading.Event()
    release = threading.Event()
    reset_returned = threading.Event()
    harness = _Harness()

    def blocking_notify(targets):
        entered.set()
        assert release.wait(timeout=1.0)
        harness._notify(targets)

    ports = harness.ports()
    worker = _start_worker(PeerTargetDispatchPorts(
        active_target=ports.active_target,
        resolve_location=ports.resolve_location,
        status_exists=ports.status_exists,
        mark_notified=ports.mark_notified,
        notify_targets=blocking_notify,
        warn=ports.warn,
        error=ports.error,
    ))
    try:
        worker.submit([_target(1)])
        assert entered.wait(timeout=1.0)

        reset_thread = threading.Thread(
            target=lambda: (worker.reset(timeout_s=1.0), reset_returned.set())
        )
        reset_thread.start()
        assert not reset_returned.wait(timeout=0.05)
        release.set()
        assert reset_returned.wait(timeout=1.0)
        reset_thread.join(timeout=1.0)

        assert 1 in harness.statuses
    finally:
        release.set()
        worker.stop()


def test_reset_fences_a_missing_geo_warning() -> None:
    entered = threading.Event()
    release = threading.Event()
    reset_returned = threading.Event()
    harness = _Harness()

    def blocking_warn(message: str) -> None:
        entered.set()
        assert release.wait(timeout=1.0)
        harness.warnings.append(message)

    ports = harness.ports()
    harness.resolve.return_value = None
    worker = _start_worker(PeerTargetDispatchPorts(
        active_target=ports.active_target,
        resolve_location=ports.resolve_location,
        status_exists=ports.status_exists,
        mark_notified=ports.mark_notified,
        notify_targets=ports.notify_targets,
        warn=blocking_warn,
        error=ports.error,
    ))
    try:
        worker.submit([_target(1)])
        assert entered.wait(timeout=1.0)

        reset_thread = threading.Thread(
            target=lambda: (worker.reset(timeout_s=1.0), reset_returned.set())
        )
        reset_thread.start()
        assert not reset_returned.wait(timeout=0.05)
        release.set()
        assert reset_returned.wait(timeout=1.0)
        reset_thread.join(timeout=1.0)

        assert harness.warnings == ["Peer target missing geo fix; skipping notify"]
    finally:
        release.set()
        worker.stop()


def test_dispatch_oserror_isolated_from_worker_lifecycle() -> None:
    harness = _Harness()
    failed = threading.Event()
    recovered = threading.Event()
    attempts = 0

    def notify(_targets) -> None:
        nonlocal attempts
        attempts += 1
        if attempts == 1:
            failed.set()
            raise OSError("network unavailable")
        recovered.set()

    ports = harness.ports()
    worker = _start_worker(
        PeerTargetDispatchPorts(
            active_target=ports.active_target,
            resolve_location=ports.resolve_location,
            status_exists=ports.status_exists,
            mark_notified=ports.mark_notified,
            notify_targets=notify,
            warn=ports.warn,
            error=ports.error,
        )
    )
    try:
        worker.submit([_target(1)])
        assert failed.wait(timeout=1.0)
        assert worker.is_alive
        worker.raise_if_failed()
        assert harness.errors
        worker.submit([_target(2)])
        assert recovered.wait(timeout=1.0)
        worker.raise_if_failed()
    finally:
        worker.stop()


def test_preparation_oserror_isolated_from_worker_lifecycle() -> None:
    harness = _Harness()
    failed = threading.Event()
    recovered = threading.Event()
    attempts = 0

    def resolve(_target):
        nonlocal attempts
        attempts += 1
        if attempts == 1:
            raise OSError("geo unavailable")
        return harness.location

    def report(message: str, error: Exception) -> None:
        harness.errors.append((message, error))
        failed.set()

    def notify(targets: list[DetectedObject]) -> None:
        harness._notify(targets)
        recovered.set()

    ports = harness.ports()
    harness.resolve.side_effect = resolve
    worker = _start_worker(PeerTargetDispatchPorts(
        active_target=ports.active_target,
        resolve_location=ports.resolve_location,
        status_exists=ports.status_exists,
        mark_notified=ports.mark_notified,
        notify_targets=notify,
        warn=ports.warn,
        error=report,
    ))
    try:
        worker.submit([_target(1)])
        assert failed.wait(timeout=1.0)
        assert worker.is_alive
        worker.raise_if_failed()

        worker.submit([_target(2)])
        assert recovered.wait(timeout=1.0)
        worker.raise_if_failed()
    finally:
        worker.stop()


def test_reset_fences_a_preparation_error_report() -> None:
    report_entered = threading.Event()
    release_report = threading.Event()
    reset_returned = threading.Event()
    harness = _Harness()

    def resolve(_target):
        raise OSError("geo unavailable")

    def blocking_report(message: str, error: Exception) -> None:
        report_entered.set()
        assert release_report.wait(timeout=1.0)
        harness.errors.append((message, error))

    ports = harness.ports()
    harness.resolve.side_effect = resolve
    worker = _start_worker(PeerTargetDispatchPorts(
        active_target=ports.active_target,
        resolve_location=ports.resolve_location,
        status_exists=ports.status_exists,
        mark_notified=ports.mark_notified,
        notify_targets=ports.notify_targets,
        warn=ports.warn,
        error=blocking_report,
    ))
    try:
        worker.submit([_target(1)])
        assert report_entered.wait(timeout=1.0)

        reset_thread = threading.Thread(
            target=lambda: (worker.reset(timeout_s=1.0), reset_returned.set())
        )
        reset_thread.start()
        assert not reset_returned.wait(timeout=0.05)
        release_report.set()
        assert reset_returned.wait(timeout=1.0)
        reset_thread.join(timeout=1.0)

        assert len(harness.errors) == 1
    finally:
        release_report.set()
        worker.stop()


def test_daemon_failure_is_persistent_and_fences_future_work() -> None:
    harness = _Harness()
    failed = threading.Event()
    failure = TypeError("bad status port")

    def status_exists(_target: DetectedObject) -> bool:
        failed.set()
        raise failure

    ports = harness.ports()
    worker = _start_worker(
        PeerTargetDispatchPorts(
            active_target=ports.active_target,
            resolve_location=ports.resolve_location,
            status_exists=status_exists,
            mark_notified=ports.mark_notified,
            notify_targets=ports.notify_targets,
            warn=ports.warn,
            error=ports.error,
        )
    )
    try:
        worker.submit([_target(1)])
        assert failed.wait(timeout=1.0)
        worker._thread.join(timeout=1.0)

        assert not worker.is_alive
        for _ in range(2):
            with pytest.raises(TypeError) as raised:
                worker.raise_if_failed()
            assert raised.value is failure

        worker.submit([_target(2)])
        assert harness.notified == []
        worker.reset()
        with pytest.raises(TypeError) as raised:
            worker.raise_if_failed()
        assert raised.value is failure
    finally:
        worker.stop()


def test_start_failure_after_native_launch_remains_owned_and_health_visible():
    worker = PeerTargetDispatchWorker(_Harness().ports())
    failure = RuntimeError("thread launch acknowledgement failed")
    original_start = worker._thread.start

    def launch_then_fail():
        original_start()
        raise failure

    with patch.object(worker._thread, "start", side_effect=launch_then_fail):
        with pytest.raises(RuntimeError) as raised:
            worker.start()

    assert raised.value is failure
    worker.stop()
    assert not worker.is_alive
    with pytest.raises(RuntimeError) as persisted:
        worker.raise_if_failed()
    assert persisted.value is failure


def test_dispatch_programmer_typeerror_propagates_from_worker_loop() -> None:
    worker = object.__new__(PeerTargetDispatchWorker)
    job = Mock()
    worker._take_pending = Mock(side_effect=[[job], None])
    prepared = Mock()
    prepared.job = job
    worker._dispatcher = Mock()
    worker._dispatcher.prepare.return_value = [prepared]
    worker._dispatcher.commit.side_effect = TypeError("bad notify signature")

    with pytest.raises(TypeError, match="bad notify signature"):
        worker._run()

    worker._dispatcher.report_error.assert_not_called()
