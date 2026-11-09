"""A failed detector is replaced by a fresh instance behind a stable proxy (docs/design/navpy_recovery.md, step 5b)."""

from __future__ import annotations

import pytest

from navpy.app.perception.restarting_detector import RestartingDetector, stable_node
from navpy.app.perception.vision_detector_factory_ports import VisionDetectorNode
from navpy.core.kernel.restart_budget import RestartBudget
from navpy.core.ports.camera_health import CameraHealth
from navpy.sim.debug_view.vision_ui_ports import SimDebugSource


class _Clock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now


class _Detector:
    def __init__(self, name: int, *, start_error: Exception | None = None, quiet_after_stop: bool = True) -> None:
        self.name = name
        self.failure: BaseException | None = None
        self.started = 0
        self.stopped = 0
        self.start_error = start_error
        self.quiet_after_stop = quiet_after_stop
        self.is_quiescent = False
        self.tracking: list[int] = []

    @property
    def source_name(self) -> str:
        return "main"

    def start(self) -> None:
        self.started += 1
        if self.start_error is not None:
            raise self.start_error

    def stop(self) -> bool:
        self.stopped += 1
        self.is_quiescent = self.quiet_after_stop
        return self.quiet_after_stop

    def raise_if_failed(self) -> None:
        if self.failure is not None:
            raise self.failure

    def start_tracking(self, obj_id: int) -> None:
        self.tracking.append(obj_id)

    def ui_step(self) -> bool:
        return self.name % 2 == 0


class _Factory:
    def __init__(self, *detectors: _Detector) -> None:
        self.queue = list(detectors)
        self.created: list[_Detector] = []

    def __call__(self) -> VisionDetectorNode:
        detector = self.queue.pop(0)
        self.created.append(detector)
        return VisionDetectorNode(
            detector=detector,  # type: ignore[arg-type]
            cleanup=detector,
            real_ui=detector,
            sim_debug=SimDebugSource(read_snapshot=lambda: detector.name),  # type: ignore[arg-type,return-value]
            camera_health=CameraHealth("NavPy", lambda: True),
        )


def _proxy(factory: _Factory, clock: _Clock, reports: list[str]) -> RestartingDetector:
    return RestartingDetector(factory, reports.append, RestartBudget(clock=clock))


def test_healthy_detector_is_never_replaced() -> None:
    factory, clock, reports = _Factory(_Detector(1)), _Clock(), []
    proxy = _proxy(factory, clock, reports)
    proxy.start()
    proxy.raise_if_failed()
    assert factory.created[0].started == 1 and len(factory.created) == 1 and reports == []


def test_failed_detector_is_stopped_and_replaced_by_a_fresh_instance() -> None:
    old, new = _Detector(1), _Detector(2)
    factory, clock, reports = _Factory(old, new), _Clock(), []
    proxy = _proxy(factory, clock, reports)
    node = stable_node(proxy)
    proxy.start()
    old.failure = RuntimeError("capture died")

    proxy.raise_if_failed()   # first try is immediate: no raise, no escalation

    assert old.stopped == 1 and new.started == 1
    assert proxy.instance is new
    proxy.start_tracking(7)
    assert new.tracking == [7] and old.tracking == []
    # Every part of the node reads the new instance.
    assert node.detector is proxy and node.real_ui is proxy and node.cleanup is proxy
    assert node.real_ui.ui_step() is True          # new.name == 2
    assert node.sim_debug is not None and node.sim_debug.read_snapshot() == 2
    assert node.camera_health.ready() is True
    assert any("failed" in report for report in reports) and any("restarted (try 1)" in r for r in reports)


def test_camera_reads_not_ready_while_a_restart_is_pending() -> None:
    old = _Detector(1)
    failing_new = _Detector(2, start_error=RuntimeError("no camera"))
    factory, clock, reports = _Factory(old, failing_new, _Detector(3)), _Clock(), []
    proxy = _proxy(factory, clock, reports)
    node = stable_node(proxy)
    old.failure = RuntimeError("capture died")
    proxy.raise_if_failed()          # try 1 fails to start
    assert node.camera_health.ready() is False
    clock.now = 1.0                  # second back-off
    proxy.raise_if_failed()
    assert node.camera_health.ready() is True and proxy.instance is factory.created[-1]


def test_tries_back_off_then_escalate_with_the_original_failure() -> None:
    old = _Detector(1)
    original = RuntimeError("capture died")
    old.failure = original
    retries = [_Detector(n, start_error=RuntimeError(f"start {n}")) for n in (2, 3, 4)]
    factory, clock, reports = _Factory(old, *retries), _Clock(), []
    proxy = _proxy(factory, clock, reports)

    proxy.raise_if_failed()          # try 1 at once, fails
    assert len(factory.created) == 2
    clock.now = 0.5
    proxy.raise_if_failed()          # back-off 1 s not over: waits, no raise
    assert len(factory.created) == 2
    clock.now = 1.0
    proxy.raise_if_failed()          # try 2
    clock.now = 4.9
    proxy.raise_if_failed()
    assert len(factory.created) == 3
    clock.now = 5.0
    with pytest.raises(RuntimeError) as raised:
        proxy.raise_if_failed()      # try 3 fails: tries used up, escalate
    assert raised.value is original
    with pytest.raises(RuntimeError):
        proxy.raise_if_failed()      # stays escalated
    assert len(factory.created) == 4


def test_a_restart_that_ran_stably_gets_every_try_again() -> None:
    detectors = [_Detector(n) for n in range(1, 6)]
    factory, clock, reports = _Factory(*detectors), _Clock(), []
    proxy = _proxy(factory, clock, reports)
    for n in range(4):
        proxy.instance.failure = RuntimeError("again")  # type: ignore[attr-defined]
        proxy.raise_if_failed()
        clock.now += 30.0            # COMPONENT_STABLE_S healthy between failures
    assert len(factory.created) == 5


def test_old_instance_not_quiet_escalates_without_a_new_instance() -> None:
    old = _Detector(1, quiet_after_stop=False)
    old.failure = RuntimeError("hung")
    factory, clock, reports = _Factory(old, _Detector(2)), _Clock(), []
    proxy = _proxy(factory, clock, reports)
    with pytest.raises(RuntimeError, match="hung"):
        proxy.raise_if_failed()
    assert len(factory.created) == 1
    assert any("did not stop" in report for report in reports)


def test_new_instance_left_not_quiet_escalates() -> None:
    old = _Detector(1)
    old.failure = RuntimeError("capture died")
    half = _Detector(2, start_error=RuntimeError("half started"), quiet_after_stop=False)
    factory, clock, reports = _Factory(old, half, _Detector(3)), _Clock(), []
    proxy = _proxy(factory, clock, reports)
    with pytest.raises(RuntimeError, match="capture died"):
        proxy.raise_if_failed()
    assert len(factory.created) == 2


def test_base_exception_is_not_restarted() -> None:
    old = _Detector(1)
    old.failure = KeyboardInterrupt()
    factory, clock, reports = _Factory(old, _Detector(2)), _Clock(), []
    proxy = _proxy(factory, clock, reports)
    with pytest.raises(KeyboardInterrupt):
        proxy.raise_if_failed()
    assert len(factory.created) == 1


def test_no_restart_after_the_owner_stopped_it() -> None:
    old = _Detector(1)
    factory, clock, reports = _Factory(old, _Detector(2)), _Clock(), []
    proxy = _proxy(factory, clock, reports)
    proxy.start()
    assert proxy.stop() is True
    old.failure = RuntimeError("late")
    with pytest.raises(RuntimeError, match="late"):
        proxy.raise_if_failed()
    assert len(factory.created) == 1


def test_restart_budget_counts_tries_and_backs_off() -> None:
    clock = _Clock()
    budget = RestartBudget(backoff_s=(0.0, 2.0), stable_s=10.0, clock=clock)
    assert not budget.due()          # nothing failed
    budget.failed()
    assert budget.due()
    assert budget.attempt() == 1
    budget.failed()
    clock.now = 1.9
    assert not budget.due()
    clock.now = 2.0
    assert budget.due() and budget.attempt() == 2
    budget.running()                 # the try's instance started at 2.0
    clock.now = 12.0                 # ran stable_s since: a new failure starts afresh
    budget.failed()
    assert budget.tries == 0 and budget.due()
    budget.attempt()
    budget.failed()
    budget.attempt()
    budget.failed()
    assert budget.exhausted and not budget.due()


def test_proxy_covers_the_whole_detector_fleet_member_surface() -> None:
    from navpy.core.perception import detector_ports

    surface: set[str] = set()
    for port in detector_ports.DetectorFleetMember.__mro__:
        if port.__module__ == detector_ports.__name__:
            surface.update(name for name in vars(port) if not name.startswith("_"))
    missing = sorted(name for name in surface if not hasattr(RestartingDetector, name))
    assert missing == []


# --- Codex review of steps 5b-5c -------------------------------------------------------------------------------


def _double(poi_obj_id: int, *, source: str = "test") -> object:
    """A detector instance reporting one POI with ``poi_obj_id`` as its tracker-local id."""
    from unittest.mock import Mock

    from navpy.core.ports.detect_response import DetectResponse
    from tests.detection_factory import make_detected_poi

    poi = make_detected_poi(obj_id=poi_obj_id)
    detector = Mock()
    detector.source_name = source
    detector.mounts = []
    detector.is_simulation = True
    detector.is_detection_armed = False
    detector.has_source_driven_detection_events = False
    detector.get_detect_data.return_value = DetectResponse([poi], primary_poi=poi)
    detector.drain_detection_events.return_value = []
    detector.open_detection_event_lease.return_value = None
    detector.poi_uses_source_driven_events.return_value = False
    detector.wall_period_for_scheduler_period.side_effect = lambda period: period
    detector.raise_if_failed.return_value = None
    detector.stop.return_value = True
    return detector


def _coordinated(*doubles: object) -> tuple[object, RestartingDetector, list[object]]:
    from unittest.mock import Mock

    from navpy.core.perception.detection_coordinator import DetectionCoordinator

    queue, created = list(doubles), []

    def create() -> VisionDetectorNode:
        detector = queue.pop(0)
        created.append(detector)
        return VisionDetectorNode(detector, detector, None, None, CameraHealth("NavPy", lambda: True))  # type: ignore[arg-type]

    proxy = RestartingDetector(create, lambda message: None, RestartBudget(clock=_Clock()))
    return DetectionCoordinator([proxy], Mock()), proxy, created


def test_a_restarted_detectors_local_ids_are_new_objects_for_the_coordinator() -> None:
    """Finding 1: the new tracker's ids start again; the old POI's task must not name the new object."""
    from navpy.core.ports.detect_request import DetectRequest

    coordinator, proxy, created = _coordinated(_double(1), _double(1))
    old_poi = coordinator.get_detect_data(DetectRequest()).primary_poi
    old_task = old_poi.identity.task_id

    created[0].raise_if_failed.side_effect = RuntimeError("capture died")
    proxy.raise_if_failed()
    new_poi = coordinator.get_detect_data(DetectRequest()).primary_poi

    assert new_poi.identity.source_generation == 1
    assert new_poi.identity.task_id != old_task
    with pytest.raises(RuntimeError, match="replaced instance"):
        coordinator.start_tracking(old_task)
    created[1].start_tracking.assert_not_called()
    assert coordinator.get_zoom_result(old_task) is None
    created[1].get_zoom_result.assert_not_called()
    coordinator.start_tracking(new_poi.identity.task_id)
    created[1].start_tracking.assert_called_once_with(1)


def test_events_and_leases_of_a_restarted_detector_carry_its_generation() -> None:
    from types import SimpleNamespace

    from navpy.core.ports.detection_publication import DetectionPublication
    from navpy.core.ports.detect_request import DetectRequest
    from tests.detection_factory import make_detected_poi

    old, new = _double(1), _double(1)
    drained, leased = make_detected_poi(obj_id=1), make_detected_poi(obj_id=1)
    new.drain_detection_events.return_value = [DetectionPublication((drained,), None, None, "test", False)]
    seen = []
    new.open_detection_event_lease.return_value = SimpleNamespace(
        closed=False,
        dispatch_available=lambda handler: handler(DetectionPublication((leased,), None, None, "test", False)),
    )
    _, proxy, _ = _coordinated(old, new)
    old.raise_if_failed.side_effect = RuntimeError("capture died")
    proxy.raise_if_failed()

    proxy.drain_detection_events(DetectRequest())
    lease = proxy.open_detection_event_lease(DetectRequest())
    lease.dispatch_available(lambda publication: seen.append(publication) or True)

    assert drained.identity.source_generation == 1
    assert leased.identity.source_generation == 1 and len(seen) == 1
    assert proxy.instance_for_generation(1) is new and proxy.instance_for_generation(0) is None


def test_a_stop_that_raises_after_quiescence_does_not_retire_the_instance() -> None:
    """Finding 2: threads quiet but model cleanup failed: no replacement beside it, and it stays owned."""
    old = _Detector(1)
    old.failure = RuntimeError("capture died")
    calls = []

    def stop_with_cleanup_failure() -> bool:
        calls.append(1)
        old.is_quiescent = True
        if len(calls) == 1:
            raise TimeoutError("appearance worker did not close")
        return True

    old.stop = stop_with_cleanup_failure  # type: ignore[method-assign]
    factory, clock, reports = _Factory(old, _Detector(2)), _Clock(), []
    proxy = _proxy(factory, clock, reports)

    with pytest.raises(RuntimeError, match="capture died"):
        proxy.raise_if_failed()
    assert len(factory.created) == 1 and proxy.instance is old
    assert proxy.stop() is True and len(calls) == 2      # the owner's stop retries the old cleanup


def test_a_replacement_that_cannot_be_stopped_becomes_the_owned_instance() -> None:
    """Finding 3: shutdown and quiescence must see the half-started replacement, not the retired original."""
    old = _Detector(1)
    old.failure = RuntimeError("capture died")
    half = _Detector(2, start_error=RuntimeError("model load failed"), quiet_after_stop=False)
    factory, clock, reports = _Factory(old, half, _Detector(3)), _Clock(), []
    proxy = _proxy(factory, clock, reports)

    with pytest.raises(RuntimeError, match="capture died"):
        proxy.raise_if_failed()

    assert proxy.instance is half and len(factory.created) == 2
    assert proxy.is_quiescent is False
    proxy.stop()
    assert half.stopped == 2 and old.stopped == 1


def test_tries_that_are_slow_to_fail_still_run_out() -> None:
    """Finding 4: a startup that takes longer than the healthy time and then fails earns no tries back."""
    clock = _Clock()

    class SlowFailing(_Detector):
        def start(self) -> None:
            clock.now += 31.0
            super().start()

    old = _Detector(1)
    old.failure = RuntimeError("capture died")
    retries = [SlowFailing(n, start_error=RuntimeError("camera open")) for n in (2, 3, 4, 5)]
    factory, reports = _Factory(old, *retries), []
    proxy = _proxy(factory, clock, reports)
    with pytest.raises(RuntimeError, match="capture died"):
        for _ in range(10):
            proxy.raise_if_failed()
            clock.now += 5.0
    assert len(factory.created) == 4     # the original and three tries


def test_restart_tries_are_timed_in_aircraft_time() -> None:
    """Finding 5 (owner ruling Q1): at sim speed-up the back-off passes in aircraft time, not wall time."""
    from types import SimpleNamespace

    from navpy.core.flow.nav_recovery_clock import AircraftTimeClock, attitude_boot_s

    wall = _Clock()
    vehicle = SimpleNamespace(attitude_sample=SimpleNamespace(time_boot_s=100.0))
    budget = RestartBudget(clock=AircraftTimeClock(lambda: attitude_boot_s(vehicle), wall))
    budget.failed()
    budget.attempt()
    budget.failed()                       # try 2 waits 1 s
    wall.now += 0.1                       # speed-up 10: 0.1 s of wall time is 1 s of aircraft time
    vehicle.attitude_sample = SimpleNamespace(time_boot_s=101.0)
    assert budget.due()


def test_restart_reports_reach_the_operator_as_status_texts() -> None:
    """Finding 6: a status text needs ``status``; a destination alone writes only the log."""
    from types import SimpleNamespace
    from unittest.mock import Mock

    from navpy.app.perception.restarting_detector import restarting_detector_node

    from tests.logger.recording_logger import RecordingLogger

    logger = RecordingLogger()
    old = _Detector(1)
    old.failure = RuntimeError("capture died")
    factory = _Factory(old, _Detector(2))
    node = restarting_detector_node(logger, SimpleNamespace(attitude_sample=None))(factory)
    node.detector.raise_if_failed()
    statuses = logger.statuses
    assert statuses and all(isinstance(status, str) and "Detector" in status for status in statuses)
    assert logger.journal.records == [("vision", status) for status in statuses]     # step 8


# --- Codex review round 2 of steps 5b-5c -------------------------------------------------------------------------


def _replaced_mid_command(*doubles: object) -> tuple[object, RestartingDetector, list[object], object]:
    """A coordinator whose detector is replaced by the main thread right after the ``armed``-th read of which
    instance serves: the router reads the source name, then checks the instance, then commands it."""
    from unittest.mock import Mock

    from navpy.app.perception.restarting_detector import DetectorRestart
    from navpy.core.perception.detection_coordinator import DetectionCoordinator

    queue, created = list(doubles), []

    def create() -> VisionDetectorNode:
        detector = queue.pop(0)
        created.append(detector)
        return VisionDetectorNode(detector, detector, None, None, CameraHealth("NavPy", lambda: True))  # type: ignore[arg-type]

    class ReplacedAfterFirstRead(DetectorRestart):
        armed = 0

        @property
        def current(self):  # type: ignore[override]
            value = self._current
            if self.armed:
                self.armed -= 1
            if self.armed == 0 and getattr(self, "_swap_pending", False):
                self._swap_pending = False
                created[-1].raise_if_failed.side_effect = RuntimeError("capture died")
                self.raise_if_failed()       # the main thread's health check swaps the instance now
            return value

        @current.setter
        def current(self, value) -> None:
            self._current = value

    proxy = RestartingDetector.__new__(RestartingDetector)
    proxy._parts = ReplacedAfterFirstRead(create, lambda message: None, RestartBudget(clock=_Clock()))
    return DetectionCoordinator([proxy], Mock()), proxy, created, proxy._parts


def test_a_replacement_between_check_and_command_never_commands_the_new_instance() -> None:
    """Round 2, finding 1: checking the generation and commanding must use the same instance."""
    from navpy.core.ports.detect_request import DetectRequest

    coordinator, proxy, created, parts = _replaced_mid_command(_double(1), _double(1))
    task = coordinator.get_detect_data(DetectRequest()).primary_poi.identity.task_id

    parts.armed, parts._swap_pending = 2, True   # swapped after the source-name read and the instance check
    coordinator.start_tracking(task)       # the instance read by the check, which saw the object, is commanded

    assert len(created) == 2 and proxy.instance is created[1]
    created[1].start_tracking.assert_not_called()
    created[0].start_tracking.assert_called_once_with(1)
    parts.armed, parts._swap_pending = 2, True
    coordinator.get_zoom_result(task)
    created[1].get_zoom_result.assert_not_called()


def test_the_factory_built_detector_times_its_tries_in_aircraft_time() -> None:
    """Round 2, finding 2: the production node maker, not a hand-built budget, uses aircraft time."""
    from types import SimpleNamespace
    from unittest.mock import Mock

    from navpy.app.perception.restarting_detector import restarting_detector_node

    old = _Detector(1)
    old.failure = RuntimeError("capture died")
    factory = _Factory(old, _Detector(2, start_error=RuntimeError("camera open")), _Detector(3))
    vehicle = SimpleNamespace(attitude_sample=SimpleNamespace(time_boot_s=100.0))
    node = restarting_detector_node(Mock(), vehicle, wall_s=lambda: 0.0)(factory)   # no wall time passes

    node.detector.raise_if_failed()         # try 1 at once fails; try 2 waits 1 s
    assert len(factory.created) == 2
    vehicle.attitude_sample = SimpleNamespace(time_boot_s=100.5)
    node.detector.raise_if_failed()         # half a second of aircraft time: still waiting
    assert len(factory.created) == 2
    vehicle.attitude_sample = SimpleNamespace(time_boot_s=101.0)
    node.detector.raise_if_failed()         # 1 s of aircraft time, no wall time: try 2 runs
    assert len(factory.created) == 3 and node.detector.instance is factory.created[2]
