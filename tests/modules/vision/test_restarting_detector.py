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
    assert any("did not stop quietly" in report for report in reports)


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
    clock.now = 12.0                 # ran stable_s: a new failure starts afresh
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
