"""A detector restarted as a new instance behind a stable proxy (docs/design/navpy_recovery.md, step 5b).

A detector cannot restart in place ("construct a new Detector"), and the real one owns the camera capture. So a
failed instance is stopped, verified quiet, and replaced by a fresh one from the same factory, with limited tries
and back-off (core/kernel/restart_budget.py). The coordinator, the fleet, NAV and the UI keep the proxy.

The health check (main thread) drives it: ``raise_if_failed`` restarts and raises only when the detector cannot
be restarted (tries used up, or an instance not quiet), which escalates (app/runtime_escalation.py).

A restart does not replay tracking, geo or zoom commands to the new instance: object ids belong to the old one,
and an event lease it opened is closed by its stop. The approach's own checks (lost POI, closed source, command
freshness) end an approach as for lost frames; search detects again on the new instance.
"""

from __future__ import annotations

from collections.abc import Callable

from navpy.app.perception.vision_detector_factory_ports import NodeCreator, VisionDetectorNode
from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.config.logger_args import LogStatusDest
from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.kernel.restart_budget import RestartBudget
from navpy.core.perception.detector_delegate import (
    DetectorControlDelegateFacet,
    DetectorPlatformDelegateFacet,
    DetectorQueryDelegateFacet,
)
from navpy.core.perception.detector_ports import DetectorFleetMember
from navpy.core.ports.camera_health import CameraHealth
from navpy.sim.debug_view.vision_ui_ports import SimDebugSnapshot, SimDebugSource


class DetectorRestart:
    """The instance behind one mount's proxy and its restarts. Lifecycle and health calls come from the main
    thread; NAV reads ``instance`` once per call, so a call in flight during a swap ends on its instance."""

    def __init__(self, create: NodeCreator, report: Callable[[str], None], budget: RestartBudget | None) -> None:
        self._create = create
        self._report = report
        self._budget = RestartBudget() if budget is None else budget
        self._lock = rt.RLock()
        self.node = create()
        self.instance: DetectorFleetMember = self.node.detector
        self._failure: Exception | None = None   # the current instance's failure, while a restart is pending
        self._retired = False                    # the failed instance is stopped and quiet
        self._stopping = False
        self._unquiet = False                    # a failed try left an instance that is not quiet

    def stop(self) -> bool:
        with self._lock:
            self._stopping = True    # no restart after the owner's stop
            return self.instance.stop()

    def raise_if_failed(self) -> None:
        with self._lock:
            if self._stopping:
                self.instance.raise_if_failed()
                return
            if self._failure is None:
                try:
                    self.instance.raise_if_failed()
                    return
                except Exception as failure:  # noqa: BLE001 - restarted; a BaseException is not
                    self._failed(failure)
            self._try_restart()

    def camera_ready(self) -> bool:
        return self._failure is None and self.node.camera_health.ready()

    def read_debug_snapshot(self) -> SimDebugSnapshot:
        sim_debug = self.node.sim_debug
        assert sim_debug is not None
        return sim_debug.read_snapshot()

    def ui_step(self) -> bool:
        real_ui = self.node.real_ui
        return True if real_ui is None else real_ui.ui_step()

    def _failed(self, failure: Exception) -> None:
        self._failure = failure
        self._retired = False
        self._budget.failed()
        self._report(f"Detector '{self._name()}' failed: {failure!r}; restarting it")

    def _try_restart(self) -> None:
        failure = self._failure
        assert failure is not None
        if self._budget.exhausted or self._unquiet:
            raise failure
        if not self._budget.due():
            return
        if not self._retire():
            self._report(f"Detector '{self._name()}' did not stop quietly: not restarted")
            raise failure
        attempt = self._budget.attempt()
        try:
            node = self._start_new()
        except Exception as error:  # noqa: BLE001 - counted as a failed try
            self._budget.failed()
            self._report(f"Detector '{self._name()}' restart {attempt} failed: {error!r}")
            if self._budget.exhausted or self._unquiet:
                raise failure from error
            return
        self.node = node
        self.instance = node.detector
        self._failure = None
        self._report(f"Detector '{self._name()}' restarted (try {attempt})")

    def _retire(self) -> bool:
        if not self._retired:
            try:
                stopped = self.instance.stop()
            except Exception:  # noqa: BLE001 - judged by quiescence below
                stopped = False
            self._retired = stopped is True or _quiet(self.instance)
        return self._retired

    def _start_new(self) -> VisionDetectorNode:
        node = self._create()
        try:
            node.detector.start()
        except BaseException:
            try:
                node.cleanup.stop()
            except Exception:  # noqa: BLE001 - the start failure is the one reported
                pass
            # A half-started instance that is not quiet cannot be left behind: no further try, escalate.
            self._unquiet = not _quiet(node.detector)
            raise
        return node

    def _name(self) -> str:
        try:
            return self.instance.source_name
        except Exception:  # noqa: BLE001 - a name for the report only
            return "?"


class RestartLifecycleDelegateFacet:
    """Lifecycle, health and UI of the proxy: its own restart, not the instance's alone."""

    _parts: DetectorRestart

    @property
    def instance(self) -> DetectorFleetMember:
        """The instance behind the proxy now (inspection only: callers use the proxy)."""
        return self._parts.instance

    def start(self) -> None:
        self._parts.instance.start()

    def stop(self) -> bool:
        return self._parts.stop()

    @property
    def is_quiescent(self) -> bool:
        return self._parts.instance.is_quiescent

    def raise_if_failed(self) -> None:
        self._parts.raise_if_failed()

    def ui_step(self) -> bool:
        return self._parts.ui_step()


class RestartingDetector(
    DetectorQueryDelegateFacet,
    DetectorControlDelegateFacet,
    DetectorPlatformDelegateFacet,
    RestartLifecycleDelegateFacet,
):
    """The stable detector of one mount: one-field facade over its restart."""

    def __init__(
        self,
        create: NodeCreator,
        report: Callable[[str], None],
        budget: RestartBudget | None = None,
    ) -> None:
        self._parts = DetectorRestart(create, report, budget)


def stable_node(detector: RestartingDetector) -> VisionDetectorNode:
    """The node the vision composition keeps: every part of it reads the current instance."""
    parts = detector._parts
    first = parts.node
    return VisionDetectorNode(
        detector=detector,
        cleanup=detector,
        real_ui=None if first.real_ui is None else detector,
        sim_debug=None if first.sim_debug is None else SimDebugSource(read_snapshot=parts.read_debug_snapshot),
        camera_health=CameraHealth(first.camera_health.vendor, parts.camera_ready),
    )


def restarting_detector_node(logger: ILogger) -> Callable[[NodeCreator], VisionDetectorNode]:
    """``build_detector_nodes``' node maker: each mount's detector behind a restarting proxy."""
    def report(message: str) -> None:
        logger.warning(message, key="vision", dest=LogStatusDest.DRONE)

    return lambda create: stable_node(RestartingDetector(create, report))


def _quiet(detector: object) -> bool:
    try:
        return getattr(detector, "is_quiescent") is True
    except Exception:  # noqa: BLE001 - unknown is not quiet
        return False


__all__ = ["DetectorRestart", "RestartingDetector", "restarting_detector_node", "stable_node"]
