"""Lifecycle and full reset boundary for one simulated detector."""

from __future__ import annotations

from navpy.exception_groups import BaseExceptionGroup, ExceptionGroup

import threading
from typing import Callable

from navpy.modules.vision.sim.ideal_pose_source import IdealPoseSource
from navpy.modules.vision.sim.sim_coordinator_ports import LifecycleCoordinatorPort
from navpy.modules.vision.sim.sim_detector_loop import SimDetectorWorker
from navpy.modules.vision.sim.sim_detector_reset import SimDetectorReset
from navpy.modules.vision.sim.sim_detector_stop_transaction import (
    SimDetectorStopTransaction,
)
from navpy.modules.vision.sim.sim_runtime_ports import InfoSink
from navpy.modules.vision.sim.sim_source_activation import SimSourceActivation


SIM_DETECTOR_JOIN_TIMEOUT_S = 2.0


def _raise_cleanup_errors(errors: tuple[BaseException, ...]) -> None:
    if len(errors) == 1:
        raise errors[0]
    if errors:
        if all(isinstance(error, Exception) for error in errors):
            raise ExceptionGroup("simulator detector cleanup failed", list(errors))
        raise BaseExceptionGroup(
            "simulator detector cleanup failed",
            list(errors),
        )


def _raise_start_failure(
    start_error: BaseException,
    cleanup_errors: tuple[BaseException, ...],
) -> None:
    failures = [start_error, *cleanup_errors]
    if all(isinstance(error, Exception) for error in failures):
        raise ExceptionGroup(
            "simulator detector start and rollback failed",
            failures,
        ) from None
    raise BaseExceptionGroup(
        "simulator detector start and rollback failed",
        failures,
    ) from None


class SimDetectorLifecycle:
    """Serialize exact-once startup, refresh, rollback, and shutdown."""

    def __init__(
        self,
        *,
        coordinator: LifecycleCoordinatorPort,
        activation: SimSourceActivation,
        pose_source: IdealPoseSource,
        worker: SimDetectorWorker,
        resetter: SimDetectorReset,
        info: InfoSink,
        diagnostic_close: Callable[[], None] | None = None,
    ) -> None:
        self._coordinator = coordinator
        self._activation = activation
        self._pose_source = pose_source
        self._worker = worker
        self._resetter = resetter
        self._info = info
        self._condition = threading.Condition(threading.RLock())
        self._phase = "new"
        self._start_error: BaseException | None = None
        self._stop_errors: tuple[BaseException, ...] = ()
        self._stop_transaction = SimDetectorStopTransaction(
            coordinator,
            pose_source,
            worker,
            diagnostic_close,
        )

    def start(self) -> None:
        with self._condition:
            while self._phase in {"starting", "stopping", "refreshing"}:
                self._condition.wait()
            if self._phase == "running":
                return
            if self._phase in {"failed", "stop_incomplete", "stopped"}:
                raise RuntimeError(
                    "simulator detector cannot restart; construct a new detector"
                ) from self._start_error
            self._phase = "starting"

        self._safe_info("Detector: Start Detection")
        try:
            with self._coordinator.reset() as resetting:
                if not resetting:
                    raise RuntimeError("simulator detector reset was refused")
                self._resetter.prepare()
                if self._activation.source_driven:
                    self._pose_source.start()
                self._activation.activate()
            self._worker.start()
        except BaseException as start_error:
            attempt = self._stop_transaction.run(SIM_DETECTOR_JOIN_TIMEOUT_S)
            cleanup_errors = list(attempt.errors)
            if not attempt.quiescent:
                cleanup_errors.append(TimeoutError(
                    "simulator detector worker did not stop during startup rollback"
                ))
            with self._condition:
                self._start_error = start_error
                self._stop_errors = tuple(attempt.errors)
                self._phase = "failed" if attempt.complete else "stop_incomplete"
                self._condition.notify_all()
            if cleanup_errors:
                _raise_start_failure(start_error, tuple(cleanup_errors))
            raise

        with self._condition:
            self._phase = "running"
            self._condition.notify_all()

    @property
    def is_quiescent(self) -> bool:
        with self._condition:
            return self._stop_transaction.is_quiescent

    def stop(self) -> bool:
        with self._condition:
            while self._phase in {"starting", "refreshing"}:
                self._condition.wait()
            if self._phase == "stopping":
                while self._phase == "stopping":
                    self._condition.wait()
            if self._phase == "stopped":
                _raise_cleanup_errors(self._stop_errors)
                return True
            if self._phase == "failed":
                self._phase = "stopped"
                self._condition.notify_all()
                _raise_cleanup_errors(self._stop_errors)
                return True
            self._phase = "stopping"

        attempt = self._stop_transaction.run(SIM_DETECTOR_JOIN_TIMEOUT_S)
        with self._condition:
            self._stop_errors = attempt.errors
            self._phase = "stopped" if attempt.complete else "stop_incomplete"
            self._condition.notify_all()
        _raise_cleanup_errors(attempt.errors)
        return attempt.quiescent

    def refresh(self) -> None:
        with self._condition:
            while self._phase in {"starting", "stopping", "refreshing"}:
                self._condition.wait()
            if self._phase in {"failed", "stop_incomplete", "stopped"}:
                return
            previous_phase = self._phase
            self._phase = "refreshing"
        try:
            with self._coordinator.reset() as resetting:
                if resetting:
                    self._resetter.refresh()
        finally:
            with self._condition:
                self._phase = previous_phase
                self._condition.notify_all()

    def _safe_info(self, message: str) -> None:
        try:
            self._info(message)
        except Exception:
            pass

    def raise_if_failed(self) -> None:
        self._worker.raise_if_failed()


__all__ = ["SimDetectorLifecycle"]
