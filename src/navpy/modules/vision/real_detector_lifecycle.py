"""Lifecycle, pose-stream setup, threads, and resource release."""

from __future__ import annotations

from navpy.exception_groups import BaseExceptionGroup, ExceptionGroup

import threading

import numpy as np

from navpy.logger.cache_logger import ILogger
from navpy.modules.vision.frame_provider import FrameProvider
from navpy.modules.vision.real_detector_diagnostics import DetectorDiagnostics
from navpy.modules.vision.real_detector_ports import (
    LifecycleLog,
)
from navpy.modules.vision.real_detector_resources import DetectorResources
from navpy.modules.vision.real_detector_state import DetectorRunState
from navpy.modules.vision.real_detector_startup import (
    DetectorStartup,
    DetectorWorkers,
    PoseStreamRequester,
)
from navpy.modules.vision.real_detector_stop_transaction import (
    DetectorStopTransaction,
)
from navpy.modules.vision.real_detector_worker_shutdown import WorkerThread

WORKER_JOIN_WARNING_S = 1.0


def _raise_cleanup_errors(
    message: str,
    errors: tuple[BaseException, ...],
) -> None:
    if len(errors) == 1:
        raise errors[0]
    if errors:
        if all(isinstance(error, Exception) for error in errors):
            raise ExceptionGroup(message, list(errors))
        raise BaseExceptionGroup(message, list(errors))


class DetectorLifecycle:
    """Serialize lifecycle phases and preserve cleanup transaction failures."""

    def __init__(
        self,
        run_state: DetectorRunState,
        frame_provider: FrameProvider,
        workers: DetectorWorkers,
        resources: DetectorResources,
        pose_streams: PoseStreamRequester,
        diagnostics: DetectorDiagnostics,
        logger: ILogger,
    ) -> None:
        self._logger = logger
        self._stop_transaction = DetectorStopTransaction(
            run_state,
            frame_provider,
            resources,
            diagnostics,
            logger,
        )
        self._startup = DetectorStartup(
            run_state,
            frame_provider,
            workers,
            pose_streams,
            self._stop_transaction,
        )
        self._threads: tuple[WorkerThread, ...] = ()
        self._condition = threading.Condition(threading.RLock())
        self._phase = "new"
        self._start_error: BaseException | None = None
        self._stop_errors: tuple[BaseException, ...] = ()

    def start(self) -> None:
        with self._condition:
            while self._phase in {"starting", "stopping"}:
                self._condition.wait()
            if self._phase == "running":
                return
            if self._phase in {"failed", "stop_incomplete", "stopped"}:
                raise RuntimeError(
                    "Detector cannot be restarted; construct a new Detector"
                ) from self._start_error
            if not self._startup.begin():
                self._phase = "running"
                return
            self._phase = "starting"

        outcome = self._startup.run(WORKER_JOIN_WARNING_S)
        if outcome.error is not None:
            with self._condition:
                self._threads = outcome.threads
                self._start_error = outcome.error
                # Startup already reports every rollback error to its caller.
                # Retain only errors whose cleanup action is still pending;
                # otherwise a later idempotent stop would replay history
                # forever even though the transaction is complete.
                self._stop_errors = (
                    outcome.cleanup_errors if not outcome.cleanup_complete else ()
                )
                self._phase = (
                    "failed" if outcome.cleanup_complete else "stop_incomplete"
                )
                self._condition.notify_all()
            if outcome.cleanup_errors:
                failures = [outcome.error, *outcome.cleanup_errors]
                if all(isinstance(error, Exception) for error in failures):
                    raise ExceptionGroup(
                        "detector startup and rollback failed",
                        failures,
                    ) from None
                raise BaseExceptionGroup(
                    "detector startup and rollback failed",
                        failures,
                    ) from None
            raise outcome.error

        with self._condition:
            self._threads = outcome.threads
            self._phase = "running"
            self._condition.notify_all()
        self._safe_log(self._logger.info, "Detector: Started (detect+track threads)")

    @property
    def is_quiescent(self) -> bool:
        return self._stop_transaction.is_quiescent

    def stop(self) -> bool:
        with self._condition:
            while self._phase == "starting":
                self._condition.wait()
            if self._phase == "stopping":
                while self._phase == "stopping":
                    self._condition.wait()
            if self._phase == "stopped":
                errors = self._stop_errors
                _raise_cleanup_errors("detector cleanup failed", errors)
                return True
            if self._phase == "failed":
                errors = self._stop_errors
                self._phase = "stopped"
                self._condition.notify_all()
                _raise_cleanup_errors("detector cleanup failed", errors)
                return True
            self._phase = "stopping"
            threads = self._threads

        attempt = self._stop_transaction.run(
            threads,
            WORKER_JOIN_WARNING_S,
        )
        if not attempt.complete:
            with self._condition:
                self._threads = attempt.live_threads
                self._stop_errors = attempt.errors
                self._phase = "stop_incomplete"
                self._condition.notify_all()
            _raise_cleanup_errors("detector cleanup failed", self._stop_errors)
            return attempt.quiescent

        try:
            errors = attempt.errors
        finally:
            with self._condition:
                self._threads = ()
                self._stop_errors = ()
                self._phase = "stopped"
                self._condition.notify_all()
        self._safe_log(self._logger.info, "Detector: Stopped")
        _raise_cleanup_errors("detector cleanup failed", errors)
        return True

    @staticmethod
    def _safe_log(log: LifecycleLog, message: str) -> None:
        try:
            log(message)
        except Exception:
            pass

    def push_frame(self, frame: np.ndarray | None) -> None:
        self._startup.push_frame(frame)

    def raise_if_failed(self) -> None:
        self._startup.raise_if_failed()


__all__ = [
    "DetectorLifecycle",
    "DetectorResources",
    "DetectorWorkers",
    "PoseStreamRequester",
]
