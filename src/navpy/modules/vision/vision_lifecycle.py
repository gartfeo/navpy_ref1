"""Transactional lifecycle owner for the composed vision system."""

from __future__ import annotations

from navpy.exception_groups import BaseExceptionGroup, ExceptionGroup
from navpy.modules.vision.vision_component_lifecycle import (
    VisionComponentLifecycle,
    VisionCoordinatorLifecyclePort,
    VisionMountLifecyclePort,
    VisionPublisherLifecyclePort,
    VisionQuiescencePort,
)
from navpy.modules.vision.vision_debug_owner import (
    VisionDebugClosePort,
    VisionDebugOwner,
)
from navpy.modules.vision.vision_stop_transaction import (
    VisionStopTimeout,
    VisionStopTransaction,
)

import threading
from typing import NoReturn, Optional, Protocol, Sequence


def _raise_start_failure(
    start_error: BaseException,
    cleanup_errors: Sequence[BaseException],
) -> NoReturn:
    errors = [start_error, *cleanup_errors]
    if all(isinstance(error, Exception) for error in errors):
        raise ExceptionGroup("vision startup and rollback failed", errors)
    raise BaseExceptionGroup("vision startup and rollback failed", errors)


def _raise_stop_failures(errors: Sequence[BaseException]) -> None:
    failures = [
        error for error in errors if not isinstance(error, VisionStopTimeout)
    ]
    if not failures:
        return
    if len(failures) == 1:
        raise failures[0]
    if all(isinstance(error, Exception) for error in failures):
        raise ExceptionGroup("vision shutdown failed", failures)
    raise BaseExceptionGroup("vision shutdown failed", failures)


class VisionErrorSink(Protocol):
    def __call__(self, message: str) -> None: ...


class VisionLifecycle:
    """Order lifecycle work once while retaining every cleanup failure."""

    def __init__(
        self,
        mounts: Sequence[VisionMountLifecyclePort],
        publisher: VisionPublisherLifecyclePort,
        coordinator: VisionCoordinatorLifecyclePort,
        debug: Optional[VisionDebugClosePort],
        error: VisionErrorSink,
    ) -> None:
        self._components = VisionComponentLifecycle(
            mounts,
            publisher,
            coordinator,
        )
        self._error = error
        self._debug_owner = VisionDebugOwner(debug, self._safe_error)
        self._condition = threading.Condition(threading.RLock())
        self._phase = "new"
        self._start_error: BaseException | None = None
        self._stop_result = True
        self._stop_errors: tuple[BaseException, ...] = ()
        self._stop_transaction: VisionStopTransaction | None = None

    def start(self) -> None:
        with self._condition:
            while self._phase in {"starting", "stopping", "refreshing"}:
                self._condition.wait()
            if self._phase == "running":
                return
            if self._phase in {"failed", "stop_incomplete", "stopped"}:
                raise RuntimeError(
                    "vision startup previously failed; reconstruct the controller"
                ) from self._start_error
            self._phase = "starting"
            self._stop_result = False

        failure = self._components.start(self._safe_error)
        if failure is not None:
            rollback_outcome = failure.outcome
            startup_cleanup_errors = list(rollback_outcome.errors)
            debug_error = self._debug_owner.close()
            if debug_error is not None:
                startup_cleanup_errors.append(debug_error)
            with self._condition:
                self._start_error = failure.error
                self._stop_result = rollback_outcome.quiescent
                self._stop_errors = tuple(startup_cleanup_errors)
                self._stop_transaction = (
                    None
                    if rollback_outcome.complete and self._debug_owner.is_closed
                    else failure.rollback
                )
                self._phase = "failed"
                self._condition.notify_all()
            if startup_cleanup_errors:
                _raise_start_failure(failure.error, startup_cleanup_errors)
            raise failure.error

        with self._condition:
            self._phase = "running"
            self._condition.notify_all()

    def stop(self) -> bool:
        with self._condition:
            while self._phase in {"starting", "refreshing"}:
                self._condition.wait()
            if self._phase == "stopping":
                while self._phase == "stopping":
                    self._condition.wait()
                result = self._stop_result
                errors = self._stop_errors
                _raise_stop_failures(errors)
                return result
            if self._phase == "stopped":
                result = self._stop_result
                errors = self._stop_errors
                _raise_stop_failures(errors)
                return result
            if self._phase == "failed" and self._stop_transaction is None:
                self._phase = "stopped"
                self._condition.notify_all()
                result = self._stop_result
                errors = self._stop_errors
                _raise_stop_failures(errors)
                return result
            if self._stop_transaction is None:
                self._stop_transaction = self._components.create_stop_transaction(
                    self._safe_error
                )
            transaction = self._stop_transaction
            self._phase = "stopping"

        outcome = transaction.run()
        errors = list(outcome.errors)
        debug_error = self._debug_owner.close()
        if debug_error is not None:
            errors.append(debug_error)
        complete = outcome.complete and self._debug_owner.is_closed
        with self._condition:
            self._stop_result = outcome.quiescent
            self._stop_errors = tuple(errors)
            if complete:
                self._stop_transaction = None
            self._phase = "stopped" if complete else "stop_incomplete"
            self._condition.notify_all()
        _raise_stop_failures(errors)
        return outcome.quiescent

    @property
    def is_quiescent(self) -> bool:
        with self._condition:
            return self._stop_result

    def refresh(self) -> None:
        with self._condition:
            while self._phase in {"starting", "stopping", "refreshing"}:
                self._condition.wait()
            if self._phase in {"failed", "stop_incomplete", "stopped"}:
                return
            previous_phase = self._phase
            self._phase = "refreshing"
        try:
            self._components.refresh()
        finally:
            with self._condition:
                self._phase = previous_phase
                self._condition.notify_all()

    def raise_if_failed(self) -> None:
        self._components.raise_if_failed()

    def _safe_error(self, message: str) -> None:
        try:
            self._error(message)
        except BaseException:
            pass


__all__ = [
    "VisionCoordinatorLifecyclePort",
    "VisionDebugClosePort",
    "VisionErrorSink",
    "VisionLifecycle",
    "VisionMountLifecyclePort",
    "VisionPublisherLifecyclePort",
    "VisionQuiescencePort",
]
