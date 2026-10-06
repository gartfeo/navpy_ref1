"""Ordered off-thread execution for post-command diagnostics."""

from __future__ import annotations

import threading
from collections.abc import Callable
from dataclasses import dataclass

from navpy.exception_groups import BaseExceptionGroup
from navpy.logger.cache_logger import ILogger
from navpy.logger.navigation_logger import NAVIGATION_EVIDENCE_FAILURE_MARKER
from navpy.logger.navigation_stream_worker import NavigationStreamWorker


@dataclass(frozen=True)
class PostprocessJob:
    """Run evidence normally, or release ownership on failed submission."""

    execute: Callable[[], None]
    abandon: Callable[[], None]

    def __call__(self) -> None:
        self.execute()


class NavigationPostprocessFailure(RuntimeError):
    """A latched evidence failure surfaced after the FIFO is drained."""


class NavigationPostprocessDispatcher:
    """Own one restartable FIFO worker for non-command-path evidence work."""

    def __init__(self, logger: ILogger) -> None:
        self._logger = logger
        self._lock = threading.RLock()
        self._worker: NavigationStreamWorker[Callable[[], None]] | None = None

    def start(self) -> None:
        with self._lock:
            if self._worker is not None:
                return
            worker = NavigationStreamWorker(self._execute)
            self._worker = worker
            try:
                accepted = worker.submit(_noop)
            except BaseException:
                self._worker = None
                raise
            if not accepted:  # pragma: no cover - a fresh worker accepts
                self._worker = None
                raise RuntimeError("navigation postprocess worker refused startup")

    def submit(self, job: Callable[[], None]) -> None:
        try:
            self.start()
            with self._lock:
                worker = self._worker
                if worker is None:  # pragma: no cover - start owns the worker
                    raise RuntimeError("navigation postprocess worker is unavailable")
                accepted = worker.submit(job)
        except BaseException as submit_error:
            errors: list[BaseException] = [submit_error]
            try:
                if isinstance(job, PostprocessJob):
                    job.abandon()
            except BaseException as abandon_error:
                errors.append(abandon_error)
            try:
                self._logger.error(
                    f"{NAVIGATION_EVIDENCE_FAILURE_MARKER}: postprocess "
                    f"submission {type(submit_error).__name__}: "
                    f"{submit_error}",
                    submit_error,
                )
            except BaseException as logging_error:
                errors.append(logging_error)
            if len(errors) == 1:
                raise errors[0]
            raise BaseExceptionGroup(
                "navigation postprocess submission cleanup failed",
                errors,
            )
        if accepted:
            return
        if isinstance(job, PostprocessJob):
            job.abandon()
        raise RuntimeError("navigation postprocess worker is closing")

    def raise_if_failed(self) -> None:
        with self._lock:
            worker = self._worker
        if worker is None:
            return
        failure = worker.first_error
        if failure is not None:
            raise NavigationPostprocessFailure(
                "terminal command evidence postprocess failed"
            ) from failure

    def close(self) -> None:
        with self._lock:
            worker = self._worker
        if worker is None:
            return
        worker.close()
        failure = worker.first_error
        with self._lock:
            if self._worker is worker:
                self._worker = None
        if failure is not None:
            raise NavigationPostprocessFailure(
                "terminal command evidence postprocess failed"
            ) from failure

    def _execute(self, job: Callable[[], None]) -> None:
        try:
            job()
        except Exception as error:
            try:
                self._logger.error(
                    f"{NAVIGATION_EVIDENCE_FAILURE_MARKER}: terminal postprocess "
                    f"{type(error).__name__}: {error}",
                    error,
                )
            except BaseException as logging_error:
                raise BaseExceptionGroup(
                    "terminal postprocess and failure reporting failed",
                    [error, logging_error],
                )
            raise


def _noop() -> None:
    return None


__all__ = [
    "NavigationPostprocessDispatcher",
    "NavigationPostprocessFailure",
    "PostprocessJob",
]
