"""Ordered off-thread execution for post-command diagnostics.

A failed job is logged with the evidence marker and skipped; later jobs still
run and NavPy keeps flying (todo 290). The first failure is kept so callers
that stop on a failed write (bench scripts) can still raise it.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.exception_groups import BaseExceptionGroup
from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.approach.logs.evidence_failure import report_evidence_failure
from navpy.core.approach.logs.navigation_stream_worker import NavigationStreamWorker


@dataclass(frozen=True)
class PostprocessJob:
    """Run evidence normally, or release ownership on failed submission."""

    execute: Callable[[], None]
    abandon: Callable[[], None]

    def __call__(self) -> None:
        self.execute()


class NavigationPostprocessFailure(RuntimeError):
    """The first failed evidence job, raised only to callers that ask."""


class NavigationPostprocessDispatcher:
    """Own one restartable FIFO worker for non-command-path evidence work."""

    def __init__(self, logger: ILogger) -> None:
        self._logger = logger
        self._lock = rt.RLock()
        self._worker: NavigationStreamWorker[Callable[[], None]] | None = None
        self._failure: BaseException | None = None

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
            self._remember(submit_error)
            try:
                if isinstance(job, PostprocessJob):
                    job.abandon()
            except BaseException as abandon_error:
                errors.append(abandon_error)
            report_evidence_failure(self._logger, "postprocess submission", submit_error)
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
        """Raise the first failed job; it stays recorded after ``close``."""
        with self._lock:
            failure = self._failure
        if failure is not None:
            raise NavigationPostprocessFailure(
                "final-approach command evidence postprocess failed"
            ) from failure

    def close(self) -> None:
        with self._lock:
            worker = self._worker
        if worker is None:
            return
        worker.close()
        with self._lock:
            if self._worker is worker:
                self._worker = None

    def _execute(self, job: Callable[[], None]) -> None:
        try:
            job()
        except Exception as error:
            self._remember(error)
            report_evidence_failure(self._logger, "final-approach postprocess", error)

    def _remember(self, error: BaseException) -> None:
        with self._lock:
            if self._failure is None:
                self._failure = error


def _noop() -> None:
    return None


__all__ = [
    "NavigationPostprocessDispatcher",
    "NavigationPostprocessFailure",
    "PostprocessJob",
]
