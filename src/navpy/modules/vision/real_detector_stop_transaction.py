"""Retryable dependency barrier for real-detector shutdown."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Sequence

from navpy.logger.cache_logger import ILogger
from navpy.modules.vision.frame_provider import FrameProvider
from navpy.modules.vision.real_detector_diagnostics import DetectorDiagnostics
from navpy.modules.vision.real_detector_resources import DetectorResources
from navpy.modules.vision.real_detector_state import DetectorRunState
from navpy.modules.vision.real_detector_worker_shutdown import (
    WorkerThread,
    wait_for_worker_quiescence,
)


@dataclass(frozen=True)
class DetectorStopAttempt:
    complete: bool
    quiescent: bool
    live_threads: tuple[WorkerThread, ...]
    errors: tuple[BaseException, ...]


class DetectorStopTransaction:
    """Quiesce workers/frame source before retiring model dependencies."""

    def __init__(
        self,
        run_state: DetectorRunState,
        frame_provider: FrameProvider,
        resources: DetectorResources,
        diagnostics: DetectorDiagnostics,
        logger: ILogger,
    ) -> None:
        self._run_state = run_state
        self._frame_provider = frame_provider
        self._resources = resources
        self._diagnostics = diagnostics
        self._logger = logger
        self._stop_signal_pending = True
        self._quiescent = False
        self._pending_retirement: tuple[Callable[[], None], ...] = (
            resources.close,
            diagnostics.close_ui,
        )
        self._mark_stopped_pending = True

    @property
    def is_quiescent(self) -> bool:
        return self._quiescent

    @property
    def is_complete(self) -> bool:
        return (
            self._quiescent
            and not self._stop_signal_pending
            and not self._pending_retirement
            and not self._mark_stopped_pending
        )

    def quiesce(
        self,
        threads: Sequence[WorkerThread],
        context: str,
        timeout_s: float,
        *,
        stop_frame_provider: bool,
    ) -> DetectorStopAttempt:
        errors: list[BaseException] = []
        if self._stop_signal_pending:
            try:
                self._run_state.request_stop()
            except BaseException as error:
                errors.append(error)
            else:
                self._stop_signal_pending = False
        workers = wait_for_worker_quiescence(
            threads,
            context,
            self._logger.warning,
            timeout_s,
        )
        errors.extend(workers.errors)
        if not workers.is_complete:
            return DetectorStopAttempt(
                False,
                False,
                workers.alive,
                tuple(errors),
            )
        if not stop_frame_provider:
            self._quiescent = True
            return DetectorStopAttempt(
                not self._stop_signal_pending,
                True,
                (),
                tuple(errors),
            )
        try:
            provider_stopped = self._frame_provider.stop()
        except BaseException as error:
            errors.append(error)
            return DetectorStopAttempt(False, False, (), tuple(errors))
        if provider_stopped is False:
            errors.append(TimeoutError(
                f"Detector frame provider did not stop during {context}"
            ))
            return DetectorStopAttempt(False, False, (), tuple(errors))
        self._quiescent = True
        return DetectorStopAttempt(
            not self._stop_signal_pending,
            True,
            (),
            tuple(errors),
        )

    def retire_resources(self) -> tuple[BaseException, ...]:
        errors: list[BaseException] = []
        pending: list[Callable[[], None]] = []
        for action in self._pending_retirement:
            try:
                action()
            except BaseException as error:
                errors.append(error)
                pending.append(action)
        self._pending_retirement = tuple(pending)
        if not pending and self._mark_stopped_pending:
            try:
                self._run_state.mark_resources_stopped()
            except BaseException as error:
                errors.append(error)
            else:
                self._mark_stopped_pending = False
        return tuple(errors)

    def run(
        self,
        threads: Sequence[WorkerThread],
        timeout_s: float,
    ) -> DetectorStopAttempt:
        if not self._quiescent:
            attempt = self.quiesce(
                threads,
                "stop",
                timeout_s,
                stop_frame_provider=True,
            )
            if not attempt.quiescent:
                return attempt
            errors = attempt.errors
        else:
            errors_list: list[BaseException] = []
            if self._stop_signal_pending:
                try:
                    self._run_state.request_stop()
                except BaseException as error:
                    errors_list.append(error)
                else:
                    self._stop_signal_pending = False
            errors = tuple(errors_list)
        retirement_errors = self.retire_resources()
        return DetectorStopAttempt(
            self.is_complete,
            True,
            (),
            errors + retirement_errors,
        )


__all__ = ["DetectorStopAttempt", "DetectorStopTransaction"]
