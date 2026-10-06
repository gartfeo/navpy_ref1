"""Frame-source selection and capture-thread lifecycle."""

from __future__ import annotations

from navpy.exception_groups import BaseExceptionGroup

import threading
from dataclasses import dataclass

from navpy.modules.vision.frame_capture_factory import open_capture_backend
from navpy.modules.vision.frame_capture_logging import SafeCaptureLogger
from navpy.modules.vision.frame_capture_ports import (
    CaptureLogger,
)
from navpy.modules.vision.frame_capture_session import (
    CaptureSession,
    create_capture_session,
)
from navpy.modules.vision.frame_capture_state import FrameCaptureState
from navpy.modules.vision.frame_capture_worker import (
    FrameCaptureWorker,
    FrameCaptureWorkerPorts,
)
from navpy.modules.vision.frame_publication import FramePublicationStore
from navpy.modules.vision.worker_failure import WorkerFailureLatch

_STOP_WARNING_S = 2.0


class FrameCaptureRuntime:
    """Select one backend and own its capture-thread lifecycle."""

    def __init__(
        self,
        source: int | str | None,
        logger: CaptureLogger,
        publications: FramePublicationStore,
    ) -> None:
        self._source = source
        self._logger = SafeCaptureLogger(logger)
        self._state = FrameCaptureState(publications)
        self._worker_failures = WorkerFailureLatch()
        self._worker = FrameCaptureWorker(
            self._logger,
            self._worker_failures,
            FrameCaptureWorkerPorts(
                publish=self._state.publish_if_current,
                finished=self._finish_session,
            ),
        )

    @property
    def is_running(self) -> bool:
        return self._state.is_running

    @property
    def has_capture_thread(self) -> bool:
        return self._state.has_session

    def start(self) -> None:
        if not self._state.begin_start():
            return
        if self._source is None:
            self._state.complete_push_start()
            self._logger.info("FrameProvider: push mode (no capture thread)")
            return
        try:
            backend = open_capture_backend(self._source, self._logger)
            session = None if backend is None else create_capture_session(backend, self._logger)
        except BaseException:
            self._finish_failed_start()
            raise
        if session is None:
            self._finish_failed_start()
            message = (
                "FrameProvider: failed to open configured source: "
                f"{self._source}"
            )
            self._logger.error(message)
            raise RuntimeError(message)
        self._state.install_starting(session)
        try:
            thread = threading.Thread(
                target=self._worker.run_generation,
                args=(session,),
                daemon=True,
            )
            session.thread = thread
            thread.start()
        except BaseException as start_error:
            if not session.launch.cancel_before_commit():
                session.stop_event.set()
                session.ready_event.set()
                session.interrupt(self._logger)
                self._state.retain_entered_start_failure(session)
                raise
            cleanup_errors = self._rollback_unstarted_session(session)
            if cleanup_errors:
                raise BaseExceptionGroup(
                    "capture startup rollback failed", [start_error, *cleanup_errors]
                ) from None
            raise
        self._state.commit_start(session)

    def stop(self) -> bool:
        try:
            session = self._state.claim_teardown()
            if session is None:
                return True
            return self._stop_session(session)
        finally:
            self._state.release_teardown_claim()

    def _stop_session(self, session: CaptureSession) -> bool:
        if not session.request_stop_and_wait(self._logger, _STOP_WARNING_S):
            return False
        still_owned = self._state.owns(session)
        if still_owned and not self._finalize_session(session):
            error = session.close_error
            if error is not None:
                raise error
            raise RuntimeError("capture backend did not close")
        return True

    def _finish_session(self, session: CaptureSession) -> None:
        self._state.mark_worker_stopping(session)
        self._finalize_session(session)

    def _finalize_session(self, session: CaptureSession) -> bool:
        if not session.close(self._logger):
            return False
        self._state.retire(session)
        return True

    def _finish_failed_start(self) -> None:
        self._state.fail_start()

    def _rollback_unstarted_session(self, session: CaptureSession) -> list[BaseException]:
        errors = session.rollback(self._logger)
        self._state.fail_start(session)
        return errors

    def raise_if_failed(self) -> None:
        self._worker_failures.raise_if_failed()


@dataclass(frozen=True)
class FrameProviderParts:
    capture: FrameCaptureRuntime
    publications: FramePublicationStore


def build_frame_provider_parts(
    source: int | str | None,
    logger: CaptureLogger,
) -> FrameProviderParts:
    publications = FramePublicationStore()
    return FrameProviderParts(
        capture=FrameCaptureRuntime(source, logger, publications),
        publications=publications,
    )


__all__ = [
    "FrameCaptureRuntime",
    "FrameProviderParts",
    "build_frame_provider_parts",
]
