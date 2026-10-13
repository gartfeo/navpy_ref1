"""Capture-backend ownership from acquisition through final close."""

from __future__ import annotations

from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.exception_groups import BaseExceptionGroup

from dataclasses import dataclass, field

from navpy.core.kernel.thread_launch import ThreadLaunchGate
from navpy.core.perception.camera.frame_capture_logging import SafeCaptureLogger
from navpy.core.ports.frame_capture_ports import FrameCaptureBackend
from navpy.core.ports.worker_failure import WorkerFailureLatch


def _cleanup_backend(
    backend: FrameCaptureBackend,
    logger: SafeCaptureLogger,
) -> list[BaseException]:
    errors: list[BaseException] = []
    try:
        backend.request_stop()
    except BaseException as error:
        errors.append(error)
        logger.error(f"FrameProvider: startup interrupt failed: {error}")
    try:
        backend.close()
    except BaseException as error:
        errors.append(error)
        logger.error(f"FrameProvider: startup rollback failed: {error}")
    return errors


@dataclass
class CaptureSession:
    backend: FrameCaptureBackend
    stop_event: rt.Event = field(default_factory=rt.Event)
    ready_event: rt.Event = field(default_factory=rt.Event)
    thread: rt.Thread | None = None
    launch: ThreadLaunchGate = field(default_factory=ThreadLaunchGate)
    worker_error: BaseException | None = None
    close_error: BaseException | None = None
    closed: bool = False
    close_lock: rt.Lock = field(default_factory=rt.Lock)

    def rollback(self, logger: SafeCaptureLogger) -> list[BaseException]:
        self.stop_event.set()
        return _cleanup_backend(self.backend, logger)

    def interrupt(self, logger: SafeCaptureLogger) -> bool:
        try:
            return self.backend.request_stop() is True
        except BaseException as error:
            logger.warning(f"FrameProvider: capture interrupt failed: {error}")
            return False

    def request_stop_and_wait(
        self,
        logger: SafeCaptureLogger,
        timeout_s: float,
    ) -> bool:
        """Request quiescence once and retain ownership when it times out."""
        self.stop_event.set()
        self.interrupt(logger)
        thread = self.thread
        if thread is None:
            return True
        if self.launch.cancel_before_commit():
            return True
        if thread is rt.current_thread():
            return False
        if not thread.is_alive():
            return True
        thread.join(timeout=max(0.0, float(timeout_s)))
        if not thread.is_alive():
            return True
        logger.warning(
            "FrameProvider: capture thread did not stop within timeout"
        )
        self.interrupt(logger)
        return False

    def close(self, logger: SafeCaptureLogger) -> bool:
        """Close the backend exactly once, retaining a failure for retry."""
        with self.close_lock:
            if self.closed:
                return True
            try:
                self.backend.close()
            except BaseException as error:
                self.close_error = error
                logger.error(
                    f"FrameProvider: capture backend close failed: {error}"
                )
                return False
            self.closed = True
            self.close_error = None
            return True


def create_capture_session(
    backend: FrameCaptureBackend,
    logger: SafeCaptureLogger,
) -> CaptureSession:
    try:
        return CaptureSession(backend)
    except BaseException as owner_error:
        cleanup_errors = _cleanup_backend(backend, logger)
        if cleanup_errors:
            raise BaseExceptionGroup(
                "capture ownership and rollback failed",
                [owner_error, *cleanup_errors],
            ) from None
        raise


def record_unexpected_return(
    session: CaptureSession,
    failures: WorkerFailureLatch,
    logger: SafeCaptureLogger,
) -> None:
    if session.stop_event.is_set():
        return
    error = RuntimeError("FrameProvider: capture backend stopped unexpectedly")
    session.worker_error = error
    failures.record(error)
    logger.error(str(error))


__all__ = [
    "CaptureSession",
    "create_capture_session",
    "record_unexpected_return",
]
