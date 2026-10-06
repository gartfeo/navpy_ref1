"""Execution boundary for one frame-capture backend generation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np

from navpy.modules.vision.frame_capture_logging import SafeCaptureLogger
from navpy.modules.vision.frame_capture_ports import CaptureStamp
from navpy.modules.vision.frame_capture_session import (
    CaptureSession,
    record_unexpected_return,
)
from navpy.modules.vision.worker_failure import WorkerFailureLatch


@dataclass(frozen=True)
class FrameCaptureWorkerPorts:
    publish: Callable[
        [CaptureSession, np.ndarray | None, CaptureStamp | None], None
    ]
    finished: Callable[[CaptureSession], None]


class FrameCaptureWorker:
    """Run a backend and retain any unexpected worker failure."""

    def __init__(
        self,
        logger: SafeCaptureLogger,
        failures: WorkerFailureLatch,
        ports: FrameCaptureWorkerPorts,
    ) -> None:
        self._logger = logger
        self._failures = failures
        self._ports = ports

    def run_generation(self, session: CaptureSession) -> None:
        if not session.launch.enter():
            return
        session.ready_event.wait()
        try:
            session.backend.run(
                lambda frame, capture: self._ports.publish(
                    session, frame, capture
                ),
                lambda: not session.stop_event.is_set(),
            )
            record_unexpected_return(session, self._failures, self._logger)
        except BaseException as error:
            session.worker_error = error
            self._failures.record(error)
            self._logger.error(f"FrameProvider: capture worker failed: {error}")
        finally:
            self._ports.finished(session)


__all__ = ["FrameCaptureWorker", "FrameCaptureWorkerPorts"]
