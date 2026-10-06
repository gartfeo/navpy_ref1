"""Retryable, dependency-ordered shutdown for one simulated detector."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

from navpy.modules.vision.sim.ideal_pose_source import IdealPoseSource
from navpy.modules.vision.sim.sim_coordinator_ports import LifecycleCoordinatorPort
from navpy.modules.vision.sim.sim_detector_loop import SimDetectorWorker


StopAction = tuple[str, Callable[[], None]]


@dataclass(frozen=True)
class SimDetectorStopAttempt:
    complete: bool
    quiescent: bool
    errors: tuple[BaseException, ...]


class SimDetectorStopTransaction:
    """Retain only failed ownership steps and never close a live worker."""

    def __init__(
        self,
        coordinator: LifecycleCoordinatorPort,
        pose_source: IdealPoseSource,
        worker: SimDetectorWorker,
        diagnostic_close: Callable[[], None] | None = None,
    ) -> None:
        self._pending_sources: tuple[StopAction, ...] = (
            ("coordinator", coordinator.stop),
            ("pose source", pose_source.detach),
        )
        self._worker = worker
        self._worker_pending = True
        self._cadence_pending = True
        self._diagnostic_close = diagnostic_close

    @property
    def is_quiescent(self) -> bool:
        return not self._worker_pending

    def run(self, join_timeout_s: float) -> SimDetectorStopAttempt:
        errors: list[BaseException] = []
        pending_sources: list[StopAction] = []
        for label, action in self._pending_sources:
            try:
                action()
            except BaseException as error:
                pending_sources.append((label, action))
                errors.append(error)
        self._pending_sources = tuple(pending_sources)

        if self._worker_pending:
            try:
                joined = self._worker.join(join_timeout_s)
            except BaseException as error:
                errors.append(error)
            else:
                if joined:
                    self._worker_pending = False

        if not self._worker_pending and self._cadence_pending:
            try:
                self._worker.close()
            except BaseException as error:
                errors.append(error)
            else:
                self._cadence_pending = False

        if not self._pending_sources and not self._worker_pending and self._diagnostic_close is not None:
            try:
                self._diagnostic_close()
            except BaseException as error:
                errors.append(error)
            else:
                self._diagnostic_close = None

        complete = (
            not self._pending_sources
            and not self._worker_pending
            and not self._cadence_pending
            and self._diagnostic_close is None
        )
        return SimDetectorStopAttempt(
            complete,
            not self._worker_pending,
            tuple(errors),
        )


__all__ = ["SimDetectorStopAttempt", "SimDetectorStopTransaction"]
