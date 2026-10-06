"""Real-detector pose, frame, and worker startup transaction."""

from __future__ import annotations

import threading
from dataclasses import dataclass

import numpy as np

from navpy.modules.common.thread_launch import ThreadLaunchGate
from navpy.modules.vehicle.pose_streams import pose_frame_association_max_skew_s
from navpy.modules.vision.frame_provider import FrameProvider
from navpy.modules.vision.real_detector_ports import (
    PoseRateResolver,
    PoseStreamRequest,
    WorkerLoop,
)
from navpy.modules.vision.real_detector_state import DetectorRunState
from navpy.modules.vision.real_detector_stop_transaction import (
    DetectorStopTransaction,
)
from navpy.modules.vision.real_detector_worker_shutdown import WorkerThread
from navpy.modules.vision.real_frame_association import FrameAssociationBuilder
from navpy.modules.vision.worker_failure import WorkerFailureLatch


class PoseStreamRequester:
    """Request pose cadence and configure the association acceptance bound."""

    def __init__(
        self,
        resolve_rate_hz: PoseRateResolver,
        request_streams: PoseStreamRequest,
        association_builder: FrameAssociationBuilder,
    ) -> None:
        self._resolve_rate_hz = resolve_rate_hz
        self._request_streams = request_streams
        self._association_builder = association_builder
        self._rate_hz: float | None = None
        self._maximum_skew_s: float | None = None

    @property
    def rate_hz(self) -> float | None:
        return self._rate_hz

    @property
    def maximum_skew_s(self) -> float | None:
        return self._maximum_skew_s

    def request(self) -> None:
        rate_hz = self._resolve_rate_hz()
        maximum_skew_s = pose_frame_association_max_skew_s(rate_hz)
        self._association_builder.set_maximum_skew(maximum_skew_s)
        self._rate_hz = rate_hz
        self._maximum_skew_s = maximum_skew_s
        self._request_streams(rate_hz)


@dataclass(frozen=True)
class DetectorWorkers:
    detection: WorkerLoop
    tracking: WorkerLoop
    deep_search: WorkerLoop | None


@dataclass(frozen=True)
class DetectorStartupOutcome:
    threads: tuple[WorkerThread, ...]
    error: BaseException | None = None
    cleanup_errors: tuple[BaseException, ...] = ()
    cleanup_complete: bool = True


class DetectorStartup:
    """Launch workers and roll back every ambiguously started generation."""

    def __init__(
        self,
        run_state: DetectorRunState,
        frame_provider: FrameProvider,
        workers: DetectorWorkers,
        pose_streams: PoseStreamRequester,
        stop_transaction: DetectorStopTransaction,
    ) -> None:
        self._run_state = run_state
        self._frame_provider = frame_provider
        self._workers = workers
        self._pose_streams = pose_streams
        self._stop_transaction = stop_transaction
        self._failures = WorkerFailureLatch()

    def begin(self) -> bool:
        return self._run_state.begin()

    def run(self, join_timeout_s: float) -> DetectorStartupOutcome:
        frame_start_attempted = False
        started: list[WorkerThread] = []
        try:
            self._pose_streams.request()
            frame_start_attempted = True
            self._frame_provider.start()
            threads = self._build_threads()
            for item in threads:
                started.append(item)
                item.thread.start()
        except BaseException as error:
            attempt = self._stop_transaction.quiesce(
                started,
                "startup rollback",
                join_timeout_s,
                stop_frame_provider=frame_start_attempted,
            )
            cleanup_errors = list(attempt.errors)
            if attempt.quiescent:
                cleanup_errors.extend(self._stop_transaction.retire_resources())
            return DetectorStartupOutcome(
                attempt.live_threads,
                error,
                tuple(cleanup_errors),
                self._stop_transaction.is_complete,
            )
        return DetectorStartupOutcome(threads)

    def push_frame(self, frame: np.ndarray | None) -> None:
        self._frame_provider.push_frame(frame)

    def raise_if_failed(self) -> None:
        self._frame_provider.raise_if_failed()
        self._failures.raise_if_failed()

    def _build_threads(self) -> tuple[WorkerThread, ...]:
        workers = [
            ("detect", self._workers.detection),
            ("track", self._workers.tracking),
        ]
        if self._workers.deep_search is not None:
            workers.append(("deep-search", self._workers.deep_search))
        return tuple(
            WorkerThread(
                name,
                threading.Thread(
                    target=self._run_worker,
                    args=(worker, launch),
                    daemon=True,
                ),
                launch,
            )
            for name, worker in workers
            for launch in (ThreadLaunchGate(),)
        )

    def _run_worker(
        self,
        worker: WorkerLoop,
        launch: ThreadLaunchGate,
    ) -> None:
        if not launch.enter():
            return
        self._failures.run(worker.run, self._run_state.request_stop)


__all__ = [
    "DetectorStartup",
    "DetectorStartupOutcome",
    "DetectorWorkers",
    "PoseStreamRequester",
]
