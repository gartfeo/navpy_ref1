"""Retryable worker-thread ownership for simulator detection."""

from __future__ import annotations


from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.thread_launch import ThreadLaunchGate
from navpy.sim.detection.ideal_pose_source import IdealPoseSample
from navpy.sim.detection.scheduler_cadence_lease import SchedulerCadenceLease
from navpy.sim.detection.sim_coordinator_ports import WorkerCoordinatorPort
from navpy.sim.detection.sim_detector_execution import SimDetectorExecution
from navpy.sim.detection.sim_polling_pose_reader import PollingPoseReaderPort
from navpy.sim.detection.sim_runtime_ports import (
    DetectFramePort,
    ErrorSink,
    FrameOutcomeSink,
    SourceTimeAdvance,
)
from navpy.sim.detection.unpaired_pose_monitor import UnpairedPoseMonitor
from navpy.core.ports.worker_failure import WorkerFailureLatch


class SimDetectorWorker:
    """Own one detector execution thread and its persistent health latch."""

    def __init__(
        self,
        *,
        coordinator: WorkerCoordinatorPort[IdealPoseSample],
        polling_pose_reader: PollingPoseReaderPort,
        advance_source_time: SourceTimeAdvance,
        unpaired_pose_monitor: UnpairedPoseMonitor,
        error: ErrorSink,
        cadence_lease: SchedulerCadenceLease,
        scheduler_period_s: float,
        ideal_360: bool,
        detect_pois: DetectFramePort,
        record_outcome: FrameOutcomeSink,
    ) -> None:
        self._execution = SimDetectorExecution(
            coordinator=coordinator,
            polling_pose_reader=polling_pose_reader,
            advance_source_time=advance_source_time,
            unpaired_pose_monitor=unpaired_pose_monitor,
            error=error,
            cadence_lease=cadence_lease,
            scheduler_period_s=scheduler_period_s,
            ideal_360=ideal_360,
            detect_pois=detect_pois,
            record_outcome=record_outcome,
        )
        self._coordinator = coordinator
        self._lifecycle_lock = rt.RLock()
        self._thread: rt.Thread | None = None
        self._launch: ThreadLaunchGate | None = None
        self._started = False
        self._failures = WorkerFailureLatch()

    def start(self) -> None:
        with self._lifecycle_lock:
            if self._coordinator.stop_event.is_set() or self._started:
                return
            launch = ThreadLaunchGate()
            self._thread = rt.Thread(
                target=self._run_generation,
                args=(launch,),
                name="DetectorSimWorker",
                daemon=True,
            )
            self._launch = launch
            self._started = True
            try:
                self._thread.start()
            except BaseException:
                if launch.cancel_before_commit():
                    self._thread = None
                    self._launch = None
                    self._started = False
                raise

    def join(self, timeout_s: float) -> bool:
        with self._lifecycle_lock:
            thread = self._thread
            launch = self._launch
            if (
                thread is not None
                and launch is not None
                and launch.cancel_before_commit()
            ):
                self._retire(thread)
                return True
        if thread is None:
            return True
        if thread is rt.current_thread():
            return False
        if thread.is_alive():
            thread.join(timeout=max(0.0, float(timeout_s)))
        if thread.is_alive():
            return False
        self._retire(thread)
        return True

    def close(self) -> None:
        self._execution.close()

    def raise_if_failed(self) -> None:
        self._failures.raise_if_failed()

    def run(self) -> None:
        self._execution.run()

    def wall_period_for_scheduler_period(self, scheduler_period_s: float) -> float:
        return self._execution.wall_period_for_scheduler_period(
            scheduler_period_s,
        )

    def _run_generation(self, launch: ThreadLaunchGate) -> None:
        if not launch.enter():
            return
        self._failures.run(self.run, self._coordinator.stop_event.set)

    def _retire(self, thread: rt.Thread) -> None:
        with self._lifecycle_lock:
            if self._thread is thread:
                self._thread = None
                self._launch = None
                self._started = False


__all__ = ["SimDetectorWorker"]
