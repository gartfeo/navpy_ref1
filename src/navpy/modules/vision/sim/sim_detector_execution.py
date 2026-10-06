"""Source-driven and polling execution loops for the simulator detector."""

from __future__ import annotations

import time

from navpy.modules.vision.sim.ideal_pose_source import IdealPoseSample
from navpy.modules.vision.sim.scheduler_cadence_lease import SchedulerCadenceLease
from navpy.modules.vision.sim.sim_coordinator_ports import WorkerCoordinatorPort
from navpy.modules.vision.sim.sim_polling_pose_reader import PollingPoseReaderPort
from navpy.modules.vision.sim.sim_runtime_ports import (
    DetectFramePort,
    ErrorSink,
    FrameOutcomeSink,
)


class SimDetectorExecution:
    """Execute detection without owning its worker-thread lifecycle."""

    def __init__(
        self,
        *,
        coordinator: WorkerCoordinatorPort[IdealPoseSample],
        polling_pose_reader: PollingPoseReaderPort,
        error: ErrorSink,
        cadence_lease: SchedulerCadenceLease,
        scheduler_period_s: float,
        ideal_360: bool,
        detect_targets: DetectFramePort,
        record_outcome: FrameOutcomeSink,
    ) -> None:
        self._coordinator = coordinator
        self._polling_pose_reader = polling_pose_reader
        self._error = error
        self._cadence_lease = cadence_lease
        self._scheduler_period_s = float(scheduler_period_s)
        self._ideal_360 = bool(ideal_360)
        self._detect_targets = detect_targets
        self._record_outcome = record_outcome

    def run(self) -> None:
        if self._ideal_360:
            self._run_ideal_source_loop()
        else:
            self._run_polling_loop()

    def close(self) -> None:
        self._cadence_lease.close()

    def wall_period_for_scheduler_period(self, scheduler_period_s: float) -> float:
        return self._cadence_lease.wall_period_for_scheduler_period(
            scheduler_period_s,
        )

    def _run_ideal_source_loop(self) -> None:
        stop_event = self._coordinator.stop_event
        while not stop_event.is_set():
            pose = self._coordinator.wait_and_pop_pose()
            if pose is None:
                return
            if pose.generation.invalidated.is_set() or stop_event.is_set():
                outcome = (
                    "source_stopped_dropped"
                    if stop_event.is_set()
                    else "source_invalidated_dropped"
                )
                self._record_outcome(pose.timestamp_s, outcome, None)
                continue
            try:
                self._detect_targets(
                    pose.location,
                    pose.render_attitude,
                    frame_timestamp_s=pose.timestamp_s,
                    frame_receipt_timestamp_s=pose.receipt_time_s,
                    frame_air_speed_mps=pose.air_speed_mps,
                    frame_navigation_attitude=pose.navigation_attitude,
                    uas_body_rates_rad_s=pose.body_rates_rad_s,
                    frame_generation=pose.generation,
                    frame_source_discontinuity=pose.source_discontinuity,
                )
            except Exception as error:
                self._record_outcome(
                    pose.timestamp_s,
                    "render_failed_dropped",
                    None,
                )
                self._error(
                    "DetectorSim ideal source render failed at raw AP time "
                    f"{pose.timestamp_s:.6f}s",
                    error,
                )

    def _run_polling_loop(self) -> None:
        stop_event = self._coordinator.stop_event
        while not stop_event.is_set():
            started_s = time.monotonic()
            sample = self._polling_pose_reader.read()
            self._detect_targets(
                sample.location,
                sample.attitude,
                attitude_time_boot_s=sample.time_boot_s,
                uas_body_rates_rad_s=sample.body_rates_rad_s,
                frame_receipt_timestamp_s=sample.receipt_time_s,
                frame_air_speed_mps=sample.air_speed_mps,
            )
            elapsed_s = time.monotonic() - started_s
            stop_event.wait(max(
                0.0,
                self.wall_period_for_scheduler_period(
                    self._scheduler_period_s
                ) - elapsed_s,
            ))


__all__ = ["SimDetectorExecution"]
