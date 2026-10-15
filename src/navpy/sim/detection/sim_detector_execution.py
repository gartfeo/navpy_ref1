"""Source-driven and polling execution loops for the simulator detector."""

from __future__ import annotations


from navpy.core.kernel import runtime_time as rt
from navpy.sim.detection.ideal_pose_source import IdealPoseSample
from navpy.sim.detection.scheduler_cadence_lease import SchedulerCadenceLease
from navpy.sim.detection.sim_coordinator_ports import WorkerCoordinatorPort
from navpy.sim.detection.sim_polling_pose_reader import PollingPoseReaderPort
from navpy.sim.detection.sim_runtime_ports import (
    DetectFramePort,
    ErrorSink,
    FrameOutcomeSink,
    SourceTimeAdvance,
)
from navpy.sim.detection.unpaired_pose_monitor import UnpairedPoseMonitor


class SimDetectorExecution:
    """Execute detection without owning its worker-thread lifecycle."""

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
        self._coordinator = coordinator
        self._polling_pose_reader = polling_pose_reader
        self._advance_source_time = advance_source_time
        self._unpaired_pose_monitor = unpaired_pose_monitor
        self._error = error
        self._cadence_lease = cadence_lease
        self._scheduler_period_s = float(scheduler_period_s)
        self._ideal_360 = bool(ideal_360)
        self._detect_pois = detect_pois
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
                self._detect_pois(
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
            started_s = rt.monotonic()
            self._poll_once()
            elapsed_s = rt.monotonic() - started_s
            stop_event.wait(max(
                0.0,
                self.wall_period_for_scheduler_period(
                    self._scheduler_period_s
                ) - elapsed_s,
            ))

    def _poll_once(self) -> None:
        read = self._polling_pose_reader.read()
        # Every polled ATTITUDE advances the source clock, paired or not, so a
        # restart is detected on the raw stream before any post-reboot pair.
        if read.attitude_time_boot_s is not None:
            self._advance_source_time(read.attitude_time_boot_s)
        sample = read.sample
        self._unpaired_pose_monitor.observe(
            read.attitude_time_boot_s,
            paired=sample is not None,
        )
        if sample is None:
            self._record_outcome(
                read.attitude_time_boot_s,
                "unpaired_pose_dropped",
                None,
            )
            return
        self._detect_pois(
            sample.location,
            sample.attitude,
            attitude_time_boot_s=sample.time_boot_s,
            uas_body_rates_rad_s=sample.body_rates_rad_s,
            frame_receipt_timestamp_s=sample.receipt_time_s,
            frame_air_speed_mps=sample.air_speed_mps,
        )


__all__ = ["SimDetectorExecution"]
