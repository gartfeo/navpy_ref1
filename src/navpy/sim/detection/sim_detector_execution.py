"""Source-driven, polling and camera-scheduled execution loops for the simulator detector."""

from __future__ import annotations


from typing import Optional

from navpy.core.kernel import runtime_time as rt
from navpy.sim.detection.detector_poll_timer import DetectorPollTimer
from navpy.sim.detection.ideal_pose_source import IdealPoseSample
from navpy.sim.detection.sim_camera_loop import SimCamera, SimCameraLoop
from navpy.sim.detection.scheduler_cadence_lease import SchedulerCadenceLease
from navpy.sim.detection.sim_coordinator_ports import WorkerCoordinatorPort
from navpy.sim.detection.sim_polling_pose_reader import (
    PollingPoseRead,
    PollingPoseReaderPort,
)
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
        poll_timer: Optional[DetectorPollTimer] = None,
        camera: Optional[SimCamera] = None,
    ) -> None:
        if camera is not None and ideal_360:
            raise ValueError("camera_frames models the finite polling camera, not ideal_360")
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
        self._poll_timer = poll_timer
        self._camera_loop = None if camera is None else SimCameraLoop(
            camera, coordinator.stop_event, self._read_pose, record_outcome,
        )

    def run(self) -> None:
        if self._ideal_360:
            self._run_ideal_source_loop()
        elif self._camera_loop is not None:
            self._camera_loop.run()
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
        timer = self._poll_timer
        while not stop_event.is_set():
            # perf_counter, not monotonic: on Windows monotonic ticks at 15.6 ms
            # and reads a 1-3 ms frame as 0, so the period would grow by the work.
            started_s = rt.perf_counter()
            if timer is not None:
                timer.begin()
            outcome = self._poll_once()
            elapsed_s = rt.perf_counter() - started_s
            wait_s = max(
                0.0,
                self.wall_period_for_scheduler_period(
                    self._scheduler_period_s
                ) - elapsed_s,
            )
            if timer is None:
                stop_event.wait(wait_s)
                continue
            timer.end_work()
            stop_event.wait(wait_s)
            timer.finish(outcome, wait_s)

    def _mark(self, stage: str) -> None:
        if self._poll_timer is not None:
            self._poll_timer.mark(stage)

    def _read_pose(self) -> PollingPoseRead:
        """Poll the pose; an unpaired one is recorded as dropped."""
        read = self._polling_pose_reader.read()
        # Every polled ATTITUDE advances the source clock, paired or not, so a
        # restart is detected on the raw stream before any post-reboot pair.
        if read.attitude_time_boot_s is not None:
            self._advance_source_time(read.attitude_time_boot_s)
        self._unpaired_pose_monitor.observe(
            read.attitude_time_boot_s,
            paired=read.sample is not None,
        )
        self._mark("read")
        if read.sample is None:
            self._record_outcome(
                read.attitude_time_boot_s,
                "unpaired_pose_dropped",
                None,
            )
        return read

    def _poll_once(self) -> str:
        read = self._read_pose()
        sample = read.sample
        if sample is None:
            self._mark("log")
            return "empty" if read.attitude_time_boot_s is None else "unpaired"
        if self._poll_timer is not None:
            self._poll_timer.frame()
        self._detect_pois(
            sample.location,
            sample.attitude,
            attitude_time_boot_s=sample.time_boot_s,
            uas_body_rates_rad_s=sample.body_rates_rad_s,
            frame_receipt_timestamp_s=sample.receipt_time_s,
            frame_air_speed_mps=sample.air_speed_mps,
        )
        # The pipeline marks "project" after projection and "publish" after
        # the commit; the rest (the projection evidence write) is logging.
        self._mark("log")
        return "paired"


__all__ = ["SimDetectorExecution"]
