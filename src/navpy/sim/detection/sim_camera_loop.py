"""The sim camera's latency and drops on the runtime clock (dev sim step 5)."""

from __future__ import annotations

from collections import deque
from collections.abc import Callable
from dataclasses import dataclass
from typing import Optional

from navpy.core.kernel import runtime_time as rt
from navpy.sim.detection.camera_frame_ports import (
    CameraFrameSchedulePort,
    CaptureFramePort,
    CapturedFramePort,
)
from navpy.sim.detection.sim_polling_pose_reader import PollingPoseRead
from navpy.sim.detection.sim_runtime_ports import FrameOutcomeSink


@dataclass(frozen=True)
class SimCamera:
    """The frame schedule (``camera_frames``) and the capture it times."""

    schedule: CameraFrameSchedulePort
    capture_frame: CaptureFramePort


class SimCameraLoop:
    """Capture at each frame slot, publish each frame at its due time.

    A slot reads the pose and, unless the draw drops it, renders the frame;
    frames are published in capture order, each with its publication as its
    receipt time. The wall-paced poll timer is not used.
    """

    def __init__(
        self,
        camera: SimCamera,
        stop_event: rt.Event,
        read_pose: Callable[[], PollingPoseRead],
        record_outcome: FrameOutcomeSink,
    ) -> None:
        self._camera = camera
        self._stop_event = stop_event
        self._read_pose = read_pose
        self._record_outcome = record_outcome

    def run(self) -> None:
        pending: deque[tuple[float, CapturedFramePort]] = deque()
        next_slot_s = rt.monotonic()
        try:
            while not self._stop_event.is_set():
                now_s = rt.monotonic()
                while pending and pending[0][0] <= now_s:
                    _due_s, frame = pending.popleft()
                    if not frame.publish(rt.time()):
                        # A reset or stop since its capture: the frame is stale.
                        self._record_outcome(frame.timestamp_s, "camera_rejected_dropped", None)
                if now_s >= next_slot_s:
                    draw = self._camera.schedule.next_frame()
                    frame = self._capture_once(draw.dropped)
                    if frame is not None:
                        # Only the head is published, so a frame due early waits for the one before it.
                        pending.append((draw.due_s(now_s), frame))
                    next_slot_s = draw.next_slot_s(now_s)
                wake_s = min(next_slot_s, pending[0][0]) if pending else next_slot_s
                self._stop_event.wait(max(0.0, wake_s - rt.monotonic()))
        finally:
            for _due_s, frame in pending:
                self._record_outcome(frame.timestamp_s, "camera_stopped_dropped", None)

    def _capture_once(self, dropped: bool) -> Optional[CapturedFramePort]:
        read = self._read_pose()
        sample = read.sample
        if sample is None:
            return None
        if dropped:
            self._record_outcome(read.attitude_time_boot_s, "camera_dropped", None)
            return None
        return self._camera.capture_frame(
            sample.location,
            sample.attitude,
            attitude_time_boot_s=sample.time_boot_s,
            uas_body_rates_rad_s=sample.body_rates_rad_s,
            frame_receipt_timestamp_s=sample.receipt_time_s,
            frame_air_speed_mps=sample.air_speed_mps,
        )


__all__ = ["SimCamera", "SimCameraLoop"]
