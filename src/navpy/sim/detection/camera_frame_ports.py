"""Ports of the sim camera's frame schedule (dev sim step 5, ``camera_frames``).

The schedule (``navpy.sim.lockstep.camera_frames``) decides per frame slot
whether the frame is dropped, when it is published and when the next slot is;
the detector loop captures at a slot and publishes the captured frame later.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Optional, Protocol, Tuple

from navpy.core.kernel.attitude import Attitude
from navpy.core.kernel.location import Location


class FrameSlotDraw(Protocol):
    @property
    def dropped(self) -> bool: ...

    def due_s(self, captured_s: float) -> float: ...

    def next_slot_s(self, captured_s: float) -> float: ...


class CameraFrameSchedulePort(Protocol):
    def next_frame(self) -> FrameSlotDraw: ...


#: A NavPy's camera schedules, one per mount index.
CameraFrameFactoryPort = Callable[[int], CameraFrameSchedulePort]


class CapturedFramePort(Protocol):
    """A frame rendered at its capture tick and not yet published."""

    @property
    def timestamp_s(self) -> float: ...

    def publish(self, receipt_timestamp_s: float) -> bool: ...


class CaptureFramePort(Protocol):
    def __call__(
        self,
        camera_location: Location,
        uas_attitude: Attitude,
        attitude_time_boot_s: Optional[float] = None,
        uas_body_rates_rad_s: Optional[Tuple[float, float, float]] = None,
        *,
        frame_receipt_timestamp_s: Optional[float] = None,
        frame_air_speed_mps: Optional[float] = None,
    ) -> Optional[CapturedFramePort]: ...


__all__ = [
    "CameraFrameFactoryPort",
    "CameraFrameSchedulePort",
    "CaptureFramePort",
    "CapturedFramePort",
    "FrameSlotDraw",
]
