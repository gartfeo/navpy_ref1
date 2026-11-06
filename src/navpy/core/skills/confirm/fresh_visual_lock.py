"""Fresh-visual-lock gate before confirm and NAV (HITL step 25).

A lock holds once one track's frames have kept the POI within the image error of the optical axis
for the dwell, measured on the frames' own timestamps. The error is the angle between the pixel's
pinhole ray and the optical axis: frame-local, so no heading, position, range or truth enters it.
A frame that is not fresh, not a frame-atomic pinhole frame, or not finite breaks the lock, as does
a gap between new frames longer than the profile's frame gap, a frame from another track, a time
going back, or the error over its limit.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass

from navpy.core.ports.detect_data import DetectedObject
from navpy.core.ports.pixel_observation import PixelProjectionKind
from navpy.core.perception.detection.poi_identity import get_poi_task_id
from navpy.core.skills.confirm.acquisition_gate_settings import AcquisitionGateSettings


@dataclass(frozen=True)
class LockSample:
    track: tuple[int, int]
    timestamp_s: float
    image_error_deg: float


def lock_sample(poi: DetectedObject) -> LockSample | None:
    """The frame-local lock sample of one detection, or None when its vision is invalid."""
    pixel = poi.pixel
    if pixel.projection is not PixelProjectionKind.PINHOLE or pixel.pose_is_frame_atomic is not True:
        return None
    try:
        calibration = pixel.calibration
        x = (float(pixel.u_px) - float(calibration.cx_px)) / float(calibration.fx_px)
        y = (float(pixel.v_px) - float(calibration.cy_px)) / float(calibration.fy_px)
        timestamp_s = float(pixel.source_timestamp_s)
        track = (int(get_poi_task_id(poi)), int(poi.identity.obj_id))
    except (AttributeError, TypeError, ValueError, ZeroDivisionError):
        return None
    if not all(math.isfinite(value) for value in (x, y, timestamp_s)):
        return None
    return LockSample(track, timestamp_s, math.degrees(math.atan(math.hypot(x, y))))


class FreshVisualLock:
    """Track the lock dwell of the active POI's frames."""

    def __init__(
        self,
        settings: AcquisitionGateSettings,
        fresh: Callable[[DetectedObject], bool],
        armed: Callable[[], bool],
    ) -> None:
        self._settings = settings
        self._fresh = fresh
        self._armed = armed
        self.reset()

    def reset(self) -> None:
        self._track: tuple[int, int] | None = None
        self._last_s: float | None = None
        self._since_s: float | None = None

    def locked(self, poi: DetectedObject | None) -> bool:
        """Take this tick's detection and say whether the lock holds on it."""
        sample = self._admit(poi)
        if sample is None:
            self.reset()
            return False
        self._observe(sample)
        return (
            self._since_s is not None
            and self._last_s - self._since_s >= self._settings.lock_dwell_s
        )

    def _admit(self, poi: DetectedObject | None) -> LockSample | None:
        if poi is None or not self._armed() or not self._fresh(poi):
            return None
        return lock_sample(poi)

    def _observe(self, sample: LockSample) -> None:
        if sample.track != self._track:
            self.reset()
            self._track = sample.track
        last_s = self._last_s
        if last_s is not None and sample.timestamp_s == last_s:
            return  # the same frame seen again: nothing new
        if last_s is not None and (
            sample.timestamp_s < last_s
            or sample.timestamp_s - last_s > self._settings.lock_max_frame_gap_s
        ):
            self._since_s = None
        self._last_s = sample.timestamp_s
        if sample.image_error_deg > self._settings.lock_max_image_error_deg:
            self._since_s = None
        elif self._since_s is None:
            self._since_s = sample.timestamp_s


__all__ = ["FreshVisualLock", "LockSample", "lock_sample"]
