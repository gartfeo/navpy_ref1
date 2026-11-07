"""Fresh-visual-lock gate before confirm and NAV (HITL step 25).

A lock is established once one track's good frames have kept the POI within the image error of the
optical axis for the dwell, measured on the frames' own timestamps. A good frame is fresh (confirm's
rule), from an armed detector, a frame-atomic pinhole frame, finite, and within the image error. The
error is the angle between the pixel's pinhole ray and the optical axis: frame-local, so no heading,
position, range or truth enters it.

The lock holds on a tick only when that tick shows a good frame. A short blip (missing, invalid or
off-axis frames) does not lose it (owner, 2026-11-07: do not lose the lock, ask again only when not
sure): it is lost only when no good frame has arrived for longer than the profile's frame gap on the
decision clock, two good frames are further apart than that on their own timestamps, a frame comes
from another track, or the frames' time goes back (any frame, good or not, counts for these two). ``kept`` says whether an established lock is
still held through such a blip.
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
    """Track the lock of the active POI's frames."""

    def __init__(
        self,
        settings: AcquisitionGateSettings,
        fresh: Callable[[DetectedObject], bool],
        armed: Callable[[], bool],
        now_s: Callable[[], float],
    ) -> None:
        self._settings = settings
        self._fresh = fresh
        self._armed = armed
        self._now_s = now_s
        self.reset()

    def reset(self) -> None:
        self._track: tuple[int, int] | None = None
        self._last_ts: float | None = None  # newest distinct frame of this track, good or not
        self._drop()

    def _drop(self) -> None:
        """Lose the lock; the track and its newest frame are kept."""
        self._since_ts: float | None = None  # first good frame of the dwell (frame clock)
        self._good_ts: float | None = None  # newest good frame (frame clock)
        self._good_s: float | None = None  # when it arrived (decision clock)

    @property
    def kept(self) -> bool:
        """An established lock not lost (it may be in a blip on this tick)."""
        return (
            self._since_ts is not None
            and self._good_ts - self._since_ts >= self._settings.lock_dwell_s
        )

    def locked(self, poi: DetectedObject | None) -> bool:
        """Take this tick's detection (None: none) and say whether the lock holds on it."""
        now_s = float(self._now_s())
        if self._good_s is not None and not 0.0 <= now_s - self._good_s <= self._settings.lock_max_frame_gap_s:
            self._drop()  # no good frame for longer than the frame gap
        new = self._watch(poi)
        sample = self._admit(poi)
        if sample is None or sample.image_error_deg > self._settings.lock_max_image_error_deg:
            return False  # a blip: not on this tick, lost only by the frame gap
        if not new:
            return self.kept and sample.timestamp_s == self._good_ts  # the newest good frame again
        if self._good_ts is not None and sample.timestamp_s - self._good_ts > self._settings.lock_max_frame_gap_s:
            self._drop()
        self._good_ts, self._good_s = sample.timestamp_s, now_s
        if self._since_ts is None:
            self._since_ts = sample.timestamp_s
        return self.kept

    def _watch(self, poi: DetectedObject | None) -> bool:
        """Watch the track and time of any frame, good or not; True when it is a new frame."""
        if poi is None:
            return False
        try:
            track = (int(get_poi_task_id(poi)), int(poi.identity.obj_id))
            timestamp_s = float(poi.pixel.source_timestamp_s)
        except (AttributeError, TypeError, ValueError):
            return False
        if track != self._track:
            self.reset()  # another track: lost even when its frame is not good
            self._track = track
        if not math.isfinite(timestamp_s) or timestamp_s == self._last_ts:
            return False
        if self._last_ts is not None and timestamp_s < self._last_ts:
            self._drop()  # the frames' time went back
        self._last_ts = timestamp_s
        return True

    def _admit(self, poi: DetectedObject | None) -> LockSample | None:
        if poi is None or not self._armed() or not self._fresh(poi):
            return None
        return lock_sample(poi)


__all__ = ["FreshVisualLock", "LockSample", "lock_sample"]
