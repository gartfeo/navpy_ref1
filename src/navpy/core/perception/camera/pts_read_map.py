"""The PTS -> receiver read time map of integration timing (HITL stage 1 step 11).

An encoded frame carries its media PTS and the capture-clock time it was read
(``CaptureStamp`` READ, ``rt.perf_counter``). Like ``board_clock_map``, the
map takes the lower envelope R of ``read - pts`` over the newest ``window_s``
of unique frames: the read time a frame would have had at the smallest video
delay seen. The offset O it publishes follows R by at most ``update_limit_s``
per unique frame. Every published frame feeds the map once (a repeated PTS
refreshes nothing); a frame is mapped with the O of its own publication,
``mapped = pts + O``.

Aborts, latched for the open, checked before O moves and before the frame is
published:
- a reset: another open, a PTS that goes backward, or a PTS gap over
  ``max_frame_gap_s``. The fitted clock rate is off by up to ~35 ppm under
  2 ms jitter, so across a 30 s gap a 1 ms slip cannot be told from rate
  error: a gap ends slip continuity, and with it the open (fail-closed);
- a slip (``integration_slip.py``);
- R more than ``slip_limit_s`` from O after O moves.
Nothing is mapped until ``rate_window_s`` plus two slip windows of frames,
with ``min_frames`` unique frames, back the checks.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

from navpy.core.kernel import runtime_time as rt
from navpy.core.perception.camera.integration_slip import FloorSlipDetector
from navpy.core.perception.camera.integration_timing import (
    NS_PER_S,
    STATUS_ABORTED,
    STATUS_MAPPED,
    STATUS_UNAVAILABLE,
    STATUS_UNKNOWN_FRAME,
    STATUS_UNSETTLED,
    IntegrationTimingConfig,
    IntegrationTimingLimits,
)
from navpy.core.ports.frame_capture_ports import (
    CaptureStamp,
    CaptureStampKind,
    FrameSourceIdentity,
)


@dataclass(frozen=True)
class MappedFrame:
    """One unique frame as published: PTS, read time, its mapping and the envelope state then."""

    open_id: str
    pts_ns: int
    read_s: float
    mapped_s: float
    offset_s: float
    envelope_s: float
    rate: float | None

    @property
    def interval_s(self) -> float:
        """The frame's time for visual rates: PTS seconds of this open."""
        return self.pts_ns / NS_PER_S


class PtsReadMap:
    """PTS -> receiver read time for one open; aborts for good on reset or slip.

    Not thread-safe: the frame provider feeds it under its publication lock and
    the association reads it under the same lock (``IntegrationTimingSource``).
    """

    def __init__(self, limits: IntegrationTimingLimits) -> None:
        problems = limits.problems()
        if problems:
            raise ValueError("; ".join(problems))
        self._limits = limits
        self._window_ns = round(limits.window_s * NS_PER_S)
        # Slip detection checks the two newest slip windows against the floor
        # fitted over the rate window before them, so it keeps that much.
        self._history_ns = round((limits.rate_window_s + 2 * limits.slip_window_s) * NS_PER_S)
        self._gap_ns = round(limits.max_frame_gap_s * NS_PER_S)
        self._slips = FloorSlipDetector(limits)
        self._open_id: str | None = None
        self._samples: deque[tuple[int, float]] = deque()
        self._frames: dict[int, MappedFrame] = {}
        self._offset_s: float | None = None
        self._first_pts_ns: int | None = None
        self._frame_count = 0
        self._aborted: str | None = None

    @property
    def aborted(self) -> str | None:
        return self._aborted

    def observe(self, stamp: CaptureStamp | None) -> str:
        """Feed one published frame; returns STATUS_MAPPED or why it was not mapped."""
        if self._aborted is not None:
            return STATUS_ABORTED
        source = _source(stamp)
        if source is None:
            return STATUS_UNAVAILABLE
        if self._open_id is None:
            self._open_id = source.open_id
        elif source.open_id != self._open_id:
            return self._abort("reset: another open")
        newest = self._samples[-1][0] if self._samples else None
        if newest is not None and source.pts_ns <= newest:
            if source.pts_ns == newest:
                return STATUS_MAPPED
            return self._abort("reset: PTS went backward")
        return self._update(source.pts_ns, float(stamp.captured_at_s))

    def frame(self, source: FrameSourceIdentity | None) -> tuple[MappedFrame | None, str]:
        """The mapping a frame got when it was published, or why there is none."""
        if self._aborted is not None:
            return None, STATUS_ABORTED
        if source is None or source.open_id != self._open_id:
            return None, STATUS_UNAVAILABLE
        mapped = self._frames.get(source.pts_ns)
        if mapped is None:
            return None, STATUS_UNSETTLED if self._offset_s is None else STATUS_UNKNOWN_FRAME
        if not self._settled_at(source.pts_ns):
            return None, STATUS_UNSETTLED
        return mapped, STATUS_MAPPED

    def _update(self, pts_ns: int, read_s: float) -> str:
        limits = self._limits
        sample = read_s - pts_ns / NS_PER_S
        if self._samples and pts_ns - self._samples[-1][0] > self._gap_ns:
            return self._abort("reset: a PTS gap over max_frame_gap_s")
        if self._first_pts_ns is None:
            self._first_pts_ns = pts_ns
        # The detector has no floor until a full history backs one, so this
        # also checks the frame that first settles the map.
        slip = self._slips.check(self._samples, pts_ns, sample)
        if slip is not None:
            return self._abort(slip)
        self._samples.append((pts_ns, sample))
        self._frame_count += 1
        while self._samples[0][0] < pts_ns - self._history_ns:
            self._samples.popleft()
        envelope = min(value for pts, value in self._samples if pts >= pts_ns - self._window_ns)
        if self._offset_s is None:
            if not self._settled_at(pts_ns):
                # No offset until the first full window: an early minimum is
                # still high by jitter.
                return STATUS_UNSETTLED
            self._offset_s = envelope
        else:
            step = max(-limits.update_limit_s, min(limits.update_limit_s, envelope - self._offset_s))
            self._offset_s += step
            if abs(envelope - self._offset_s) > limits.slip_limit_s:
                return self._abort("slip: the envelope jumped")
        self._frames[pts_ns] = MappedFrame(
            open_id=str(self._open_id),
            pts_ns=pts_ns,
            read_s=read_s,
            mapped_s=pts_ns / NS_PER_S + self._offset_s,
            offset_s=self._offset_s,
            envelope_s=envelope,
            rate=self._slips.rate,
        )
        for old in [key for key in self._frames if key < pts_ns - self._window_ns]:
            del self._frames[old]
        return STATUS_MAPPED

    def _settled_at(self, pts_ns: int) -> bool:
        return (
            self._first_pts_ns is not None
            and self._frame_count >= self._limits.min_frames
            and pts_ns - self._first_pts_ns >= self._history_ns
        )

    def _abort(self, reason: str) -> str:
        self._aborted = reason
        self._frames.clear()
        return STATUS_ABORTED


class IntegrationTimingSource:
    """One run's map behind a lock: the capture thread feeds it, the association reads it."""

    def __init__(self, config: IntegrationTimingConfig) -> None:
        problems = config.problems()
        if problems:
            raise ValueError("integration timing refused: " + "; ".join(problems))
        self.config = config
        self._map = PtsReadMap(config.limits)
        self._lock = rt.Lock()

    @property
    def aborted(self) -> str | None:
        with self._lock:
            return self._map.aborted

    def observe(self, stamp: CaptureStamp | None) -> str:
        with self._lock:
            return self._map.observe(stamp)

    def frame(self, source: FrameSourceIdentity | None) -> tuple[MappedFrame | None, str]:
        with self._lock:
            return self._map.frame(source)


def _source(stamp: CaptureStamp | None) -> FrameSourceIdentity | None:
    if stamp is None or stamp.kind is not CaptureStampKind.READ:
        return None
    source = stamp.source
    read_s = stamp.captured_at_s
    if (
        source is None
        or isinstance(source.pts_ns, bool)
        or not isinstance(source.pts_ns, int)
        or not isinstance(source.open_id, str)
        or isinstance(read_s, bool)
        or not isinstance(read_s, (int, float))
        or not math.isfinite(read_s)
    ):
        return None
    return source


__all__ = ["IntegrationTimingSource", "MappedFrame", "PtsReadMap"]
