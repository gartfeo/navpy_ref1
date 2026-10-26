"""Camera latency and frame drops: the sim camera's frame schedule (dev sim step 5).

Step 5 of ``docs/design/deterministic_dev_sim.md``. The sim camera in a NavPy
participant captures the whole frame at a tick, as without the model (pose,
gimbal, optics, POIs and capture time), and publishes it ``L`` ticks later. Its
capture time is kept; its receipt time is the tick it is published in, on the
same host clock as a pose receipt. The camera's own latency is drawn once, first,
from ``min_latency_ticks``-``max_latency_ticks``: the ZR10 publishes at a steady
interval (median 41.8 ms, p99 53.7 ms), so its latency does not change from frame
to frame. Then per frame slot the stream draws, in order:

- ``dropped``: with ``drop_pct`` % the slot's frame is skipped (never rendered);
- the tail: with ``tail_pct`` % the frame's latency is a value above the camera's
  latency up to ``tail_latency_ticks`` (a stall; the frames behind it then publish
  in a burst);
- the gap to the next slot: ``gap_ticks``, and with ``long_gap_pct`` % one tick more.

Every slot draws all four values, so the sequence does not depend on which
frames were dropped. Frames are published in capture order (a camera pipeline
does not reorder): one due before an earlier frame waits for it.

The draw reaches a NavPy role through its role config, as the step-4 models do
(``participant_budget.SIM_MODELS_KEY``); the stream is
``participant.<ordinal>.<generation>.camera.<mount index>``. With the model off,
nothing is built and the sim detector runs as before step 5.
"""

from __future__ import annotations

import math
import random
from collections.abc import Callable, Mapping
from dataclasses import dataclass

from navpy.sim.lockstep.sim_profile import Number, model_stream

CAMERA_FRAMES = "camera_frames"
#: One lockstep tick: the SITL's 50 Hz loop (``SCHED_LOOP_RATE`` default 50); t_k is its clock.
TICK_S = 0.020
#: A wait for whole ticks ends this far before the nominal instant, so float rounding of
#: t_k never moves a due frame to the tick after it.
_TICK_MARGIN_S = TICK_S / 2


def _ticks(params: Mapping[str, Number], name: str, low: int) -> int:
    value = params[name]
    if isinstance(value, bool) or not isinstance(value, int) or value < low:
        raise ValueError(f"{name} must be an int of at least {low}, got {value!r}")
    return value


def _pct(params: Mapping[str, Number], name: str) -> float:
    value = params[name]
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not (
            math.isfinite(value) and 0.0 <= value <= 100.0):
        raise ValueError(f"{name} must be a percentage within [0, 100], got {value!r}")
    return float(value)


def _uniform_int(stream: random.Random, low: int, high: int) -> int:
    span = high - low
    return low + min(span, math.floor(stream.random() * (span + 1)))


@dataclass(frozen=True)
class FrameDraw:
    """One frame slot's draw: dropped or not, its latency and the gap to the next slot."""

    dropped: bool
    latency_ticks: int
    gap_ticks: int

    def due_s(self, captured_s: float) -> float:
        """The earliest runtime clock reading at which the frame captured at ``captured_s`` is published."""
        return captured_s + self.latency_ticks * TICK_S - _TICK_MARGIN_S

    def next_slot_s(self, captured_s: float) -> float:
        return captured_s + self.gap_ticks * TICK_S - _TICK_MARGIN_S


class CameraFrameSchedule:
    """One camera's frame slots, drawn in slot order from its own stream."""

    def __init__(self, params: Mapping[str, Number], stream: random.Random) -> None:
        min_latency = _ticks(params, "min_latency_ticks", 0)
        max_latency = _ticks(params, "max_latency_ticks", 0)
        self._tail_latency = _ticks(params, "tail_latency_ticks", 0)
        self._tail_pct = _pct(params, "tail_pct")
        self._gap = _ticks(params, "gap_ticks", 1)
        self._long_gap_pct = _pct(params, "long_gap_pct")
        self._drop_pct = _pct(params, "drop_pct")
        if min_latency > max_latency or max_latency > self._tail_latency:
            raise ValueError("camera latency ticks need min <= max <= tail, got "
                             f"{min_latency}, {max_latency}, {self._tail_latency}")
        self._stream = stream
        self.latency_ticks = _uniform_int(stream, min_latency, max_latency)

    def next_frame(self) -> FrameDraw:
        stream = self._stream
        dropped = stream.random() * 100.0 < self._drop_pct
        tail = stream.random() * 100.0 < self._tail_pct
        tail_latency = _uniform_int(stream, self.latency_ticks + 1, self._tail_latency)
        latency = tail_latency if tail and self._tail_latency > self.latency_ticks else self.latency_ticks
        gap = self._gap + (stream.random() * 100.0 < self._long_gap_pct)
        return FrameDraw(dropped, latency, gap)


#: A NavPy's camera schedules, one per mount index.
CameraFrameFactory = Callable[[int], CameraFrameSchedule]


def camera_frame_factory(models: Mapping[str, Mapping[str, Number]], seed: int, ordinal: int,
                         generation: int) -> CameraFrameFactory | None:
    """The camera schedules of one participant, or None while the model is off."""
    params = models.get(CAMERA_FRAMES)
    if params is None:
        return None
    key = f"participant.{ordinal}.{generation}.camera"
    CameraFrameSchedule(params, random.Random(0))  # a bad draw refuses the run before anything starts

    def schedule(mount_index: int) -> CameraFrameSchedule:
        return CameraFrameSchedule(params, model_stream(seed, CAMERA_FRAMES, f"{key}.{mount_index}"))

    return schedule


__all__ = [
    "CAMERA_FRAMES", "CameraFrameFactory", "CameraFrameSchedule", "FrameDraw", "TICK_S",
    "camera_frame_factory",
]
