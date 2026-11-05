"""Fixed-step zoom policy: calibrated steps, both-axis image containment and hysteresis (HITL step 18).

Every command is a calibrated step (``fixed_zoom_steps.py``); nothing between steps is ever commanded. Decisions use
only frame-local measurements of a frame settled at the current step (``zoom_settle_evidence.py``) and that step's
own optics: the detection box in pixels and the frame size. No range, altitude, ground speed or aircraft heading
enters, and the box size is never turned into a range.

Containment is the gap from the box to the nearest frame edge on each axis, as a fraction of that axis:

- zoom out one step when either axis gap is below ``zoom_out``;
- zoom in one step when the box predicted at the next narrower step keeps a gap of at least ``zoom_in`` on both
  axes. ``zoom_in > zoom_out`` is the hysteresis: a step entered by zooming in starts at least ``zoom_in`` inside
  the edges, and a step left by zooming out had a gap below ``zoom_out`` there, so the box predicted back at it
  cannot qualify for zooming in again at once.

The prediction maps a pixel through the pinhole of each row (fx, fy, cx, cy; distortion and skew ignored), which is
the lens change alone with the gimbal and target held still.

While the lens travels or a settled frame is not yet bracketed, nothing is decided. A settled step with no detection
for ``loss_updates`` updates returns to the widest step. An unknown lens state (no step commanded in this session,
a manual zoom, a lens that moved by itself) commands the widest step. Aborted evidence commands nothing.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from navpy.core.perception.zoom.fixed_zoom_steps import FixedZoomSteps, ZoomStep
from navpy.core.perception.zoom.zoom_settle_evidence import (
    STATUS_ZOOM_UNKNOWN,
    ZoomJudgement,
)


@dataclass(frozen=True)
class ContainmentMargins:
    """My values (HITL step 18), not measured.

    ``zoom_out`` 0.10: a box this close to an edge is near leaving the frame within a few frames of gimbal lag.
    ``zoom_in`` 0.25: zooming in keeps the box inside the central half of the frame on both axes.
    """

    zoom_out: float = 0.10
    zoom_in: float = 0.25

    def __post_init__(self) -> None:
        for value in (self.zoom_out, self.zoom_in):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError("containment margins must be finite numbers")
        if not 0.0 <= self.zoom_out < self.zoom_in < 0.5:
            raise ValueError("containment margins need 0 <= zoom_out < zoom_in < 0.5")


# My value (HITL step 18): three settled updates without a detection, so one detector miss does not throw the
# lens into travel (every travel frame is refused).
ZOOM_LOSS_UPDATES = 3


@dataclass(frozen=True)
class FrameBox:
    """A detection box in frame pixels (corners) and the size of its frame."""

    x0: float
    y0: float
    x1: float
    y1: float
    width: float
    height: float

    @classmethod
    def from_cxcywh(cls, box: tuple[float, float, float, float], width: float, height: float) -> "FrameBox | None":
        values = (*box, width, height)
        if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in values):
            return None
        cx, cy, w, h = box
        if w <= 0.0 or h <= 0.0 or width <= 0.0 or height <= 0.0:
            return None
        return cls(cx - w / 2.0, cy - h / 2.0, cx + w / 2.0, cy + h / 2.0, float(width), float(height))

    def gaps(self) -> tuple[float, float]:
        """Gap to the nearest edge on each axis, as a fraction of that axis (negative when the box crosses it)."""
        return (min(self.x0, self.width - self.x1) / self.width,
                min(self.y0, self.height - self.y1) / self.height)


def predict_box(box: FrameBox, current: ZoomStep, target: ZoomStep) -> FrameBox:
    """The box at ``target``'s optics, from its pixels at ``current``'s."""
    a, b = current.optics, target.optics
    sx, sy = b.fx / a.fx, b.fy / a.fy
    return FrameBox(b.cx + sx * (box.x0 - a.cx), b.cy + sy * (box.y0 - a.cy),
                    b.cx + sx * (box.x1 - a.cx), b.cy + sy * (box.y1 - a.cy), box.width, box.height)


class FixedStepZoomPolicy:
    def __init__(self, steps: FixedZoomSteps, margins: ContainmentMargins | None = None,
                 loss_updates: int = ZOOM_LOSS_UPDATES) -> None:
        if isinstance(loss_updates, bool) or not isinstance(loss_updates, int) or loss_updates < 1:
            raise ValueError("loss_updates must be a positive integer")
        self.steps = steps
        self.margins = margins or ContainmentMargins()
        self.loss_updates = loss_updates
        self._losses = 0

    def reset(self) -> None:
        self._losses = 0

    def decide(self, judgement: ZoomJudgement, box: FrameBox | None) -> ZoomStep | None:
        """The step to command now, or None to hold. ``box`` must come from a frame settled at the judged step."""
        if judgement.status == STATUS_ZOOM_UNKNOWN:
            self._losses = 0
            return self.steps.widest
        current = judgement.step
        if not judgement.settled or current is None:
            return None
        if box is None:
            self._losses += 1
            if self._losses < self.loss_updates or current == self.steps.widest:
                return None
            self._losses = 0
            return self.steps.widest
        self._losses = 0
        stream = self.steps.stream
        if (box.width, box.height) != (stream.image_width, stream.image_height):
            return None
        if min(box.gaps()) < self.margins.zoom_out:
            return self.steps.wider(current)
        narrower = self.steps.narrower(current)
        if narrower is not None and min(predict_box(box, current, narrower).gaps()) >= self.margins.zoom_in:
            return narrower
        return None


__all__ = [
    "ContainmentMargins",
    "FixedStepZoomPolicy",
    "FrameBox",
    "ZOOM_LOSS_UPDATES",
    "predict_box",
]
