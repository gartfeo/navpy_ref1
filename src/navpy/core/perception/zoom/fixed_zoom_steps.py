"""The calibrated zoom steps of one stream mode (HITL step 18).

A step is an absolute zoom command whose settled lens state has a measured row in the reading-keyed table
(``camera/reading_intrinsics.py``): the command names the setting, the row's reading is what the camera reports once
the lens is there, and the row's intrinsics are the ones a frame at that step is projected with. Interpolated rows
and the wide stop (reached by a held zoom-out, not by a command) are not steps. The reading is the zoom key, so two
steps may not share one: a reading would then not say which step the lens is at.
"""

from __future__ import annotations

from dataclasses import dataclass

from navpy.core.perception.camera.reading_intrinsics import (
    ABSOLUTE_ZOOM,
    MEASURED,
    ReadingIntrinsicsTable,
    ReadingOptics,
    StreamMode,
    zoom_tenths,
)


@dataclass(frozen=True)
class ZoomStep:
    command_tenths: int
    reading_tenths: int
    optics: ReadingOptics

    @property
    def command(self) -> str:
        """The absolute zoom command, as the mount's zoom key ("2.0")."""
        return f"{self.command_tenths // 10}.{self.command_tenths % 10}"


class FixedZoomSteps:
    """Steps ordered widest first (smallest focal length)."""

    def __init__(self, stream: StreamMode, steps: tuple[ZoomStep, ...]) -> None:
        if not steps:
            raise ValueError("fixed zoom needs at least one measured absolute-zoom step")
        readings = [step.reading_tenths for step in steps]
        if len(set(readings)) != len(readings):
            raise ValueError("fixed zoom steps must have distinct readings")
        commands = [step.command_tenths for step in steps]
        if len(set(commands)) != len(commands):
            raise ValueError("fixed zoom steps must have distinct commands")
        self.stream = stream
        self.steps = tuple(sorted(steps, key=lambda step: step.optics.fx))

    @classmethod
    def from_table(cls, table: ReadingIntrinsicsTable, stream: StreamMode) -> "FixedZoomSteps":
        steps: list[ZoomStep] = []
        for label in table.readings(stream):
            reading = float(label)
            for setting in table.settings(stream, reading):
                if setting.kind != ABSOLUTE_ZOOM or setting.command_tenths is None:
                    continue
                optics = table.optics(stream, reading, reading, setting)
                if optics is None or optics.basis != MEASURED:
                    continue
                tenths = zoom_tenths(float(optics.reading))
                if tenths is not None:
                    steps.append(ZoomStep(setting.command_tenths, tenths, optics))
        return cls(stream, tuple(steps))

    @property
    def widest(self) -> ZoomStep:
        return self.steps[0]

    def by_command(self, command_tenths: int | None) -> ZoomStep | None:
        return next((step for step in self.steps if step.command_tenths == command_tenths), None)

    def narrower(self, step: ZoomStep) -> ZoomStep | None:
        index = self.steps.index(step)
        return self.steps[index + 1] if index + 1 < len(self.steps) else None

    def wider(self, step: ZoomStep) -> ZoomStep | None:
        index = self.steps.index(step)
        return self.steps[index - 1] if index > 0 else None


__all__ = ["FixedZoomSteps", "ZoomStep"]
