"""Zoom dynamics for the simulated SIYI gimbal."""

from __future__ import annotations

from dataclasses import dataclass

from navpy.modules.vision.peripheral.siyi.sim.gimbal_angles import clamp
from navpy.modules.vision.peripheral.siyi.siyi_sdk import ZR10


ZOOM_INCREMENTAL_SPEED = 5.0
ZOOM_SEEK_SPEED = 5.0


@dataclass(frozen=True)
class GimbalZoomSnapshot:
    level: float
    target: float | None
    direction: int


class GimbalZoomPlant:
    def __init__(self, initial_level: float = 1.0) -> None:
        self._level = clamp(float(initial_level), 1.0, ZR10.MAX_ZOOM)
        self._target: float | None = None
        self._direction = 0

    @property
    def level(self) -> float:
        return self._level

    def snapshot(self) -> GimbalZoomSnapshot:
        return GimbalZoomSnapshot(
            self._level,
            self._target,
            self._direction,
        )

    def set_target(self, level: float) -> None:
        self._target = clamp(float(level), 1.0, ZR10.MAX_ZOOM)
        self._direction = 0

    def start(self, direction: int) -> None:
        self._direction = max(-1, min(1, int(direction)))
        self._target = None

    def stop(self) -> None:
        self._direction = 0
        self._target = None

    def advance(self, dt_s: float) -> None:
        if dt_s <= 0.0:
            return
        if self._target is not None:
            error = self._target - self._level
            step = ZOOM_SEEK_SPEED * dt_s
            if abs(error) <= max(0.05, step):
                self._level = self._target
                self._target = None
            else:
                self._level += step if error > 0.0 else -step
        elif self._direction != 0:
            self._level += self._direction * ZOOM_INCREMENTAL_SPEED * dt_s
        self._level = clamp(self._level, 1.0, ZR10.MAX_ZOOM)


__all__ = [
    "GimbalZoomPlant",
    "GimbalZoomSnapshot",
    "ZOOM_INCREMENTAL_SPEED",
    "ZOOM_SEEK_SPEED",
]
