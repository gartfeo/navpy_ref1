"""Zoom dynamics for the simulated SIYI gimbal."""

from __future__ import annotations

from dataclasses import dataclass

from navpy.sim.gimbal.siyi.gimbal_angles import clamp
from navpy.plugins.gimbal.siyi.siyi_sdk import ZR10


ZOOM_INCREMENTAL_SPEED = 5.0
ZOOM_SEEK_SPEED = 5.0


@dataclass(frozen=True)
class GimbalZoomLimits:
    """Zoom range (levels) and speeds (levels/s); an infinite seek speed jumps."""

    min_level: float = 1.0
    max_level: float = ZR10.MAX_ZOOM
    seek_speed: float = ZOOM_SEEK_SPEED
    incremental_speed: float = ZOOM_INCREMENTAL_SPEED


DEFAULT_ZOOM_LIMITS = GimbalZoomLimits()


@dataclass(frozen=True)
class GimbalZoomSnapshot:
    level: float
    target: float | None
    direction: int


class GimbalZoomPlant:
    def __init__(
        self,
        initial_level: float = 1.0,
        limits: GimbalZoomLimits = DEFAULT_ZOOM_LIMITS,
    ) -> None:
        self._limits = limits
        self._level = self._clamp(float(initial_level))
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

    def _clamp(self, level: float) -> float:
        return clamp(level, self._limits.min_level, self._limits.max_level)

    def set_target(self, level: float) -> None:
        self._target = self._clamp(float(level))
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
            step = self._limits.seek_speed * dt_s
            if abs(error) <= max(0.05, step):
                self._level = self._target
                self._target = None
            else:
                self._level += step if error > 0.0 else -step
        elif self._direction != 0:
            self._level += (
                self._direction * self._limits.incremental_speed * dt_s
            )
        self._level = self._clamp(self._level)


__all__ = [
    "DEFAULT_ZOOM_LIMITS",
    "GimbalZoomLimits",
    "GimbalZoomPlant",
    "GimbalZoomSnapshot",
    "ZOOM_INCREMENTAL_SPEED",
    "ZOOM_SEEK_SPEED",
]
