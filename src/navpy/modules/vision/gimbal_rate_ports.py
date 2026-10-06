"""Exact command and logging ports for gimbal rate tracking."""

from __future__ import annotations

from typing import Protocol


class GimbalRateActuator(Protocol):
    def set_rate(self, yaw_rate: float, pitch_rate: float) -> None: ...


class GimbalRateLog(Protocol):
    def debug(self, message: str) -> None: ...

    def info(self, message: str) -> None: ...

    def warning(self, message: str) -> None: ...


__all__ = ["GimbalRateActuator", "GimbalRateLog"]
