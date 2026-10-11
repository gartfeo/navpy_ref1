"""Narrow dependency contracts for the hardware SIYI adapter."""

from __future__ import annotations

from collections.abc import Callable
from typing import Protocol


AttitudeSample = tuple[int, float, float, float, float]
ZoomSample = tuple[int, float, float]
MonotonicClock = Callable[[], float]


class SiyiConnectionPort(Protocol):
    def connect(self) -> bool: ...

    def disconnect(self) -> bool | None: ...

    def isConnected(self) -> bool: ...


class SiyiControlPort(Protocol):
    def requestCenterGimbal(self) -> bool: ...

    def requestLockMode(self) -> bool: ...

    def requestFollowMode(self) -> bool: ...

    def requestFPVMode(self) -> bool: ...

    def requestAbsoluteZoom(self, level: float) -> bool: ...

    def requestAutoFocus(self) -> bool: ...

    def requestZoomIn(self) -> bool: ...

    def requestZoomOut(self) -> bool: ...

    def requestZoomHold(self) -> bool: ...

    def requestSetAngles(self, yaw_deg: float, pitch_deg: float) -> bool: ...

    def requestGimbalSpeed(self, yaw_speed: int, pitch_speed: int) -> bool: ...


class SiyiTelemetryPort(Protocol):
    def getAttitudeSample(self) -> AttitudeSample: ...

    def getCurrentZoomLevelSample(self) -> ZoomSample: ...

    def requestCurrentZoomLevel(self) -> bool: ...


class SiyiSdkPort(
    SiyiConnectionPort,
    SiyiControlPort,
    SiyiTelemetryPort,
    Protocol,
):
    """Complete SDK shape assembled from focused capability contracts."""


class SiyiSdkFactory(Protocol):
    def __call__(self, *, server_ip: str, port: int) -> SiyiSdkPort: ...


__all__ = [
    "AttitudeSample",
    "MonotonicClock",
    "SiyiSdkFactory",
    "SiyiSdkPort",
    "ZoomSample",
]
