"""Resolve raw simulator frame timestamps through exact clock operations."""

from __future__ import annotations

from typing import Optional, Protocol

from navpy.modules.vision.sim.sim_runtime_ports import (
    AttitudeTimestampAcceptor,
)


class FrameTimestampPort(Protocol):
    def resolve(
        self,
        attitude_time_boot_s: Optional[float],
        supplied_timestamp_s: Optional[float],
    ) -> Optional[float]: ...


class SimFrameTimestampResolver:
    """Choose a supplied source time or validate one raw AP boot time."""

    def __init__(
        self,
        accept_attitude: AttitudeTimestampAcceptor,
    ) -> None:
        self._accept_attitude = accept_attitude

    def resolve(
        self,
        attitude_time_boot_s: Optional[float],
        supplied_timestamp_s: Optional[float],
    ) -> Optional[float]:
        if supplied_timestamp_s is not None:
            return supplied_timestamp_s
        return self._accept_attitude(
            attitude_time_boot_s,
            record_emitted=True,
        )


__all__ = ["FrameTimestampPort", "SimFrameTimestampResolver"]
