"""Atomic SIYI simulation attitude and zoom readback."""

from __future__ import annotations

import time

from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from navpy.modules.vision.peripheral.siyi.sim.gimbal_siyi_state import (
    SiyiSimState,
)


class SiyiSimReadback:
    def __init__(self, state: SiyiSimState) -> None:
        self._state = state

    def get_data(self) -> GimbalData:
        with self._state.lock:
            return self._state.data

    def get_frame_state_sample(
        self,
    ) -> tuple[GimbalData, float | None, float | None, int | None]:
        with self._state.lock:
            age_s = self._age_locked()
            if age_s is None:
                return self._state.data, None, None, None
            return (
                self._state.data,
                self._state.zoom.level,
                age_s,
                self._state.zoom_sample_seq,
            )

    def get_zoom_level(self) -> float:
        with self._state.lock:
            return self._state.zoom.level

    def get_zoom_level_age_s(self) -> float:
        with self._state.lock:
            age_s = self._age_locked()
            return float("inf") if age_s is None else age_s

    def get_zoom_level_sample_id(self) -> int:
        with self._state.lock:
            return self._state.zoom_sample_seq

    def get_zoom_level_sample(self) -> tuple[float, float, int] | None:
        with self._state.lock:
            age_s = self._age_locked()
            if age_s is None:
                return None
            return (
                self._state.zoom.level,
                age_s,
                self._state.zoom_sample_seq,
            )

    @staticmethod
    def supports_zoom_readback() -> bool:
        return True

    def _age_locked(self) -> float | None:
        timestamp_s = self._state.zoom_updated_monotonic_s
        if not self._state.started or timestamp_s is None:
            return None
        return max(0.0, time.monotonic() - timestamp_s)


__all__ = ["SiyiSimReadback"]
