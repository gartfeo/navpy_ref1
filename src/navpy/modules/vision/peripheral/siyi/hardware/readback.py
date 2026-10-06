"""Public readback capability over atomic SIYI telemetry state."""

from __future__ import annotations

from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from navpy.modules.vision.peripheral.siyi.hardware.ports import MonotonicClock
from navpy.modules.vision.peripheral.siyi.hardware.state import SiyiReadbackStore


class SiyiHardwareReadback:
    def __init__(
        self,
        store: SiyiReadbackStore,
        monotonic: MonotonicClock,
    ) -> None:
        self._store = store
        self._monotonic = monotonic

    def get_data(self) -> GimbalData:
        return self._store.snapshot().data

    def get_frame_state_sample(
        self,
    ) -> tuple[GimbalData, float | None, float | None, int | None]:
        sample = self._store.snapshot()
        age_s = self._age(sample.zoom_receipt_monotonic_s)
        return sample.data, sample.zoom_level, age_s, sample.zoom_sample_id

    def get_zoom_level(self) -> float | None:
        return self._store.snapshot().zoom_level

    def get_zoom_level_age_s(self) -> float:
        age_s = self._age(self._store.snapshot().zoom_receipt_monotonic_s)
        return float("inf") if age_s is None else age_s

    def supports_zoom_readback(self) -> bool:
        return True

    def get_zoom_level_sample_id(self) -> int | None:
        return self._store.snapshot().zoom_sample_id

    def get_zoom_level_sample(self) -> tuple[float, float, int] | None:
        sample = self._store.snapshot()
        if (
            sample.zoom_level is None
            or sample.zoom_receipt_monotonic_s is None
            or sample.zoom_sample_id is None
        ):
            return None
        return (
            sample.zoom_level,
            self._monotonic() - sample.zoom_receipt_monotonic_s,
            sample.zoom_sample_id,
        )

    def _age(self, receipt_monotonic_s: float | None) -> float | None:
        if receipt_monotonic_s is None:
            return None
        return self._monotonic() - receipt_monotonic_s


__all__ = ["SiyiHardwareReadback"]
