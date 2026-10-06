"""Finite zoom readback and command/sample progression state."""

from __future__ import annotations

from navpy.modules.vision.target_zoom_ports import ZoomReadbackPort
from navpy.modules.vision.target_zoom_types import (
    FiniteReading,
    SampleIdReading,
    ZoomReadback,
)


class ZoomReadbackState:
    def __init__(self, readback: ZoomReadbackPort) -> None:
        self._readback = readback
        self._sample_telemetry_seen = False
        self._last_command_sample_id: str | None = None

    def capture(
        self,
        *,
        current: bool = False,
        level: bool = False,
    ) -> ZoomReadback:
        sample = self.sample()
        sample_id = sample.value
        if sample.invalid:
            advanced = False
        elif sample_id is not None:
            self._sample_telemetry_seen = True
            advanced = (
                self._last_command_sample_id is None
                or sample_id != self._last_command_sample_id
            )
        else:
            advanced = not self._sample_telemetry_seen
        return ZoomReadback(
            current=(
                self._readback.current_command()
                if current else FiniteReading(None)
            ),
            fresh=self._readback.fresh_command(),
            level=(self._readback.current_level() if level else FiniteReading(None)),
            sample_id=sample_id,
            sample_advanced=advanced,
            sample_invalid=sample.invalid,
        )

    def record_command(self, sample_id: str | None) -> None:
        self._last_command_sample_id = sample_id

    def sample(self) -> SampleIdReading:
        return self._readback.fresh_sample_id()

    def record_lifecycle_hold(self, sample_id: str | None) -> None:
        if sample_id is not None:
            self._sample_telemetry_seen = True
        self._last_command_sample_id = sample_id

    def clear_command_gate(self) -> None:
        self._last_command_sample_id = None

    def reset_source(self) -> None:
        self._sample_telemetry_seen = False
        self._last_command_sample_id = None


__all__ = ["ZoomReadbackState"]
