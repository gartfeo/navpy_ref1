"""Synchronized SIYI telemetry state and connection-local provenance."""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass, replace

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData


_ParsedAttitude = tuple[int, float, float, float, float]
_ParsedZoom = tuple[int, float, float]


def _finite_number(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    number = float(value)
    return number if math.isfinite(number) else None


def _parse_attitude(sample: object) -> _ParsedAttitude | None:
    if not isinstance(sample, (tuple, list)) or len(sample) != 5:
        return None
    seq, stamp, yaw, pitch, roll = sample
    stamp_number = _finite_number(stamp)
    yaw_number = _finite_number(yaw)
    pitch_number = _finite_number(pitch)
    roll_number = _finite_number(roll)
    if (
        isinstance(seq, bool)
        or not isinstance(seq, int)
        or stamp_number is None
        or stamp_number <= 0.0
        or yaw_number is None
        or pitch_number is None
        or roll_number is None
    ):
        return None
    return seq, stamp_number, yaw_number, pitch_number, roll_number


def _parse_zoom(
    sample: object,
    now_monotonic_s: float,
) -> _ParsedZoom | None:
    if not isinstance(sample, (tuple, list)) or len(sample) != 3:
        return None
    seq, level, receipt = sample
    level_number = _finite_number(level)
    receipt_number = _finite_number(receipt)
    now_number = _finite_number(now_monotonic_s)
    if (
        isinstance(seq, bool)
        or not isinstance(seq, int)
        or level_number is None
        or level_number <= 0.0
        or receipt_number is None
        or now_number is None
        or not 0.0 <= receipt_number <= now_number
    ):
        return None
    return seq, level_number, receipt_number


@dataclass(frozen=True)
class SiyiReadbackSnapshot:
    data: GimbalData
    zoom_level: float | None = None
    zoom_receipt_monotonic_s: float | None = None
    zoom_sample_id: int | None = None
    attitude_sample_id: int | None = None
    pitch_sign: float = 1.0


def _with_attitude(
    snapshot: SiyiReadbackSnapshot,
    sample: _ParsedAttitude | None,
) -> tuple[SiyiReadbackSnapshot, bool]:
    if sample is None or sample[0] == snapshot.attitude_sample_id:
        return snapshot, False
    seq, stamp, yaw, pitch, roll = sample
    return (
        replace(
            snapshot,
            data=replace(
                snapshot.data,
                att=Attitude(pitch, yaw, roll),
                timestamp_s=stamp,
            ),
            attitude_sample_id=seq,
        ),
        True,
    )


def _with_zoom(
    snapshot: SiyiReadbackSnapshot,
    sample: _ParsedZoom | None,
) -> tuple[SiyiReadbackSnapshot, bool]:
    if sample is None or sample[0] == snapshot.zoom_sample_id:
        return snapshot, False
    seq, level, receipt = sample
    return (
        replace(
            snapshot,
            zoom_level=level,
            zoom_receipt_monotonic_s=receipt,
            zoom_sample_id=seq,
        ),
        True,
    )


class SiyiReadbackStore:
    """Owns atomic telemetry updates and rejects invalid SDK samples."""

    def __init__(self, initial_data: GimbalData) -> None:
        self._initial_data = replace(initial_data, timestamp_s=None)
        self._lock = threading.Lock()
        self._snapshot = SiyiReadbackSnapshot(self._initial_data)
        self._attitude_ready = threading.Event()

    @property
    def attitude_ready(self) -> threading.Event:
        return self._attitude_ready

    def reset_session(self) -> None:
        with self._lock:
            self._snapshot = SiyiReadbackSnapshot(self._initial_data)
            self._attitude_ready.clear()

    def accept_attitude(self, sample: object) -> bool:
        parsed = _parse_attitude(sample)
        with self._lock:
            updated, accepted = _with_attitude(self._snapshot, parsed)
            if accepted:
                self._snapshot = updated
                self._attitude_ready.set()
            return accepted

    def accept_zoom(self, sample: object, now_monotonic_s: float) -> bool:
        parsed = _parse_zoom(sample, now_monotonic_s)
        with self._lock:
            updated, accepted = _with_zoom(self._snapshot, parsed)
            if accepted:
                self._snapshot = updated
            return accepted

    def commit_poll(
        self,
        attitude_sample: object,
        zoom_sample: object,
        now_monotonic_s: float,
    ) -> tuple[bool, bool]:
        """Publish one attitude/zoom poll without an observable torn pair."""
        parsed_attitude = _parse_attitude(attitude_sample)
        parsed_zoom = _parse_zoom(zoom_sample, now_monotonic_s)
        with self._lock:
            updated, attitude_accepted = _with_attitude(
                self._snapshot,
                parsed_attitude,
            )
            updated, zoom_accepted = _with_zoom(updated, parsed_zoom)
            if attitude_accepted or zoom_accepted:
                self._snapshot = updated
            if attitude_accepted:
                self._attitude_ready.set()
            return attitude_accepted, zoom_accepted

    def set_mount_orientation(self) -> tuple[float, float]:
        with self._lock:
            roll = self._snapshot.data.att.roll
            pitch_sign = -1.0 if abs(roll) > 90.0 else 1.0
            self._snapshot = replace(self._snapshot, pitch_sign=pitch_sign)
            return roll, pitch_sign

    def snapshot(self) -> SiyiReadbackSnapshot:
        with self._lock:
            return self._snapshot


__all__ = ["SiyiReadbackSnapshot", "SiyiReadbackStore"]
