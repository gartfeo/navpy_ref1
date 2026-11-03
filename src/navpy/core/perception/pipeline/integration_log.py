"""The integration timing log's records (HITL stage 1 steps 11 and 12).

``integration_association.py`` writes one ``association`` record per attempt, built from these pieces, and before
it every ATTITUDE and gimbal receipt not logged yet (``attitude_receipt``, ``gimbal_receipt``), once each: the
receiver-local histories step 12 fits each correction from (``tools/hitl/check_integration_timing.py``).
"""
from __future__ import annotations

import math
from collections.abc import Callable, Sequence
from types import SimpleNamespace

from navpy.core.perception.camera.capture_interpolation import AttitudeAtLookup
from navpy.core.perception.camera.pts_read_map import MappedFrame
from navpy.core.ports.gimbal_capture_history import GimbalCaptureHistory

EVENT_ASSOCIATION = "association"
EVENT_ATTITUDE_RECEIPT = "attitude_receipt"
EVENT_GIMBAL_RECEIPT = "gimbal_receipt"

TimingLog = Callable[[dict[str, object]], None]


def _finite(value: object) -> bool:
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)


class ReceiptLog:
    """Logs each receipt an association can see once, before its record."""

    def __init__(self, log: TimingLog, run_id: str) -> None:
        self._log = log
        self._run_id = run_id
        # The newest receipt already logged per history, so each sample is logged once.
        self._logged_attitude_s = -math.inf
        self._logged_gimbal: tuple[int, float] | None = None

    def log(self, attitude_history: Sequence[SimpleNamespace], gimbal_history: GimbalCaptureHistory | None) -> None:
        for sample in attitude_history:
            receipt = getattr(sample, "capture_receipt_s", None)
            if not _finite(receipt) or receipt <= self._logged_attitude_s:
                continue
            self._logged_attitude_s = receipt
            boot = getattr(sample, "time_boot_s", None)
            self._log({
                "event": EVENT_ATTITUDE_RECEIPT, "run_id": self._run_id,
                "time_boot_s": boot if _finite(boot) else None, "capture_receipt_s": receipt,
            })
        if not isinstance(gimbal_history, GimbalCaptureHistory):
            return
        for sample in gimbal_history.samples:
            last = self._logged_gimbal
            if last is not None and last[0] == sample.session and sample.receipt_s <= last[1]:
                continue
            self._logged_gimbal = (sample.session, sample.receipt_s)
            self._log({
                "event": EVENT_GIMBAL_RECEIPT, "run_id": self._run_id,
                "session": sample.session, "seq": sample.seq, "receipt_s": sample.receipt_s,
            })


def attitude_history_record(history: Sequence[SimpleNamespace]) -> dict[str, object]:
    receipts = [getattr(sample, "capture_receipt_s", None) for sample in history]
    return {
        "count": len(receipts),
        "first_receipt_s": receipts[0] if receipts else None,
        "last_receipt_s": receipts[-1] if receipts else None,
    }


def gimbal_history_record(history: GimbalCaptureHistory | None) -> dict[str, object] | None:
    """The snapshot this attempt saw, so an offline replay uses exactly its samples."""
    if not isinstance(history, GimbalCaptureHistory):
        return None
    samples = history.samples
    return {
        "session": history.session,
        "count": len(samples),
        "first_receipt_s": samples[0].receipt_s if samples else None,
        "last_receipt_s": samples[-1].receipt_s if samples else None,
    }


def mapping_record(mapped: MappedFrame) -> dict[str, object]:
    return {
        "read_s": mapped.read_s,
        "mapped_s": mapped.mapped_s,
        "offset_s": mapped.offset_s,
        "envelope_s": mapped.envelope_s,
        "rate": mapped.rate,
    }


def attitude_record(attitude: AttitudeAtLookup) -> dict[str, object]:
    record: dict[str, object] = {"attitude_time_boot_s": attitude.time_boot_s}
    bracket = attitude.bracket
    if bracket is not None:
        record.update(
            attitude_bracket_time_boot_s=[bracket.earlier_time_boot_s, bracket.later_time_boot_s],
            attitude_bracket_receipts_s=[bracket.earlier_receipt_s, bracket.later_receipt_s],
            attitude_weight=bracket.fraction,
        )
    return record


__all__ = [
    "EVENT_ASSOCIATION",
    "EVENT_ATTITUDE_RECEIPT",
    "EVENT_GIMBAL_RECEIPT",
    "ReceiptLog",
    "TimingLog",
    "attitude_history_record",
    "attitude_record",
    "gimbal_history_record",
    "mapping_record",
]
