"""Frame association on explicit integration timing (HITL stage 1 step 11).

The integration counterpart of the certified ``_capture_lookup``: the frame's
mapped receiver time (``camera/integration_timing.py``) plus each source's
correction C places the ATTITUDE and gimbal lookups, both strictly bracketed.
Everything it admits carries ``AUTHORITY_INTEGRATION``, never certified. Every
attempt, admitted or refused, writes one timing record naming the samples and
weights used.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass, replace
from types import SimpleNamespace

from navpy.core.kernel.attitude import Attitude
from navpy.core.perception.camera.camera_mount_types import CameraMountFrameState
from navpy.core.perception.camera.capture_interpolation import (
    AttitudeAtLookup,
    interpolate_attitude_at,
)
from navpy.core.perception.camera.integration_timing import (
    AUTHORITY_INTEGRATION,
    STATUS_ABORTED,
    STATUS_MAPPED,
    IntegrationTimingConfig,
)
from navpy.core.perception.camera.pts_read_map import IntegrationTimingSource, MappedFrame
from navpy.core.perception.pipeline.real_frame_association_gate import CaptureTiming
from navpy.core.ports.frame_capture_ports import CaptureStamp
from navpy.core.ports.gimbal_capture_history import (
    GimbalCaptureHistory,
    GimbalHistorySample,
)

STATUS_BRACKETED = "bracketed"
STATUS_TIMESTAMP_UNAVAILABLE = "real_frame_capture_timestamp_unavailable"
STATUS_CONTEXT_MISMATCH = "real_frame_integration_context_mismatch"
STATUS_FRAME_STALE = "real_frame_integration_stale"
STATUS_SOURCE_STALE = "real_frame_integration_source_stale"
STATUS_ZOOM_UNSETTLED = "real_frame_zoom_unsettled"
STATUS_GIMBAL_HISTORY_UNAVAILABLE = "real_frame_gimbal_history_unavailable"
STATUS_GIMBAL_UNBRACKETED = "real_frame_gimbal_unbracketed"
STATUS_GIMBAL_GAP = "real_frame_gimbal_gap"

TimingLog = Callable[[dict[str, object]], None]


def _lerp_angle_deg(first: float, second: float, fraction: float) -> float:
    delta = (second - first + 180.0) % 360.0 - 180.0
    return first + delta * fraction


@dataclass(frozen=True)
class GimbalAtLookup:
    """Gimbal angles interpolated between two adjacent readbacks of one session."""

    attitude: Attitude
    session: int
    earlier: GimbalHistorySample
    later: GimbalHistorySample
    fraction: float


def interpolate_gimbal_at(
    history: GimbalCaptureHistory | None,
    lookup_s: float,
    bracket_max_s: float,
) -> tuple[GimbalAtLookup | None, str]:
    """Strictly bracketed gimbal angles at ``lookup_s`` on the receipt axis, or a refusal."""
    if not isinstance(history, GimbalCaptureHistory):
        return None, STATUS_GIMBAL_HISTORY_UNAVAILABLE
    samples = [sample for sample in history.samples if sample.session == history.session]
    for earlier, later in zip(samples, samples[1:]):
        if not earlier.receipt_s <= lookup_s <= later.receipt_s:
            continue
        width = later.receipt_s - earlier.receipt_s
        if width > bracket_max_s:
            return None, STATUS_GIMBAL_GAP
        fraction = 0.0 if width <= 0.0 else (lookup_s - earlier.receipt_s) / width
        attitude = Attitude(
            pitch=_lerp_angle_deg(earlier.att.pitch, later.att.pitch, fraction),
            yaw=_lerp_angle_deg(earlier.att.yaw, later.att.yaw, fraction),
            roll=_lerp_angle_deg(earlier.att.roll, later.att.roll, fraction),
        )
        return GimbalAtLookup(attitude, history.session, earlier, later, fraction), STATUS_BRACKETED
    return None, STATUS_GIMBAL_UNBRACKETED


@dataclass(frozen=True)
class IntegrationOutcome:
    timing: CaptureTiming | None
    status: str
    attitude: AttitudeAtLookup | None
    mount_state: CameraMountFrameState | None


class IntegrationAssociation:
    """Maps one frame and brackets its attitude and gimbal state, or names the refusal."""

    def __init__(
        self,
        source: IntegrationTimingSource,
        gimbal_history_reader: Callable[[], GimbalCaptureHistory | None],
        log: TimingLog | None = None,
    ) -> None:
        self._source = source
        self._gimbal_history_reader = gimbal_history_reader
        self._log = log

    @property
    def config(self) -> IntegrationTimingConfig:
        return self._source.config

    def associate(
        self,
        stamp: CaptureStamp | None,
        source_id: str,
        link_id: object,
        attitude_history: Sequence[SimpleNamespace],
        mount_state: CameraMountFrameState | None,
        now_s: float,
        attitude_bracket_max_s: float,
        seam_tolerance_s: float,
    ) -> IntegrationOutcome:
        record: dict[str, object] = {"run_id": self.config.run_id, "authority": AUTHORITY_INTEGRATION}
        outcome = self._associate(
            record, stamp, source_id, link_id, attitude_history, mount_state,
            now_s, attitude_bracket_max_s, seam_tolerance_s,
        )
        if outcome.status == STATUS_BRACKETED and self._source.aborted is not None:
            # The capture thread aborted the map while this frame was being
            # associated: refuse it like every later frame.
            outcome = IntegrationOutcome(outcome.timing, STATUS_ABORTED, None, outcome.mount_state)
        record["status"] = outcome.status
        if self._log is not None:
            self._log(record)
        return outcome

    def _associate(
        self,
        record: dict[str, object],
        stamp: CaptureStamp | None,
        source_id: str,
        link_id: object,
        attitude_history: Sequence[SimpleNamespace],
        mount_state: CameraMountFrameState | None,
        now_s: float,
        attitude_bracket_max_s: float,
        seam_tolerance_s: float,
    ) -> IntegrationOutcome:
        config = self.config
        limits = config.limits
        if stamp is None or stamp.source is None:
            return IntegrationOutcome(None, STATUS_TIMESTAMP_UNAVAILABLE, None, mount_state)
        record.update(open_id=stamp.source.open_id, frame_sequence=stamp.source.sequence, pts_ns=stamp.source.pts_ns)
        if source_id != config.source_id or link_id != config.link_id:
            return IntegrationOutcome(None, STATUS_CONTEXT_MISMATCH, None, mount_state)
        mapped, status = self._source.frame(stamp.source)
        if status != STATUS_MAPPED or mapped is None:
            return IntegrationOutcome(None, status, None, mount_state)
        record.update(_mapping_record(mapped))
        attitude_lookup_s = mapped.mapped_s + config.attitude_correction_s
        timing = CaptureTiming(
            kind=stamp.kind,
            captured_at_s=stamp.captured_at_s,
            lookup_s=attitude_lookup_s,
            estimate_s=mapped.mapped_s,
            authority=AUTHORITY_INTEGRATION,
            interval_s=mapped.interval_s,
        )
        record["attitude_lookup_s"] = attitude_lookup_s
        if now_s - mapped.mapped_s > limits.max_frame_age_s:
            return IntegrationOutcome(timing, STATUS_FRAME_STALE, None, mount_state)
        receipts = [getattr(sample, "capture_receipt_s", None) for sample in attitude_history]
        newest = max((value for value in receipts if _finite(value)), default=None)
        if newest is None or now_s - newest > limits.max_source_age_s:
            return IntegrationOutcome(timing, STATUS_SOURCE_STALE, None, mount_state)
        # A clock seam within the map's own window before the lookup refuses.
        attitude, status = interpolate_attitude_at(
            attitude_history,
            attitude_lookup_s,
            limits.window_s,
            bracket_max_width_s=attitude_bracket_max_s,
            seam_tolerance_s=seam_tolerance_s,
        )
        if attitude is None:
            return IntegrationOutcome(timing, status, None, mount_state)
        record.update(_attitude_record(attitude))
        if mount_state is None:
            return IntegrationOutcome(timing, STATUS_BRACKETED, attitude, None)
        record["zoom_sample_id"] = mount_state.zoom_sample_id
        if mount_state.zoom_command is not None:
            # No settled-zoom evidence exists before step 18: zoom-capable
            # mounts stay refused.
            return IntegrationOutcome(timing, STATUS_ZOOM_UNSETTLED, attitude, mount_state)
        if mount_state.gimbal_is_static:
            return IntegrationOutcome(timing, STATUS_BRACKETED, attitude, mount_state)
        return self._gimbal(record, timing, attitude, mount_state, mapped, now_s)

    def _gimbal(
        self,
        record: dict[str, object],
        timing: CaptureTiming,
        attitude: AttitudeAtLookup,
        mount_state: CameraMountFrameState,
        mapped: MappedFrame,
        now_s: float,
    ) -> IntegrationOutcome:
        limits = self.config.limits
        gimbal_lookup_s = mapped.mapped_s + self.config.gimbal_correction_s
        record["gimbal_lookup_s"] = gimbal_lookup_s
        history = self._gimbal_history_reader()
        if isinstance(history, GimbalCaptureHistory) and history.samples:
            if now_s - history.samples[-1].receipt_s > limits.max_source_age_s:
                return IntegrationOutcome(timing, STATUS_SOURCE_STALE, attitude, mount_state)
        gimbal, status = interpolate_gimbal_at(history, gimbal_lookup_s, limits.gimbal_bracket_max_s)
        if gimbal is None:
            return IntegrationOutcome(timing, status, attitude, mount_state)
        record.update(
            gimbal_session=gimbal.session,
            gimbal_seqs=[gimbal.earlier.seq, gimbal.later.seq],
            gimbal_receipts_s=[gimbal.earlier.receipt_s, gimbal.later.receipt_s],
            gimbal_weight=gimbal.fraction,
        )
        captured = replace(
            mount_state,
            gimbal_data=replace(mount_state.gimbal_data, att=gimbal.attitude, timestamp_s=gimbal_lookup_s),
            gimbal_timestamp_s=gimbal_lookup_s,
            gimbal_capture_relative=True,
        )
        return IntegrationOutcome(timing, STATUS_BRACKETED, attitude, captured)


def _finite(value: object) -> bool:
    return (
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(value)
    )


def _mapping_record(mapped: MappedFrame) -> dict[str, object]:
    return {
        "read_s": mapped.read_s,
        "mapped_s": mapped.mapped_s,
        "offset_s": mapped.offset_s,
        "envelope_s": mapped.envelope_s,
        "rate": mapped.rate,
    }


def _attitude_record(attitude: AttitudeAtLookup) -> dict[str, object]:
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
    "GimbalAtLookup",
    "IntegrationAssociation",
    "IntegrationOutcome",
    "STATUS_CONTEXT_MISMATCH",
    "STATUS_FRAME_STALE",
    "STATUS_GIMBAL_GAP",
    "STATUS_GIMBAL_HISTORY_UNAVAILABLE",
    "STATUS_GIMBAL_UNBRACKETED",
    "STATUS_SOURCE_STALE",
    "STATUS_ZOOM_UNSETTLED",
    "TimingLog",
    "interpolate_gimbal_at",
]
