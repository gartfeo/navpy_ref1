"""Capture-domain consistency gate for real-detector frame association.

Split from `real_frame_association.py`: this module owns WHAT makes an
association frame-atomic (decision doc §3e); the builder owns freezing the
state. The pose criterion is strict bracketing at the capture instant; the
frame-age check keeps only the absolute sanity ceiling, so a calibrated
high-latency source passes CONSISTENCY here while measurement-age admission
stays downstream in the detector freshness policy.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

from navpy.modules.vehicle.pose_streams import (
    POSE_FRAME_ASSOCIATION_ABSOLUTE_MAX_SKEW_S,
    POSE_FRAME_ASSOCIATION_MAX_SKEW_S,
)
from navpy.modules.vision.camera_mount import CameraMountFrameState
from navpy.modules.vision.frame_capture_ports import CaptureStampKind


@dataclass(frozen=True)
class CaptureTiming:
    """Capture-domain times for one frame (decision doc §2/§3d).

    `estimate_s` is the geometry/measurement time (best exposure estimate);
    `lookup_wall_s` is where the de-rotating attitude was interpolated on the
    ATTITUDE receipt axis. `frame_timestamp_s` on the association stays the
    PUBLICATION time and keeps receipt-liveness semantics.
    """

    kind: CaptureStampKind
    captured_at_s: float
    lookup_wall_s: float
    estimate_s: float


def finite_time(value: float | int | None) -> float | None:
    if isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def finite_body_rates(
    value: Sequence[float] | None,
) -> tuple[float, float, float] | None:
    if value is None:
        return None
    try:
        rates = (float(value[0]), float(value[1]), float(value[2]))
    except (IndexError, TypeError, ValueError):
        return None
    return rates if all(math.isfinite(rate) for rate in rates) else None


def real_frame_association_quality(
        *,
        capture: CaptureTiming | None,
        capture_status: str,
        associated_at_s: float,
        mount_state: CameraMountFrameState | None,
        max_skew_s: float = POSE_FRAME_ASSOCIATION_MAX_SKEW_S,
) -> tuple[bool, str, float | None]:
    """Consistency gate in the CAPTURE domain (decision doc §3e)."""
    if not math.isfinite(max_skew_s) or max_skew_s <= 0.0:
        max_skew_s = POSE_FRAME_ASSOCIATION_MAX_SKEW_S
    if capture is None:
        return False, capture_status, None
    frame_age_s = associated_at_s - capture.estimate_s
    if frame_age_s < 0.0 or (
            frame_age_s > POSE_FRAME_ASSOCIATION_ABSOLUTE_MAX_SKEW_S
            and not math.isclose(
                frame_age_s,
                POSE_FRAME_ASSOCIATION_ABSOLUTE_MAX_SKEW_S,
                rel_tol=1e-12,
                abs_tol=1e-12,
            )
    ):
        return False, "real_frame_publication_stale", abs(frame_age_s)
    if capture_status != "bracketed":
        return False, capture_status, frame_age_s
    if mount_state is None:
        return False, "real_frame_mount_snapshot_unavailable", frame_age_s

    skews = [frame_age_s]
    if not mount_state.gimbal_is_static or mount_state.zoom_command is not None:
        # A dynamic MOUNT is a moving gimbal OR a zoom-capable camera: both
        # sample their state at association time
        # (`camera_mount_frame_state.py:55`), which cannot describe the
        # capture instant -- proximity does not prove the association-time
        # zoom matches exposure-time optics, and the approved contract
        # (decision doc §3d) admits no window. Fail closed for EVERY stamp
        # kind until a capture-relative mount-state contract exists; a truly
        # fixed camera (no zoom command) has timeless optics and is exempt.
        return False, "real_frame_mount_dynamic_unsupported", frame_age_s

    if mount_state.zoom_sample_id is None:
        return False, "real_frame_zoom_timestamp_unavailable", max(skews)
    zoom_age_s = finite_time(mount_state.zoom_sample_age_s)
    if zoom_age_s is None or zoom_age_s < 0.0:
        return False, "real_frame_zoom_timestamp_unavailable", max(skews)
    if zoom_age_s > max_skew_s:
        return False, "real_frame_zoom_skew", max(*skews, zoom_age_s)
    skews.append(zoom_age_s)
    return True, "real_frame_pose", max(skews)


__all__ = [
    "CaptureTiming",
    "finite_body_rates",
    "finite_time",
    "real_frame_association_quality",
]
