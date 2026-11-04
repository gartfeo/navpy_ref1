"""Settled zoom at a frame's gimbal lookup, for the integration association (HITL step 18).

A zoom-capable mount is admitted only when the fixed-zoom settle evidence (``zoom/zoom_settle_evidence.py``) proves
the lens settled at a calibrated step at the lookup; the frame then takes that step's measured row (K and
distortion) instead of the association-time optics, and must be of the table's stream size.
"""

from __future__ import annotations

from dataclasses import replace

from navpy.core.perception.camera.camera_mount_types import CameraMountFrameState
from navpy.core.perception.pipeline.integration_log import EVENT_ZOOM_CONFIG
from navpy.core.perception.zoom.fixed_step_zoom_drive import FixedStepZoom
from navpy.core.ports.gimbal_capture_history import GimbalCaptureHistory

STATUS_ZOOM_STREAM_UNMEASURED = "real_frame_zoom_stream_unmeasured"


def settled_zoom_state(
    fixed: FixedStepZoom,
    record: dict[str, object],
    mount_state: CameraMountFrameState,
    history: GimbalCaptureHistory | None,
    lookup_s: float,
    now_s: float,
    frame_size: tuple[int, int] | None,
) -> tuple[CameraMountFrameState | None, str]:
    """The mount state with the settled step's measured optics, or the refusal.

    Judged at the gimbal lookup: the zoom readings share the gimbal's
    source link, so its correction places them on the frame.
    """
    judgement = fixed.evidence.judge(history, lookup_s, now_s)
    record["zoom_lookup_s"] = lookup_s
    record.update(judgement.record())
    if not judgement.settled or judgement.step is None:
        return None, judgement.status
    stream = fixed.steps.stream
    if frame_size != (stream.image_width, stream.image_height):
        return None, STATUS_ZOOM_STREAM_UNMEASURED
    optics = judgement.step.optics
    k = optics.k()
    dist = optics.dist_coeffs()
    k.setflags(write=False)
    dist.setflags(write=False)
    return replace(
        mount_state,
        k=k,
        dist=dist,
        zoom_command=judgement.step.command,
        zoom_sample_id=f"zoom:{judgement.session}:{judgement.arrival_seq}:{judgement.step.command}",
        # The step is proven at the lookup instant itself (held readings on both sides).
        zoom_sample_age_s=0.0,
        zoom_capture_relative=True,
    ), judgement.status


def zoom_config_record(run_id: str, fixed: FixedStepZoom) -> dict[str, object]:
    """Freezes the fixed-zoom steps and settle limits into the timing log, once per association."""
    limits = fixed.evidence.limits
    stream = fixed.steps.stream
    return {
        "event": EVENT_ZOOM_CONFIG, "run_id": run_id,
        "stream": [stream.mode, stream.image_width, stream.image_height, list(stream.crop)],
        "steps": [[step.command, step.optics.reading] for step in fixed.steps.steps],
        "dwell_s": limits.dwell_s, "min_held_readings": limits.min_held_readings,
        "reading_gap_max_s": limits.reading_gap_max_s, "reading_age_max_s": limits.reading_age_max_s,
    }


__all__ = ["STATUS_ZOOM_STREAM_UNMEASURED", "settled_zoom_state", "zoom_config_record"]
