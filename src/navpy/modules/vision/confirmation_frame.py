"""Confirmation-frame ranking shared by real and simulated detectors."""
from __future__ import annotations

import math
from typing import Optional, Sequence, Tuple

from navpy.modules.vision.poi_size import characteristic_pixels


BBox = Tuple[float, float, float, float]
CONFIRMATION_FRAME_MAX_CENTER_SCORE = 0.4


def bbox_recognizable_size(bbox: Optional[Sequence[float]]) -> float:
    """Recognizable bbox extent — the diagonal ``sqrt(w**2+h**2)`` — used
    by the CONFIRM pixel gate and frame ranking. Returns 0.0 (lowest rank)
    when the bbox is missing or degenerate, so ranking never raises."""
    if bbox is None or len(bbox) < 4:
        return 0.0
    try:
        width = float(bbox[2])
        height = float(bbox[3])
    except (TypeError, ValueError):
        return 0.0
    if not (math.isfinite(width) and math.isfinite(height)):
        return 0.0
    if width <= 0.0 or height <= 0.0:
        return 0.0
    return characteristic_pixels(width, height)


def is_confirmation_frame_centered(
    center_score: float,
    max_center_score: float = CONFIRMATION_FRAME_MAX_CENTER_SCORE,
) -> bool:
    """Return whether a sim frame is centered enough to be sent."""
    return float(center_score) < max_center_score


def confirmation_frame_rank(
    bbox: Optional[Sequence[float]],
    center_score: float,
    context_count: int = 0,
) -> tuple[float, float, int]:
    """Rank by recognizable size (bbox diagonal), then centering, then
    visible context."""
    return (
        bbox_recognizable_size(bbox),
        -float(center_score),
        int(context_count),
    )


def should_replace_confirmation_frame(
    candidate_bbox: BBox,
    candidate_center_score: float,
    stored_bbox: Optional[Sequence[float]],
    stored_center_score: float = 0.0,
    *,
    candidate_context_count: int = 0,
    stored_context_count: int = 0,
) -> bool:
    """Return true when candidate is a better confirmation source frame."""
    if stored_bbox is None:
        return True
    return confirmation_frame_rank(
        candidate_bbox, candidate_center_score, candidate_context_count,
    ) > confirmation_frame_rank(
        stored_bbox, stored_center_score, stored_context_count,
    )


def should_replace_center_gated_confirmation_frame(
    candidate_bbox: BBox,
    candidate_center_score: float,
    stored_bbox: Optional[Sequence[float]],
    stored_center_score: float = 0.0,
    *,
    candidate_context_count: int = 0,
    stored_context_count: int = 0,
    max_center_score: float = CONFIRMATION_FRAME_MAX_CENTER_SCORE,
) -> bool:
    """Rank sim frames while preserving the existing centered-frame gate."""
    if stored_bbox is None:
        return True

    candidate_centered = is_confirmation_frame_centered(
        candidate_center_score, max_center_score,
    )
    stored_centered = is_confirmation_frame_centered(
        stored_center_score, max_center_score,
    )
    if candidate_centered != stored_centered:
        return candidate_centered

    return should_replace_confirmation_frame(
        candidate_bbox,
        candidate_center_score,
        stored_bbox,
        stored_center_score,
        candidate_context_count=candidate_context_count,
        stored_context_count=stored_context_count,
    )
