from __future__ import annotations

import math
from typing import Iterable, Optional, TYPE_CHECKING

from navpy.modules.vision.target_identity import get_target_task_id

if TYPE_CHECKING:
    from navpy.modules.vision.models.detect_data import DetectedObject


def select_most_centered_target(targets: Iterable["DetectedObject"]) -> Optional["DetectedObject"]:
    """Return the detection closest to its image principal point."""
    target_list = list(targets)
    if not target_list:
        return None

    best = None
    best_error = float("inf")
    for target in target_list:
        error = _image_center_error(target)
        if error < best_error:
            best = target
            best_error = error

    return best if best is not None else target_list[0]


def prioritize_targets(
        targets: Iterable["DetectedObject"],
        primary_target: Optional["DetectedObject"],
) -> list["DetectedObject"]:
    """Return targets with the primary target first while preserving peers."""
    ordered = list(targets)
    if primary_target is None:
        return ordered

    for index, target in enumerate(ordered):
        if target is primary_target:
            if index == 0:
                return ordered
            return [target] + ordered[:index] + ordered[index + 1:]

    return ordered


def find_target_by_task_id(
        targets: Iterable["DetectedObject"],
        task_id: Optional[int],
) -> Optional["DetectedObject"]:
    """Find the first detection with the requested navigation task identity."""
    if task_id is None:
        return None

    for target in targets:
        if get_target_task_id(target) == task_id:
            return target
    return None


def find_target_by_id(
        targets: Iterable["DetectedObject"],
        obj_id: Optional[int],
) -> Optional["DetectedObject"]:
    """Backward-compatible alias for task-identity lookup."""
    return find_target_by_task_id(targets, obj_id)


def _image_center_error(target: "DetectedObject") -> float:
    try:
        calibration = target.pixel.calibration
        return math.hypot(
            float(target.pixel.u_px) - calibration.cx_px,
            float(target.pixel.v_px) - calibration.cy_px,
        )
    except (AttributeError, TypeError, ValueError):
        return float("inf")
