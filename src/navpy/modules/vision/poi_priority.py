from __future__ import annotations

import math
from typing import Iterable, Optional, TYPE_CHECKING

from navpy.modules.vision.poi_identity import get_poi_task_id

if TYPE_CHECKING:
    from navpy.modules.vision.models.detect_data import DetectedObject


def select_most_centered_poi(pois: Iterable["DetectedObject"]) -> Optional["DetectedObject"]:
    """Return the detection closest to its image principal point."""
    poi_list = list(pois)
    if not poi_list:
        return None

    best = None
    best_error = float("inf")
    for poi in poi_list:
        error = _image_center_error(poi)
        if error < best_error:
            best = poi
            best_error = error

    return best if best is not None else poi_list[0]


def prioritize_pois(
        pois: Iterable["DetectedObject"],
        primary_poi: Optional["DetectedObject"],
) -> list["DetectedObject"]:
    """Return POIs with the primary POI first while preserving peers."""
    ordered = list(pois)
    if primary_poi is None:
        return ordered

    for index, poi in enumerate(ordered):
        if poi is primary_poi:
            if index == 0:
                return ordered
            return [poi] + ordered[:index] + ordered[index + 1:]

    return ordered


def find_poi_by_task_id(
        pois: Iterable["DetectedObject"],
        task_id: Optional[int],
) -> Optional["DetectedObject"]:
    """Find the first detection with the requested navigation task identity."""
    if task_id is None:
        return None

    for poi in pois:
        if get_poi_task_id(poi) == task_id:
            return poi
    return None


def find_poi_by_id(
        pois: Iterable["DetectedObject"],
        obj_id: Optional[int],
) -> Optional["DetectedObject"]:
    """Backward-compatible alias for task-identity lookup."""
    return find_poi_by_task_id(pois, obj_id)


def _image_center_error(poi: "DetectedObject") -> float:
    try:
        calibration = poi.pixel.calibration
        return math.hypot(
            float(poi.pixel.u_px) - calibration.cx_px,
            float(poi.pixel.v_px) - calibration.cy_px,
        )
    except (AttributeError, TypeError, ValueError):
        return float("inf")
