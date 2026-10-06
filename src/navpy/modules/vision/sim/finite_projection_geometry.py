"""Finite simulator reference bounding box; navigation does not consume truth."""
from __future__ import annotations
from navpy.modules.vision.vision_class_profile import get_detector_class_dimensions


def tracking_bbox(pixel_ok: bool, focal_y: float, distance_m: float,
                  x_error: float, y_error: float) -> tuple[float, float, float, float] | None:
    if not pixel_ok:
        return None
    width_m, height_m = get_detector_class_dimensions(0)
    return (float(x_error), float(y_error), focal_y * width_m / distance_m,
            focal_y * height_m / distance_m)
