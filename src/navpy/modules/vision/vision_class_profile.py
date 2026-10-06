"""Detector class catalog and range math shared by vision profile consumers."""

from __future__ import annotations

import json
import math
from collections.abc import Mapping
from pathlib import Path


MIN_DETECT_PIXELS = 8
MIN_TRACK_PIXELS = 12
MIN_CONFIRM_PIXELS = 20
_DEFAULT_DETECTOR_CLASS_DIMENSIONS = (2.0, 2.0)
_DETECT_RANGE_MARGIN = 1.1
CONFIRM_Y_BUDGET = 0.30
DOCK_CLASS_TO_DETECT_ID = {
    "small": 4,
    "medium": 0,
    "large": 0,
}


def _diagonal_m(dims: tuple[float, float]) -> float:
    return math.hypot(dims[0], dims[1])


def _load_detector_class_dimensions() -> dict[int, tuple[float, float]]:
    """Load the single detector-dimension catalog and fail fast if malformed."""
    path = Path(__file__).with_name("vision_profiles.json")
    raw = json.loads(path.read_text(encoding="utf-8"))
    blocks = raw.get("detector_class_dimensions") if isinstance(raw, Mapping) else None
    if not isinstance(blocks, Mapping) or not blocks:
        raise ValueError(
            "vision_profiles.json is missing the 'detector_class_dimensions' block"
        )

    dims: dict[int, tuple[float, float]] = {}
    for raw_class_id, raw_entry in blocks.items():
        if not isinstance(raw_entry, Mapping):
            raise ValueError(
                f"invalid detector_class_dimensions entry {raw_class_id!r}: must be a mapping"
            )
        try:
            class_id = int(raw_class_id)
            width = float(raw_entry["width_m"])
            height = float(raw_entry["height_m"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(
                f"invalid detector_class_dimensions entry {raw_class_id!r}: {exc}"
            ) from exc
        if class_id in dims:
            raise ValueError(f"duplicate detector class id {class_id}")
        if not (
            math.isfinite(width)
            and math.isfinite(height)
            and width > 0.0
            and height > 0.0
        ):
            raise ValueError(
                f"detector_class_dimensions entry {raw_class_id!r} must have positive "
                f"finite width_m/height_m, got ({width}, {height})"
            )
        dims[class_id] = (width, height)

    missing = {0, 4} - dims.keys()
    if missing:
        raise ValueError(
            f"detector_class_dimensions missing required class id(s) {sorted(missing)}"
        )
    return dims


DETECTOR_CLASS_DIMENSIONS = _load_detector_class_dimensions()
CLASS_DETECT_SIZES = {
    class_id: _diagonal_m(dims)
    for class_id, dims in DETECTOR_CLASS_DIMENSIONS.items()
}
_MIN_CLASS_SIZE = min(CLASS_DETECT_SIZES.values())
_MAX_CLASS_SIZE = max(CLASS_DETECT_SIZES.values())


def get_class_detect_size(class_id: int) -> float:
    """Return the physical bbox-diagonal size in metres for a class."""
    return CLASS_DETECT_SIZES.get(class_id, _diagonal_m(_DEFAULT_DETECTOR_CLASS_DIMENSIONS))


def get_detector_class_dimensions(class_id: int) -> tuple[float, float]:
    """Return physical ``(width_m, height_m)`` for a detector class."""
    return DETECTOR_CLASS_DIMENSIONS.get(class_id, _DEFAULT_DETECTOR_CLASS_DIMENSIONS)


def _optional_mapping(value: object, name: str) -> Mapping[str, object]:
    if value is None:
        return {}
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be a mapping")
    return value


def get_min_pixels_for_class(
    profile: Mapping[str, object],
    class_id: int,
) -> float:
    """Resolve the strictest valid profile confirmation gate for a class."""
    if not isinstance(profile, Mapping):
        raise ValueError("vision profile must be a mapping")
    detector = _optional_mapping(profile.get("detector"), "detector")
    presets = _optional_mapping(
        detector.get("dock_presets"),
        "detector.dock_presets",
    )

    candidates: list[float] = []
    for preset_name, raw_preset in presets.items():
        if DOCK_CLASS_TO_DETECT_ID.get(preset_name) != class_id:
            continue
        if not isinstance(raw_preset, Mapping):
            continue
        raw_pixels = raw_preset.get("min_pixel_size")
        if raw_pixels is None or isinstance(raw_pixels, bool):
            continue
        try:
            pixels = float(raw_pixels)
        except (TypeError, ValueError):
            continue
        if math.isfinite(pixels) and pixels > 0.0:
            candidates.append(pixels)

    return max(candidates, default=float(MIN_CONFIRM_PIXELS))


def _positive_finite(value: object, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a positive finite number")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a positive finite number") from exc
    if not math.isfinite(number) or number <= 0.0:
        raise ValueError(f"{name} must be a positive finite number")
    return number


def compute_detect_slant_range(
    fy: float,
    class_size: float,
    min_pixels: float = MIN_DETECT_PIXELS,
) -> float:
    """Return maximum raw detection slant range for a detector class."""
    return (
        _positive_finite(fy, "fy")
        * _positive_finite(class_size, "class_size")
        / _positive_finite(min_pixels, "min_pixels")
    )


def compute_confirm_slant_range(
    fy: float,
    class_size: float = _MIN_CLASS_SIZE,
    min_pixels: float = MIN_CONFIRM_PIXELS,
) -> float:
    """Return maximum conservative confirmation slant range."""
    return compute_detect_slant_range(fy, class_size, min_pixels)


def compute_max_detect_dist(fy: float) -> float:
    """Return largest-class detection range with the transient margin."""
    return (
        compute_detect_slant_range(fy, _MAX_CLASS_SIZE, MIN_DETECT_PIXELS)
        * _DETECT_RANGE_MARGIN
    )


def compute_approach_interval(
    fy: float,
    cy: float,
    img_h: float,
    pitch_deg: float,
    alt: float,
    class_size: float,
) -> tuple[float, float] | None:
    """Return the valid confirmation ground-distance interval, if any."""
    try:
        fy_value = float(fy)
        cy_value = float(cy)
        image_height = float(img_h)
        pitch = float(pitch_deg)
        altitude = float(alt)
        target_size = float(class_size)
    except (TypeError, ValueError):
        return None
    values = (fy_value, cy_value, image_height, pitch, altitude, target_size)
    if not all(math.isfinite(value) for value in values):
        return None
    if (
        fy_value <= 0.0
        or image_height <= 0.0
        or pitch <= 0.0
        or altitude <= 0.0
        or target_size <= 0.0
    ):
        return None

    confirm_slant = fy_value * target_size / MIN_CONFIRM_PIXELS
    if confirm_slant <= altitude:
        return None
    ground_max = math.sqrt(confirm_slant**2 - altitude**2)

    max_y_pct = 0.5 + CONFIRM_Y_BUDGET
    pixels_from_cy = max_y_pct * image_height - cy_value
    if pixels_from_cy <= 0.0:
        ground_min = 0.0
    else:
        total_angle = math.radians(pitch) + math.atan(pixels_from_cy / fy_value)
        ground_min = (
            0.0 if total_angle <= 0.0 else altitude / math.tan(total_angle)
        )
    if not math.isfinite(ground_min) or ground_min > ground_max:
        return None
    return ground_min, ground_max


__all__ = [
    "CLASS_DETECT_SIZES",
    "DETECTOR_CLASS_DIMENSIONS",
    "CONFIRM_Y_BUDGET",
    "MIN_CONFIRM_PIXELS",
    "MIN_DETECT_PIXELS",
    "MIN_TRACK_PIXELS",
    "DOCK_CLASS_TO_DETECT_ID",
    "compute_approach_interval",
    "compute_confirm_slant_range",
    "compute_detect_slant_range",
    "compute_max_detect_dist",
    "get_class_detect_size",
    "get_detector_class_dimensions",
    "get_min_pixels_for_class",
]
