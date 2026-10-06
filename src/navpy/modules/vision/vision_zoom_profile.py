"""Source-driven target-zoom enablement from a vision profile."""

from __future__ import annotations

from collections.abc import Mapping

from navpy.modules.vision.target_zoom_types import TargetZoomTrackerConfig
from navpy.modules.vision.vision_class_profile import (
    CLASS_DETECT_SIZES,
    MIN_CONFIRM_PIXELS,
    get_min_pixels_for_class,
)


def _target_pixels(
    profile: Mapping[str, object] | None,
) -> dict[str, float]:
    if profile is None:
        return {"default": float(MIN_CONFIRM_PIXELS)}
    thresholds = {
        str(class_id): get_min_pixels_for_class(profile, class_id)
        for class_id in CLASS_DETECT_SIZES
    }
    thresholds["default"] = float(MIN_CONFIRM_PIXELS)
    return thresholds


def build_zoom_config(
    device: Mapping[str, object],
    profile: Mapping[str, object] | None = None,
) -> TargetZoomTrackerConfig | None:
    """Enable zoom; all thresholds and hardware limits come from owners."""
    if not isinstance(device, Mapping):
        raise ValueError("Vision profile device must be a mapping")
    raw_gimbal = device.get("gimbal", {})
    if not isinstance(raw_gimbal, Mapping):
        raise ValueError("Vision profile device gimbal must be a mapping")
    raw_zoom = raw_gimbal.get("zoom")
    if raw_zoom is None:
        return None
    if not isinstance(raw_zoom, Mapping):
        raise ValueError("gimbal.zoom must be a mapping")
    unknown = set(raw_zoom) - {"enabled"}
    if unknown:
        raise ValueError(f"Unknown gimbal.zoom key(s): {sorted(unknown)}")
    enabled = raw_zoom.get("enabled", False)
    if not isinstance(enabled, bool):
        raise ValueError("gimbal.zoom.enabled must be boolean")
    if not enabled:
        return None
    return TargetZoomTrackerConfig(target_pixels=_target_pixels(profile))


__all__ = ["build_zoom_config"]
