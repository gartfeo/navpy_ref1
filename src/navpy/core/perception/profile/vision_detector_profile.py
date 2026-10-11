"""Detector settings composed from a vision profile."""

from __future__ import annotations

from collections.abc import Mapping


DEFAULT_DETECTOR_SETTINGS = {
    "detect_hz": 15.0,
    "track_hz": 60.0,
    "reference_height_m": 2.0,
    "imgsz": 640,
    "conf": 0.35,
    "device": "auto",
    "ideal_360": False,
}


def validate_detector_settings_keys(settings: Mapping[str, object]) -> None:
    """Reject retired field spellings before they can select defaults."""
    if "poi_height" in settings:
        raise ValueError("retired detector field poi_height; use reference_height_m")


def get_detector_settings(profile: Mapping[str, object]) -> dict:
    """Return detector settings, rejecting malformed explicit blocks."""
    if not isinstance(profile, Mapping):
        raise ValueError("Vision profile must be a mapping")
    settings = dict(DEFAULT_DETECTOR_SETTINGS)
    profile_settings = profile.get("detector")
    if profile_settings is None:
        return settings
    if not isinstance(profile_settings, Mapping):
        raise ValueError("Vision profile detector must be a mapping")
    validate_detector_settings_keys(profile_settings)
    settings.update(profile_settings)
    return settings


__all__ = ["DEFAULT_DETECTOR_SETTINGS", "get_detector_settings", "validate_detector_settings_keys"]
