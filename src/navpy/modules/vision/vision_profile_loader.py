"""Vision profile JSON loading, inheritance, and device extraction."""

from __future__ import annotations

import json
from collections.abc import Mapping
from pathlib import Path

from navpy.logger.cache_logger import ILogger
from navpy.modules.vision.vision_profile_inheritance import (
    ProfileInheritanceResolver,
    merge_profile_dict as _merge_profile_dict,
)
from navpy.modules.vision.vision_profile_types import VisionProfile
from navpy.modules.vision.vision_detector_profile import validate_detector_settings_keys
from navpy.modules.vision.vision_class_profile import DOCK_PRESET_NAME


def _profiles_path() -> Path:
    return Path(__file__).with_name("vision_profiles.json")


def _resolve_profile_definitions(raw_profiles: dict) -> dict:
    """Materialize profile inheritance without mutating JSON definitions."""
    return ProfileInheritanceResolver(raw_profiles).resolve_all()


def load_profiles() -> tuple[dict, str, Path]:
    path = _profiles_path()
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, Mapping):
        raise ValueError("Vision profiles root must be a JSON object")
    if "poi_classes" in data:
        raise ValueError(f"{path}: retired profile field poi_classes; use detector_class_dimensions")
    profiles = _resolve_profile_definitions(data.get("profiles", {}))
    for name, profile in profiles.items():
        detector = profile.get("detector")
        if detector is None:
            continue
        if not isinstance(detector, Mapping):
            raise ValueError(f"{path}: profile {name!r} detector must be a mapping")
        validate_detector_settings_keys(detector)
        if "poi_presets" in detector:
            raise ValueError(f"{path}: profile {name!r} has retired profile field poi_presets; use dock_presets")
        presets = detector.get("dock_presets")
        if presets is None:
            continue
        if not isinstance(presets, Mapping):
            raise ValueError(f"{path}: profile {name!r} dock_presets must be a mapping")
        unknown = presets.keys() - {DOCK_PRESET_NAME}
        if unknown:
            raise ValueError(f"{path}: profile {name!r} has unknown sizing presets: {sorted(unknown)}")
    raw_default = data.get("default_profile", "")
    if not isinstance(raw_default, str):
        raise ValueError("default_profile must be a string")
    return profiles, raw_default.strip(), path


def resolve_profile(
    profile_name: str,
    logger: ILogger,
) -> tuple[str, VisionProfile, Path]:
    profiles, default_profile, path = load_profiles()
    if not profiles:
        raise ValueError(f"No vision profiles found in {path}")
    key = profile_name or default_profile or next(iter(profiles))
    profile = profiles.get(key)
    if profile is None:
        available = ", ".join(sorted(profiles.keys()))
        logger.error(f"Vision profile '{key}' not found. Available: {available}")
        raise ValueError(f"Vision profile '{key}' not found")
    if not isinstance(profile, dict):  # pragma: no cover - resolver guarantees it
        raise ValueError(f"Vision profile '{key}' must be a JSON object")
    logger.info(f"Using vision profile '{key}' from {path}")
    return key, profile, path


def get_devices(profile: Mapping[str, object]) -> list[dict]:
    if not isinstance(profile, Mapping):
        raise ValueError("Vision profile must be a mapping")
    devices = profile.get("devices", [])
    if not isinstance(devices, list):
        raise ValueError("Vision profile devices must be a list")
    if not all(isinstance(device, dict) for device in devices):
        raise ValueError("Vision profile devices entries must be JSON objects")
    return devices


__all__ = ["get_devices", "load_profiles", "resolve_profile"]
