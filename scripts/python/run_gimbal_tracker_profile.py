"""Canonical SIYI vision-profile loading and explicit CLI overrides."""

from __future__ import annotations

import argparse
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Protocol

from navpy.modules.navigation.gimbal_navigation_state import GimbalTrackingSetup
from navpy.modules.vision.peripheral.camera_intrinsics import CameraIntrinsics
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from navpy.modules.vision.target_zoom_tracker import TargetZoomTrackerConfig
from navpy.modules.vision.vision_profiles import (
    build_camera_model,
    build_gimbal_data,
    build_tracking_config,
    build_zoom_calibration,
    build_zoom_config,
    get_detector_settings,
    get_devices,
    resolve_profile,
)
from navpy.modules.vision.zoom_calibration import ZoomCalibrationTable

from scripts.python.run_gimbal_tracker_args import SIYI_PROFILE_NAME


class RunnerLog(Protocol):
    def info(self, message: str, *args: object) -> None: ...

    def warning(self, message: str, *args: object) -> None: ...

    def error(self, message: str, *args: object) -> None: ...


@dataclass(frozen=True)
class SiyiRunnerProfile:
    device: dict[str, object]
    detector: dict[str, object]
    tracking_setup: GimbalTrackingSetup | None
    zoom_config: TargetZoomTrackerConfig | None
    camera: CameraIntrinsics
    gimbal_data: GimbalData
    zoom_calibration: ZoomCalibrationTable | None


def load_siyi_profile(logger: RunnerLog) -> SiyiRunnerProfile:
    _, profile, _ = resolve_profile(SIYI_PROFILE_NAME, logger)
    devices = get_devices(profile)
    if len(devices) != 1:
        raise ValueError("SIYI profile must contain exactly one device")
    device = devices[0]
    gimbal = _mapping(device.get("gimbal", {}), "device.gimbal")
    if gimbal.get("type") != "siyi":
        raise ValueError("SIYI profile has a non-SIYI gimbal")
    detector = _validated_detector_settings(profile)
    camera = build_camera_model(device, logger, log=True)
    calibration = build_zoom_calibration(device, logger)
    gimbal_data = build_gimbal_data(device, 0, detector)
    if gimbal_data is None:
        raise ValueError("SIYI profile does not define gimbal data")
    return SiyiRunnerProfile(
        device=device,
        detector=detector,
        tracking_setup=build_tracking_config(device, sim=False),
        zoom_config=build_zoom_config(device, profile),
        camera=camera,
        gimbal_data=gimbal_data,
        zoom_calibration=calibration,
    )


def apply_detector_overrides(
    args: argparse.Namespace,
    detector_settings: Mapping[str, object],
) -> dict[str, object]:
    settings = dict(detector_settings)
    tracker = _optional_group(settings, "tracker")
    deep_search = _optional_group(settings, "deep_search")
    _validate_group_bools("tracker", tracker, ("half", "with_reid"))
    _validate_group_bools("deep_search", deep_search, ("enabled",))
    if args.detect_hz is not None:
        settings["detect_hz"] = float(args.detect_hz)
        tracker["frame_rate"] = max(1, round(float(args.detect_hz)))
    if args.track_hz is not None:
        settings["track_hz"] = float(args.track_hz)
    if args.tracker_backend is not None:
        tracker["backend"] = args.tracker_backend
    if args.deep_search is not None:
        deep_search["enabled"] = args.deep_search
    for attribute, key in (
        ("deep_search_hz", "hz"),
        ("deep_search_imgsz", "imgsz"),
        ("deep_search_conf", "conf"),
    ):
        value = getattr(args, attribute)
        if value is not None:
            deep_search[key] = value
    if tracker:
        settings["tracker"] = tracker
    if deep_search:
        settings["deep_search"] = deep_search
    return settings


def pick_setting(
    cli_value: object | None,
    config: Mapping[str, object],
    key: str,
    default: object,
) -> object:
    return cli_value if cli_value is not None else config.get(key, default)


def build_deep_search_config(
    args: argparse.Namespace,
    detector_settings: Mapping[str, object],
    model_path: str,
    classes: list[int] | None,
) -> dict[str, object] | None:
    settings = _optional_group(detector_settings, "deep_search")
    _validate_group_bools("deep_search", settings, ("enabled",))
    if args.deep_search is not None:
        settings["enabled"] = args.deep_search
    if not settings.get("enabled", False):
        return None
    for attribute, key in (
        ("deep_search_hz", "hz"),
        ("deep_search_imgsz", "imgsz"),
        ("deep_search_conf", "conf"),
    ):
        value = getattr(args, attribute)
        if value is not None:
            settings[key] = value
    settings.setdefault("model_path", model_path)
    settings.setdefault("classes", classes)
    return settings


def build_tracker_config(
    args: argparse.Namespace,
    detector_settings: Mapping[str, object],
) -> dict[str, object] | None:
    settings = _optional_group(detector_settings, "tracker")
    _validate_group_bools("tracker", settings, ("half", "with_reid"))
    if args.tracker_backend is not None:
        settings["backend"] = args.tracker_backend
    return settings or None


def _validated_detector_settings(
    profile: Mapping[str, object],
) -> dict[str, object]:
    settings = dict(get_detector_settings(profile))
    for name, bool_keys in (
        ("tracker", ("half", "with_reid")),
        ("appearance", ("enabled", "half")),
        ("deep_search", ("enabled",)),
    ):
        group = _optional_group(settings, name)
        _validate_group_bools(name, group, bool_keys)
        if name in settings:
            settings[name] = group
    return settings


def _optional_group(
    parent: Mapping[str, object],
    name: str,
) -> dict[str, object]:
    value = parent.get(name)
    if value is None:
        return {}
    return dict(_mapping(value, f"detector.{name}"))


def _mapping(value: object, name: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be a mapping")
    return value


def _validate_group_bools(
    group_name: str,
    group: Mapping[str, object],
    keys: tuple[str, ...],
) -> None:
    for key in keys:
        if key in group and not isinstance(group[key], bool):
            raise ValueError(f"detector.{group_name}.{key} must be boolean")


__all__ = [
    "RunnerLog",
    "SiyiRunnerProfile",
    "apply_detector_overrides",
    "build_deep_search_config",
    "build_tracker_config",
    "load_siyi_profile",
    "pick_setting",
]
