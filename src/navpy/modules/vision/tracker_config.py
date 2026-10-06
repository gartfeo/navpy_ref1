"""Focused tracker policies composed from the existing profile schema."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

from navpy.modules.vision.device import DeviceT


@dataclass(frozen=True)
class TrackerRuntimeConfig:
    backend: str = "custom"
    frame_rate: int = 30


@dataclass(frozen=True)
class CustomTrackerConfig:
    max_age: int = 60
    min_hits: int = 3
    max_center_dist_px: float = 500.0
    revive_seconds: float = 2.5


@dataclass(frozen=True)
class TrackerReidConfig:
    device: DeviceT = "auto"
    weights: Optional[str] = None
    half: bool = True
    cmc_method: Optional[str] = "sof"
    enabled: bool = True


@dataclass(frozen=True)
class TrackerAssociationConfig:
    """Detector confidence and library association thresholds."""

    detector_conf: float = 0.25
    track_high_thresh: Optional[float] = None
    track_low_thresh: Optional[float] = None
    new_track_thresh: Optional[float] = None
    match_thresh: float = 0.8
    proximity_thresh: float = 0.5
    appearance_thresh: float = 0.25
    fuse_first_associate: bool = False

    def resolved_botsort_thresholds(self) -> dict[str, float]:
        conf = float(self.detector_conf)
        high = (
            conf
            if self.track_high_thresh is None
            else float(self.track_high_thresh)
        )
        new = (
            conf
            if self.new_track_thresh is None
            else float(self.new_track_thresh)
        )
        low = (
            min(0.1, conf * 0.5)
            if self.track_low_thresh is None
            else float(self.track_low_thresh)
        )
        return {
            "track_high_thresh": high,
            "track_low_thresh": min(low, high),
            "new_track_thresh": new,
            "match_thresh": float(self.match_thresh),
            "proximity_thresh": float(self.proximity_thresh),
            "appearance_thresh": float(self.appearance_thresh),
        }


@dataclass(frozen=True)
class TrackerBackendConfig:
    runtime: TrackerRuntimeConfig = field(default_factory=TrackerRuntimeConfig)
    custom: CustomTrackerConfig = field(default_factory=CustomTrackerConfig)
    reid: TrackerReidConfig = field(default_factory=TrackerReidConfig)
    association: TrackerAssociationConfig = field(
        default_factory=TrackerAssociationConfig
    )


def tracker_config_from_settings(
    settings: Optional[dict],
    *,
    detector_conf: float = 0.25,
) -> TrackerBackendConfig:
    settings = settings or {}

    def optional_float(key: str) -> Optional[float]:
        value = settings.get(key)
        return None if value is None else float(value)

    return TrackerBackendConfig(
        runtime=TrackerRuntimeConfig(
            backend=str(settings.get("backend", "custom")).lower().strip(),
            frame_rate=int(settings.get("frame_rate", settings.get("fps", 30))),
        ),
        custom=CustomTrackerConfig(
            max_age=int(settings.get("max_age", 60)),
            min_hits=int(settings.get("min_hits", 3)),
            max_center_dist_px=float(
                settings.get("max_center_dist_px", 500.0)
            ),
            revive_seconds=float(settings.get("revive_seconds", 2.5)),
        ),
        reid=TrackerReidConfig(
            device=settings.get("reid_device", settings.get("device", "auto")),
            weights=settings.get("reid_weights"),
            half=bool(settings.get("half", True)),
            cmc_method=settings.get("cmc_method", "sof"),
            enabled=bool(settings.get("with_reid", True)),
        ),
        association=TrackerAssociationConfig(
            detector_conf=float(settings.get("conf", detector_conf)),
            track_high_thresh=optional_float("track_high_thresh"),
            track_low_thresh=optional_float("track_low_thresh"),
            new_track_thresh=optional_float("new_track_thresh"),
            match_thresh=float(settings.get("match_thresh", 0.8)),
            proximity_thresh=float(settings.get("proximity_thresh", 0.5)),
            appearance_thresh=float(settings.get("appearance_thresh", 0.25)),
            fuse_first_associate=bool(
                settings.get("fuse_first_associate", False)
            ),
        ),
    )


__all__ = [
    "CustomTrackerConfig",
    "TrackerAssociationConfig",
    "TrackerBackendConfig",
    "TrackerReidConfig",
    "TrackerRuntimeConfig",
    "tracker_config_from_settings",
]
