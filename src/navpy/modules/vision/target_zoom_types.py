"""Immutable values shared by the target-zoom components."""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass, field
from types import MappingProxyType

from navpy.modules.vision.vision_class_profile import MIN_CONFIRM_PIXELS
from navpy.modules.vision.zoom_tracking_types import ZoomTrackingState


def _freeze_thresholds(values: Mapping[object, object]) -> Mapping[str, float]:
    if not isinstance(values, Mapping) or not values:
        raise ValueError("target_pixels must be a non-empty mapping")
    frozen: dict[str, float] = {}
    for raw_key, raw_value in values.items():
        if isinstance(raw_value, bool):
            raise ValueError("target_pixels values must be positive finite numbers")
        try:
            value = float(raw_value)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                "target_pixels values must be positive finite numbers"
            ) from exc
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError("target_pixels values must be positive finite numbers")
        frozen[str(raw_key)] = value
    if "default" not in frozen:
        raise ValueError("target_pixels must define a default threshold")
    return MappingProxyType(frozen)


@dataclass(frozen=True)
class TargetZoomTrackerConfig:
    """Detector-derived recognition thresholds; no hardware tuning knobs."""

    target_pixels: Mapping[str, float] = field(
        default_factory=lambda: {"default": float(MIN_CONFIRM_PIXELS)}
    )

    def __post_init__(self) -> None:
        object.__setattr__(self, "target_pixels", _freeze_thresholds(self.target_pixels))


@dataclass(frozen=True)
class ZoomTrackResult:
    state: ZoomTrackingState
    has_target: bool
    size_px: float | None = None
    target_pixels: float | None = None
    reason: str = ""
    current_zoom: float | None = None
    desired_zoom: float | None = None
    command_zoom: float | None = None
    at_max_zoom: bool = False
    transition_pending: bool = False

    @property
    def is_stable(self) -> bool:
        if self.state is ZoomTrackingState.UNSUPPORTED:
            return True
        if self.transition_pending or self.reason == "confirming":
            return False
        if self.state is not ZoomTrackingState.HOLDING or not self.has_target:
            return False
        if self.at_max_zoom:
            return True
        return (
            self.size_px is not None
            and self.target_pixels is not None
            and self.size_px >= self.target_pixels
        )


@dataclass(frozen=True)
class ZoomCapabilities:
    continuous: bool
    absolute: bool
    levels: tuple[str, ...]
    minimum: float
    maximum: float
    command_step: float
    available: bool = True

    @property
    def supported(self) -> bool:
        return self.available and (
            self.continuous or self.absolute or len(self.levels) >= 2
        )


@dataclass(frozen=True)
class FiniteReading:
    value: float | None
    invalid: bool = False


@dataclass(frozen=True)
class SampleIdReading:
    value: str | None
    invalid: bool = False


@dataclass(frozen=True)
class ZoomReadback:
    current: FiniteReading
    fresh: FiniteReading
    level: FiniteReading
    sample_id: str | None
    sample_advanced: bool
    sample_invalid: bool = False

    @property
    def invalid(self) -> bool:
        return (
            self.current.invalid
            or self.fresh.invalid
            or self.level.invalid
            or self.sample_invalid
        )


@dataclass(frozen=True)
class ZoomGeometry:
    width: float
    height: float
    fy: float | None


@dataclass(frozen=True)
class ZoomGeometryReading:
    value: ZoomGeometry | None
    invalid: bool = False


@dataclass(frozen=True)
class ZoomStopPlan:
    was_active: bool
    had_continuous: bool
    sample_id: str | None = None


@dataclass(frozen=True)
class ZoomObservation:
    bbox: tuple[float, float, float, float]
    size_px: float
    target_pixels: float
    containment_scale: float | None
    reserve_scale: float | None
    centered: bool
    optical_fresh: bool
    hold_on_decenter: bool


@dataclass(frozen=True)
class ContinuousZoomDecision:
    state: ZoomTrackingState
    reason: str
    at_max_zoom: bool = False


@dataclass(frozen=True)
class ZoomCommandPlan:
    direction: ZoomTrackingState
    reason: str
    current_zoom: float | None = None
    desired_zoom: float | None = None
    command_zoom: float | None = None
    command_key: str | None = None
    at_max_zoom: bool = False


@dataclass(frozen=True)
class ZoomDriveOutcome:
    result: ZoomTrackResult
    committed: bool


__all__ = [
    "ContinuousZoomDecision",
    "FiniteReading",
    "SampleIdReading",
    "TargetZoomTrackerConfig",
    "ZoomCapabilities",
    "ZoomCommandPlan",
    "ZoomGeometry",
    "ZoomGeometryReading",
    "ZoomDriveOutcome",
    "ZoomObservation",
    "ZoomReadback",
    "ZoomStopPlan",
    "ZoomTrackResult",
]
