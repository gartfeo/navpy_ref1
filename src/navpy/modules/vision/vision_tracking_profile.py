"""Vision-owned tracking profile parsing without Navigation dependencies."""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass

from navpy.modules.vision.gimbal_rate_tracker import GimbalRateTrackerConfig
from navpy.modules.vision.target_angle_estimator import TargetAngleEstimatorConfig


@dataclass(frozen=True)
class VisionTrackingProfile:
    """Parsed visual-rate configuration plus optional loss-policy overrides."""

    rate: GimbalRateTrackerConfig
    loss_hold_sec: float | None = None
    loss_repoint_sec: float | None = None
    preserve_zoom_during_loss: bool | None = None


def _mapping(value: object, name: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be a mapping")
    return value


def _finite_float(value: object, name: str, *, minimum: float = 0.0) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a finite number >= {minimum}")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a finite number >= {minimum}") from exc
    if not math.isfinite(number) or number < minimum:
        raise ValueError(f"{name} must be a finite number >= {minimum}")
    return number


def _boolean(value: object, name: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{name} must be boolean")
    return value


def _estimator_config(raw: object) -> TargetAngleEstimatorConfig | None:
    if raw is None:
        return None
    estimator = _mapping(raw, "gimbal.tracking.estimator")
    if not estimator:
        return None
    overrides: dict[str, float | None] = {}
    positive_keys = {
        "measurement_sigma",
        "accel_sigma",
        "initial_angle_sigma",
        "initial_rate_sigma",
        "max_step_dt",
    }
    nonnegative_keys = {"robust_nis_knee", "max_abs_rate"}
    unknown = set(estimator) - positive_keys - nonnegative_keys
    if unknown:
        raise ValueError(
            "Unknown gimbal.tracking.estimator key(s): "
            f"{sorted(unknown)}"
        )
    for key in positive_keys:
        if key in estimator:
            value = _finite_float(estimator[key], f"estimator.{key}")
            if value <= 0.0:
                raise ValueError(f"estimator.{key} must be > 0")
            overrides[key] = value
    for key in nonnegative_keys:
        if key not in estimator:
            continue
        if key == "max_abs_rate" and estimator[key] is None:
            overrides[key] = None
        else:
            overrides[key] = _finite_float(estimator[key], f"estimator.{key}")
    return TargetAngleEstimatorConfig(**overrides)


def build_tracking_profile(
    device: Mapping[str, object],
    *,
    sim: bool,
) -> VisionTrackingProfile | None:
    """Parse visual tracker inputs without importing Navigation policy types."""
    if not isinstance(device, Mapping):
        raise ValueError("Vision profile device must be a mapping")
    raw_gimbal = device.get("gimbal", {})
    if not isinstance(raw_gimbal, Mapping):
        raise ValueError("Vision profile device gimbal must be a mapping")
    raw_tracking = raw_gimbal.get("tracking")
    if raw_tracking is None:
        return None
    tracking = _mapping(raw_tracking, "gimbal.tracking")
    rate_keys = {
        "correction_bw",
        "max_rate",
        "max_slew_dps",
        "settle_tolerance_deg",
    }
    allowed_keys = rate_keys | {
        "command_lead_time",
        "enabled",
        "estimator",
        "loss_hold_sec",
        "loss_repoint_sec",
        "preserve_zoom_during_loss",
    }
    unknown = set(tracking) - allowed_keys
    if unknown:
        raise ValueError(
            f"Unknown gimbal.tracking key(s): {sorted(unknown)}"
        )
    if not _boolean(tracking.get("enabled", False), "gimbal.tracking.enabled"):
        return None

    rate_kwargs = {
        key: _finite_float(tracking[key], f"gimbal.tracking.{key}")
        for key in rate_keys
        if key in tracking
    }
    if sim:
        rate_kwargs["command_lead_time"] = 0.0
    elif "command_lead_time" in tracking:
        rate_kwargs["command_lead_time"] = _finite_float(
            tracking["command_lead_time"],
            "gimbal.tracking.command_lead_time",
        )
    estimator = _estimator_config(tracking.get("estimator"))
    if estimator is not None:
        rate_kwargs["estimator"] = estimator

    hold = (
        None
        if "loss_hold_sec" not in tracking
        else _finite_float(tracking["loss_hold_sec"], "loss_hold_sec")
    )
    repoint = (
        None
        if "loss_repoint_sec" not in tracking
        else _finite_float(tracking["loss_repoint_sec"], "loss_repoint_sec")
    )
    preserve = (
        None
        if "preserve_zoom_during_loss" not in tracking
        else _boolean(
            tracking["preserve_zoom_during_loss"],
            "gimbal.tracking.preserve_zoom_during_loss",
        )
    )
    return VisionTrackingProfile(
        rate=GimbalRateTrackerConfig(**rate_kwargs),
        loss_hold_sec=hold,
        loss_repoint_sec=repoint,
        preserve_zoom_during_loss=preserve,
    )


__all__ = ["VisionTrackingProfile", "build_tracking_profile"]
