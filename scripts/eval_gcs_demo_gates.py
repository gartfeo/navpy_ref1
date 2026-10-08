"""Sensor, operator-review, and mission workflow certification gates."""

from __future__ import annotations

import json
import math
import re
from functools import lru_cache
from pathlib import Path

from scripts.eval_gcs_demo_constants import (
    FORBIDDEN_VISION_PROFILE,
    SIM_SPEEDUP,
    VISION_DETECT_HZ,
    VISION_MOUNT_PITCH_DEG,
    VISION_PROFILE,
)
from scripts.eval_gcs_demo_evidence import navigation_speedup_errors
from scripts.eval_gcs_demo_models import EpisodeMetrics, RegressionError

from navpy.modules.vision.vision_profiles import (
    get_min_pixels_for_class,
    load_profiles,
)
from navpy.modules.vision.vision_profile_types import VisionProfile


_VISION_PROFILE_RE = re.compile(r"Using vision profile '(?P<profile>[^']+)'")
_ARGS_VISION_PROFILE_RE = re.compile(r"vision_profile='(?P<profile>[^']+)'")
_MOUNT_RE = re.compile(
    r"Mount '(?P<name>[^']+)':\s*\d+x\d+,\s*"
    r"pitch=(?P<pitch>-?[0-9]+(?:\.[0-9]+)?),\s*"
    r"fixed=(?P<fixed>True|False)"
)
_IDEAL_360_ACTIVE_RE = re.compile(r"static ideal 360 enabled", re.IGNORECASE)
_ASSIGNED_RE = re.compile(r"\bTask \d+ assigned by owner \d+\b")


@lru_cache(maxsize=1)
def active_vision_profile() -> VisionProfile:
    profiles, _, _ = load_profiles()
    profile = profiles.get(VISION_PROFILE)
    if not isinstance(profile, dict):
        raise RegressionError(f"vision profile {VISION_PROFILE!r} is unavailable")
    return profile


def confirmation_image_errors(
    log_dir: Path,
    sys_id: int,
    navigation_text: str,
    metrics: EpisodeMetrics,
) -> list[str]:
    paths = sorted(log_dir.glob(f"uav_{sys_id}_confirmation_t*_meta.json"))
    if len(paths) != 1:
        return [f"expected exactly one confirmation metadata artifact, found {len(paths)}"]
    try:
        metadata = json.loads(paths[0].read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        return [f"invalid confirmation metadata {paths[0].name}: {error}"]
    if not isinstance(metadata, dict):
        return [f"invalid confirmation metadata {paths[0].name}: not an object"]
    bbox = metadata.get("bbox_cxcywh")
    class_id = metadata.get("class_id")
    if (
        not isinstance(bbox, list)
        or len(bbox) != 4
        or type(class_id) is not int
        or class_id < 0
    ):
        return [f"invalid confirmation metadata {paths[0].name}: schema"]
    width, height = bbox[2], bbox[3]
    if any(
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
        or float(value) <= 0.0
        for value in (width, height)
    ):
        return [f"invalid confirmation bbox size {width}x{height}"]
    source_size = math.hypot(float(width), float(height))
    required_size = get_min_pixels_for_class(active_vision_profile(), class_id)
    metrics.confirmation_source_size_px = source_size
    metrics.confirmation_required_size_px = required_size
    best_available = "CONFIRM gate best available at max zoom" in navigation_text
    if source_size < required_size and not best_available:
        return [
            f"confirmation source {source_size:.1f}px < required "
            f"{required_size:.1f}px without max-zoom best-available proof"
        ]
    return []


def sensor_config_errors(navigation_text: str) -> list[str]:
    errors: list[str] = []
    detector = active_vision_profile().get("detector")
    detect_hz = detector.get("detect_hz") if isinstance(detector, dict) else None
    if (
        isinstance(detect_hz, bool)
        or not isinstance(detect_hz, (int, float))
        or not math.isfinite(float(detect_hz))
        or not math.isclose(float(detect_hz), VISION_DETECT_HZ, abs_tol=1e-9)
    ):
        errors.append(
            f"checked profile detector rate {detect_hz!r} != {VISION_DETECT_HZ:.1f}Hz"
        )
    profile_match = _VISION_PROFILE_RE.search(navigation_text)
    if profile_match is None:
        errors.append(f"missing resolved vision profile marker (\"Using vision profile '{VISION_PROFILE}'\")")
    elif profile_match.group("profile") != VISION_PROFILE:
        errors.append(
            f"resolved vision profile {profile_match.group('profile')!r} != {VISION_PROFILE}"
        )
    args_match = _ARGS_VISION_PROFILE_RE.search(navigation_text)
    if args_match is not None and args_match.group("profile") == FORBIDDEN_VISION_PROFILE:
        errors.append(
            f"args vision_profile={FORBIDDEN_VISION_PROFILE!r} is the simulator-only sensor upper bound"
        )
    mount_match = _MOUNT_RE.search(navigation_text)
    if mount_match is None:
        errors.append(f"missing resolved mount marker (\"Mount '{VISION_PROFILE}'\")")
    else:
        pitch = float(mount_match.group("pitch"))
        if abs(pitch - VISION_MOUNT_PITCH_DEG) > 1e-6:
            errors.append(f"resolved mount pitch {pitch} != {VISION_MOUNT_PITCH_DEG}")
        if mount_match.group("fixed") != "False":
            errors.append(f"resolved mount fixed={mount_match.group('fixed')} != False")
    if _IDEAL_360_ACTIVE_RE.search(navigation_text) is not None:
        errors.append(
            "detector reported the simulator-only ideal_360 static sensor active; "
            "the upper bound must never certify a final-approach run"
        )
    return errors


def _ordered_workflow_errors(navigation_text: str) -> list[str]:
    errors: list[str] = []
    required = {
        "manual confirmation request": "Sending confirm request for P",
        "confirmation state": "CONFIRMING: P",
        "ground-station confirmation": "confirmed by ground station.",
        "gimbal detection lock": "start_tracking obj_id=",
        "NAV initialization": "INIT: NAV MODE",
        "coordinate pass reset": "RESET: PASSED POI",
        "vision-nav SNAP": "SNAP(VISION-NAV",
    }
    for label, marker in required.items():
        if marker not in navigation_text:
            errors.append(f"missing {label} marker ({marker!r})")
    positions = {
        "confirming": navigation_text.find("CONFIRMING: P"),
        "request": navigation_text.find("Sending confirm request for P"),
        "confirmed": navigation_text.find("confirmed by ground station."),
        "nav": navigation_text.find("INIT: NAV MODE"),
        "reset": navigation_text.find("RESET: PASSED POI"),
        "snap": navigation_text.find("SNAP(VISION-NAV"),
    }
    if all(value >= 0 for value in positions.values()):
        if max(positions["confirming"], positions["request"]) > positions["confirmed"]:
            errors.append("ground-station confirmation preceded the companion request state")
        if not positions["confirmed"] < positions["nav"] < positions["reset"] < positions["snap"]:
            errors.append("required order is ground-station-confirmed -> INIT NAV -> coordinate pass reset -> SNAP")
        review = navigation_text[min(positions["confirming"], positions["request"]):positions["confirmed"]]
        if "gimbal recentred" in review or "-> HOLDING" in review:
            errors.append("gimbal tracking was not retained during operator review")
    return errors


def _peer_assignment_errors(navigation_text: str) -> list[str]:
    """A peer flies its task only once its owner applied it (acked logs)."""
    assigned = _ASSIGNED_RE.search(navigation_text)
    if assigned is None:
        return ["peer never logged 'Task T assigned by owner O'"]
    approach = navigation_text.find("GUIDED_LOITER")
    if 0 <= approach < assigned.start():
        return ["peer started GUIDED_LOITER before 'Task T assigned by owner O'"]
    return []


def workflow_errors(
    navigation_text: str,
    *,
    role: str,
    approval_count: int,
    navigation_speedup: float,
    launch_speedup: float = SIM_SPEEDUP,
    assignment_acked: bool = False,
) -> list[str]:
    if role not in {"owner", "peer"}:
        raise ValueError(f"invalid demo role {role!r}")
    errors = sensor_config_errors(navigation_text) + _ordered_workflow_errors(navigation_text)
    if role == "owner":
        for task_id in (2, 3):
            if f"Rebroadcast task {task_id}" not in navigation_text and re.search(rf"\bTask {task_id} accepted by \d+\b", navigation_text) is None:
                errors.append(f"owner did not dispatch task {task_id} to peers")
        if "Self-detect orbit:" not in navigation_text:
            errors.append("owner did not keep a POI-centred self-detect orbit")
    else:
        if "No POI is set" not in navigation_text:
            errors.append("peer did not start with targ_wps=0")
        if "GUIDED_LOITER" not in navigation_text:
            errors.append("peer did not execute GUIDED_LOITER assignment approach")
        if assignment_acked:
            errors.extend(_peer_assignment_errors(navigation_text))
    if approval_count != 1:
        errors.append(f"expected exactly one audited approval, found {approval_count}")
    lowered = navigation_text.lower()
    forbidden = {
        "auto confirmation": "auto-confirmed",
        "local vision auto confirmation": "vision nav local",
        "current-coordinate confirmation hold": "confirm hold:",
        "post-confirmation POI re-approach": "confirm release:",
    }
    for label, marker in forbidden.items():
        if marker in lowered:
            errors.append(f"forbidden {label} marker ({marker!r})")
    errors.extend(
        navigation_speedup_errors(
            navigation_text,
            requested=navigation_speedup,
            launch_speed=launch_speedup,
        )
    )
    return errors


__all__ = [
    "active_vision_profile",
    "confirmation_image_errors",
    "sensor_config_errors",
    "workflow_errors",
]
