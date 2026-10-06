"""Pure copy/format helpers for final-approach command diagnostics."""

from __future__ import annotations

import math

import numpy as np

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.nav.vision_nav.command_transaction import (
    FinalApproachLawEvidence,
)
from navpy.modules.navigation.nav.vision_nav.frame import FinalApproachVisionFrame
from navpy.modules.vision.models.detect_data import DetectedObject


def body_bearing_deg(frame: FinalApproachVisionFrame) -> float:
    return math.degrees(math.atan2(frame.body_y, frame.body_x))


def law_evidence_payload(
    evidence: FinalApproachLawEvidence | None,
) -> dict[str, object]:
    names = (
        "control_bearing_deg", "lateral_rate_deg_s",
        "aircraft_turn_rate_deg_s", "raw_inertial_los_rate_deg_s",
        "inertial_los_rate_deg_s",
        "aircraft_roll_deg", "aircraft_pitch_deg", "air_speed_mps",
        "control_elevation_deg",
        "vertical_rate_deg_s",
        "pitch_time_constant_s",
        "raw_roll_deg", "raw_pitch_deg",
    )
    # Flattened into the row: the CSV stays one flat namespace, while the
    # in-memory record keeps its value objects.
    nested = {
        "roll_limit_deg": ("configured_limits", "roll_limit_deg"),
        "pitch_min_deg": ("configured_limits", "pitch_min_deg"),
        "pitch_max_deg": ("configured_limits", "pitch_max_deg"),
        "effective_roll_limit_deg": ("effective_limits", "roll_limit_deg"),
        "effective_pitch_min_deg": ("effective_limits", "pitch_min_deg"),
        "effective_pitch_max_deg": ("effective_limits", "pitch_max_deg"),
        "plan_reason": ("origin", "reason"),
        "anchor_cmd_roll_deg": ("origin", "anchor_cmd_roll_deg"),
        "anchor_cmd_pitch_deg": ("origin", "anchor_cmd_pitch_deg"),
        "dt_s": ("origin", "dt_s"),
        "lateral_nav_constant": ("origin", "lateral_nav_constant"),
        "vertical_nav_constant": ("origin", "vertical_nav_constant"),
    }
    payload: dict[str, object] = {
        name: "" if evidence is None else getattr(evidence, name)
        for name in names
    }
    for column, (group, field) in nested.items():
        payload[column] = (
            "" if evidence is None
            else getattr(getattr(evidence, group), field)
        )
    return payload


def debug_poi(poi: DetectedObject | None) -> Location | None:
    if poi is None or not poi.geo.is_simulation:
        return None
    return poi.geo.truth_poi_location


def debug_camera(poi: DetectedObject | None) -> Location | None:
    if poi is None or not poi.geo.is_simulation:
        return None
    return poi.geo.camera_location


def copy_location(location: Location | None) -> Location | None:
    if location is None:
        return None
    return Location(
        location.lat,
        location.lng,
        location.alt,
        heading=location.heading,
        is_absolute=location.is_absolute,
    )


def copy_attitude(attitude: Attitude | None) -> Attitude | None:
    if attitude is None:
        return None
    return Attitude(attitude.pitch, attitude.yaw, attitude.roll)


def optional_float(value: object) -> float | None:
    return None if value is None else float(value)


def copy_camera_matrix(
    poi: DetectedObject | None,
) -> np.ndarray | None:
    if poi is None:
        return None
    matrix = poi.optics.camera_matrix()
    if matrix is None:
        return None
    copied = np.array(matrix, copy=True)
    copied.setflags(write=False)
    return copied


__all__ = [
    "body_bearing_deg",
    "copy_attitude",
    "copy_camera_matrix",
    "copy_location",
    "debug_camera",
    "debug_poi",
    "law_evidence_payload",
    "optional_float",
]
