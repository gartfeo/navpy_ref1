"""Formatting for structured navigation event rows."""

from __future__ import annotations

import math
from typing import Any, Mapping, Optional

from navpy.logger.navigation_snap_types import ClosestSnap
from navpy.logger.log_events import (
    LogEvent,
    format_snap_event_row,
    format_standard_event_row,
)


def build_event_line(
    timestamp: str,
    event: LogEvent,
    payload: Mapping[str, Any],
) -> str:
    if event == LogEvent.SNAP:
        return format_snap_event_row(
            timestamp,
            algorithm=payload.get("algorithm"),
            snap_str=str(payload.get("snap", "")),
            kp_str=str(payload.get("kp_str", "")),
        )
    pairs = [(str(key), str(value)) for key, value in payload.items()]
    return format_standard_event_row(timestamp, event, pairs)


def format_component_value(value: float) -> str:
    """Format certification components at sub-millimetre resolution."""
    if not math.isfinite(value):
        return "NA"
    return f"{value:.6f}"


def snap_components_payload(
    snap: ClosestSnap,
    *,
    algorithm: Optional[str],
    kp_str: str,
) -> Mapping[str, str]:
    payload = {
        "algorithm": algorithm.upper() if algorithm else "",
        "dist_3d_m": format_component_value(snap.dist),
        "h_m": format_component_value(snap.h_dist),
        "v_m": format_component_value(snap.v_dist),
        "lateral_m": format_component_value(snap.component_lateral),
        "longitudinal_m": format_component_value(snap.component_longitudinal),
        "vertical_m": format_component_value(snap.component_vertical),
        "signed_lateral_m": format_component_value(
            snap.component_lateral_signed
        ),
        "signed_longitudinal_m": format_component_value(
            snap.component_longitudinal_signed
        ),
        "signed_vertical_m": format_component_value(
            snap.component_vertical_signed
        ),
    }
    if kp_str:
        payload["kp"] = kp_str.removeprefix("kp=")
    return payload
