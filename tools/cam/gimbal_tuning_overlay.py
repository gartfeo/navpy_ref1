"""Display overlay and pure hotkey decoding for the gimbal tuning tool."""

from __future__ import annotations

from dataclasses import dataclass
import cv2
import numpy as np

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.vision.gimbal_rate_types import GimbalTrackResult
from .gimbal_tuning_geometry import (
    ANCHOR_NAMES,
    TrackingCommand,
    TrackingIntrinsics,
    anchor_pixel,
    format_zoom,
    normalize_bbox,
    step_zoom,
)


@dataclass(frozen=True)
class HotkeyAction:
    name: str
    value: float | int | None = None


@dataclass(frozen=True)
class OverlayPoi:
    obj_id: int | None
    bbox_cxcywh: tuple[float, float, float, float] | None


def decode_hotkey(
    key: int,
    current_zoom: float,
    zoom_levels: list[float],
) -> HotkeyAction | None:
    if key in (ord("q"), 27):
        return HotkeyAction("quit")
    if key == ord("c"):
        return HotkeyAction("center")
    if key in (ord("+"), ord("=")):
        return HotkeyAction("zoom", step_zoom(current_zoom, zoom_levels, 1))
    if key == ord("-"):
        return HotkeyAction("zoom", step_zoom(current_zoom, zoom_levels, -1))
    if ord("1") <= key <= ord("9"):
        return HotkeyAction("anchor", key - ord("0"))
    return None


def draw_overlay(
    frame: np.ndarray,
    poi: OverlayPoi | None,
    command: TrackingCommand | None,
    result: GimbalTrackResult,
    anchor_mode: int,
    zoom_level: float,
    intrinsics: TrackingIntrinsics,
    attitude: Attitude,
) -> np.ndarray:
    display = frame.copy()
    anchor = anchor_pixel(
        anchor_mode,
        display.shape[1],
        display.shape[0],
        intrinsics.cx,
        intrinsics.cy,
    )
    cv2.drawMarker(
        display,
        (int(anchor[0]), int(anchor[1])),
        (0, 255, 255),
        cv2.MARKER_DIAMOND,
        18,
        2,
    )
    bbox = _bbox_to_xyxy(
        None if poi is None else poi.bbox_cxcywh
    )
    if bbox is not None:
        cv2.rectangle(display, bbox[:2], bbox[2:], (0, 255, 0), 2)
    if command is not None:
        center = tuple(int(round(value)) for value in command.bbox_center)
        cv2.drawMarker(
            display,
            center,
            (0, 0, 255),
            cv2.MARKER_CROSS,
            18,
            2,
        )
        cv2.line(
            display,
            (int(round(anchor[0])), int(round(anchor[1]))),
            center,
            (255, 0, 255),
            2,
        )
    _draw_status(
        display,
        poi,
        command,
        result,
        anchor_mode,
        zoom_level,
        intrinsics,
        attitude,
    )
    return display


def _draw_status(
    display: np.ndarray,
    poi: OverlayPoi | None,
    command: TrackingCommand | None,
    result: GimbalTrackResult,
    anchor_mode: int,
    zoom_level: float,
    intrinsics: TrackingIntrinsics,
    attitude: Attitude,
) -> None:
    source = intrinsics.source
    if source == "estimated":
        source = f"estimated@{format_zoom(intrinsics.reference_zoom)}x"
    status = (
        f"Zoom: {format_zoom(zoom_level)}x  "
        f"Anchor: {ANCHOR_NAMES[anchor_mode]} [{anchor_mode}]  "
        f"Intrinsics: {source}"
    )
    cv2.putText(
        display, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
        0.65, (0, 255, 0), 2,
    )
    if command is not None and result.yaw_rate is not None:
        poi_id = "?" if poi is None else poi.obj_id
        tracking = (
            f"id={poi_id}  delta=({command.delta_yaw_deg:+.2f}, "
            f"{command.delta_pitch_deg:+.2f}) deg  "
            f"cmd=({result.yaw_rate:+.1f}, {result.pitch_rate:+.1f})"
        )
    else:
        tracking = f"state={result.state.value}  waiting for locked POI"
    cv2.putText(
        display, tracking, (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
        0.65, (0, 255, 0), 2,
    )
    cv2.putText(
        display,
        f"gimbal=({attitude.yaw:.1f}, {attitude.pitch:.1f})",
        (10, 90),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 200, 200),
        2,
    )
    cv2.putText(
        display,
        "1-9 anchor  +/- zoom  c center  q quit",
        (10, display.shape[0] - 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (200, 200, 200),
        1,
    )


def _bbox_to_xyxy(
    bbox_cxcywh: tuple[float, float, float, float] | None,
) -> tuple[int, int, int, int] | None:
    bbox = normalize_bbox(bbox_cxcywh)
    if bbox is None:
        return None
    center_x, center_y, width, height = bbox
    return (
        int(round(center_x - width / 2.0)),
        int(round(center_y - height / 2.0)),
        int(round(center_x + width / 2.0)),
        int(round(center_y + height / 2.0)),
    )


__all__ = [
    "HotkeyAction",
    "OverlayPoi",
    "decode_hotkey",
    "draw_overlay",
]
