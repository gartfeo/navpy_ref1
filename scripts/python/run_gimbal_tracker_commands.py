"""Anchor geometry and stdin commands for the interactive SIYI runner."""

from __future__ import annotations

import queue
import threading
from collections.abc import Callable
from dataclasses import dataclass

import cv2
import numpy as np

from scripts.python.run_gimbal_tracker_args import ANCHOR_PADDING_PX


@dataclass
class RunnerControlState:
    anchor: int
    last_locked_id: int | None = None
    click_bbox: tuple[float, float, float, float] | None = None


@dataclass(frozen=True)
class TrackSnapshot:
    track_id: int
    cx: float
    cy: float
    width: float
    height: float


def anchor_pixel(
    mode: int,
    img_w: int,
    img_h: int,
    bbox_half_w: float = 0.0,
    bbox_half_h: float = 0.0,
) -> tuple[float, float]:
    inset_x = bbox_half_w + ANCHOR_PADDING_PX
    inset_y = bbox_half_h + ANCHOR_PADDING_PX
    middle_x, middle_y = img_w / 2.0, img_h / 2.0
    return {
        1: (inset_x, img_h - inset_y),
        2: (middle_x, img_h - inset_y),
        3: (img_w - inset_x, img_h - inset_y),
        4: (inset_x, middle_y),
        5: (middle_x, middle_y),
        6: (img_w - inset_x, middle_y),
        7: (inset_x, inset_y),
        8: (middle_x, inset_y),
        9: (img_w - inset_x, inset_y),
    }[mode]


def build_anchor_k(
    k: np.ndarray,
    dist_coeffs: np.ndarray,
    anchor_mode: int,
    img_w: int,
    img_h: int,
    bbox_half_w: float = 0.0,
    bbox_half_h: float = 0.0,
) -> np.ndarray:
    anchor = anchor_pixel(
        anchor_mode,
        img_w,
        img_h,
        bbox_half_w,
        bbox_half_h,
    )
    points = np.asarray([[anchor]], dtype=np.float64)
    undistorted = cv2.undistortPoints(points, k, dist_coeffs, P=k)
    anchored = k.copy()
    anchored[0, 2] = float(undistorted[0, 0, 0])
    anchored[1, 2] = float(undistorted[0, 0, 1])
    return anchored


def parse_command(line: str) -> tuple[str | None, str | None]:
    parts = line.strip().split() if line else []
    if not parts:
        return None, None
    name = parts[0].lower()
    if name in ("n", "f"):
        return (name, None) if len(parts) == 1 else (None, None)
    if name in ("z", "p") and len(parts) == 2:
        return name, parts[1]
    return None, None


def pick_next_id(
    track_ids: list[int],
    current_id: int | None,
) -> int | None:
    if not track_ids:
        return None
    if current_id in track_ids:
        index = track_ids.index(current_id)
        return track_ids[(index + 1) % len(track_ids)]
    return track_ids[0]


def start_command_reader(
    commands: queue.Queue[str],
    warning: Callable[[str], object],
) -> None:
    def read() -> None:
        while True:
            try:
                line = input()
            except (EOFError, OSError, KeyboardInterrupt):
                warning("stdin command reader stopped")
                return
            if line.strip():
                commands.put(line)

    threading.Thread(target=read, daemon=True).start()


def drain_commands(
    commands: queue.Queue[str],
    state: RunnerControlState,
    tracks: list[TrackSnapshot],
    autofocus: Callable[[], object],
    set_zoom: Callable[[str], object],
    warning: Callable[[str], object],
) -> None:
    while True:
        try:
            line = commands.get_nowait()
        except queue.Empty:
            return
        name, argument = parse_command(line)
        if name == "n":
            next_id = pick_next_id(
                [track.track_id for track in tracks],
                state.last_locked_id,
            )
            match = next(
                (track for track in tracks if track.track_id == next_id),
                None,
            )
            if match is not None:
                state.click_bbox = (
                    match.cx,
                    match.cy,
                    match.width,
                    match.height,
                )
        elif name == "f":
            autofocus()
        elif name == "z" and argument is not None:
            set_zoom(argument)
        elif name == "p":
            try:
                anchor = int(argument)
            except (TypeError, ValueError):
                anchor = 0
            if 1 <= anchor <= 9:
                state.anchor = anchor
        else:
            warning(f"unknown command: {line!r}")


__all__ = [
    "RunnerControlState",
    "TrackSnapshot",
    "anchor_pixel",
    "build_anchor_k",
    "drain_commands",
    "parse_command",
    "pick_next_id",
    "start_command_reader",
]
