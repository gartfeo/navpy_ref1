"""Debug overlay rendering for the detector.

Kept out of the detector orchestrator (which should only wire things together).
Pure drawing: takes the frame + tracks + the bits of camera/vehicle state it
needs and returns an annotated copy.
"""
from __future__ import annotations

from typing import List, Optional, Tuple

import cv2
import numpy as np

from navpy.modules.vision.multi_object_tracker import TrackedObject


def draw_corner_box(
        img: np.ndarray,
        x1: int,
        y1: int,
        x2: int,
        y2: int,
        color: Tuple[int, int, int],
        thickness: int,
) -> None:
    h, w = img.shape[:2]
    x1 = max(0, min(w - 1, int(x1)))
    y1 = max(0, min(h - 1, int(y1)))
    x2 = max(0, min(w - 1, int(x2)))
    y2 = max(0, min(h - 1, int(y2)))
    if x2 <= x1 or y2 <= y1:
        return
    corner_len = max(8, int(min(x2 - x1, y2 - y1) * 0.22))

    cv2.line(img, (x1, y1), (x1 + corner_len, y1), color, thickness)
    cv2.line(img, (x1, y1), (x1, y1 + corner_len), color, thickness)
    cv2.line(img, (x2, y1), (x2 - corner_len, y1), color, thickness)
    cv2.line(img, (x2, y1), (x2, y1 + corner_len), color, thickness)
    cv2.line(img, (x1, y2), (x1 + corner_len, y2), color, thickness)
    cv2.line(img, (x1, y2), (x1, y2 - corner_len), color, thickness)
    cv2.line(img, (x2, y2), (x2 - corner_len, y2), color, thickness)
    cv2.line(img, (x2, y2), (x2, y2 - corner_len), color, thickness)


def draw_debug_overlay(
        frame: np.ndarray,
        tracks: List[TrackedObject],
        locked: Optional[TrackedObject],
        *,
        mount,
        vehicle_attitude,
        fps_est: float,
        use_lock: bool,
) -> np.ndarray:
    """Draw detection overlays on a frame copy and return it."""
    img = frame.copy()
    h, w = img.shape[:2]

    k = mount.get_k()
    fx = k[0, 0] if k is not None else 1000.0
    fy = k[1, 1] if k is not None else 1000.0
    dist = mount.get_dist()

    if locked is not None:
        lock_x, lock_y = int(locked.cx), int(locked.cy)
        cross_size = 18
        selected_color = (0, 255, 255)
        cv2.line(img, (lock_x - cross_size, lock_y), (lock_x + cross_size, lock_y), selected_color, 1)
        cv2.line(img, (lock_x, lock_y - cross_size), (lock_x, lock_y + cross_size), selected_color, 1)

    # UAV pitch/roll as a red circle, projected with distortion.
    if vehicle_attitude is not None and k is not None:
        pitch_rad = np.radians(vehicle_attitude.pitch)
        roll_rad = np.radians(vehicle_attitude.roll)
        point_3d = np.array([[[np.tan(roll_rad), -np.tan(pitch_rad), 1.0]]], dtype=np.float32)
        rvec = np.zeros(3, dtype=np.float32)
        tvec = np.zeros(3, dtype=np.float32)
        pixels, _ = cv2.projectPoints(point_3d, rvec, tvec, k, dist)
        uav_x = int(pixels[0, 0, 0])
        uav_y = int(pixels[0, 0, 1])
        if 0 <= uav_x < w and 0 <= uav_y < h:
            cv2.circle(img, (uav_x, uav_y), 12, (0, 0, 255), 2)

    for t in tracks:
        x1 = int(t.cx - t.w * 0.5)
        y1 = int(t.cy - t.h * 0.5)
        x2 = int(t.cx + t.w * 0.5)
        y2 = int(t.cy + t.h * 0.5)

        is_locked = (locked is not None and t.id == locked.id)
        if is_locked:
            color = (0, 255, 255)
            thickness = 2
            label = f"id={t.id} {t.confidence:.2f}"
        elif t.is_confirmed:
            color = (150, 150, 150)
            thickness = 1
            label = f"{t.confidence:.2f}"
        else:
            color = (90, 90, 90)
            thickness = 1
            label = f"{t.confidence:.2f}"

        draw_corner_box(img, x1, y1, x2, y2, color, thickness)

        detected_pt = np.array([[[t.cx, t.cy]]], dtype=np.float32)
        undistorted = cv2.undistortPoints(detected_pt, k, dist, P=k)
        ud_x, ud_y = undistorted[0, 0, 0], undistorted[0, 0, 1]
        px_off_x = ud_x - k[0, 2]
        px_off_y = ud_y - k[1, 2]
        ang_x = np.degrees(np.arctan2(px_off_x, fx))
        ang_y = np.degrees(np.arctan2(-px_off_y, fy))

        cv2.putText(
            img, label, (x1, max(0, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX,
            0.45 if not is_locked else 0.55, color, thickness,
        )
        if is_locked:
            cv2.putText(
                img,
                f"px=({int(px_off_x)},{int(px_off_y)}) a=({ang_x:.1f},{ang_y:.1f})",
                (x1, min(h - 5, y2 + 18)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1,
            )

    txt = f"track_hz~{fps_est:.1f}"
    if use_lock:
        txt += f" | LOCK={locked.id if locked else 'None'}"
    zoom = mount.get_current_zoom()
    if zoom is not None:
        txt += f" | Z{zoom}x"
    cv2.putText(img, txt, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

    return img
