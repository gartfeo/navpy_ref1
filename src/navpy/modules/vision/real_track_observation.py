"""Pixel-space measurements derived from one tracked object."""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from navpy.modules.vision.models.detection_components import BoundingBox
from navpy.modules.vision.multi_object_tracker import TrackedObject


@dataclass(frozen=True)
class MappedTrackObservation:
    """Undistorted target center plus tracking-frame geometry."""

    u_px: float
    v_px: float
    bbox_cxcywh: BoundingBox
    center_score: float


def map_track_observation(
        track: TrackedObject,
        intrinsic: np.ndarray,
        distortion: np.ndarray,
        frame_width: int,
        frame_height: int,
) -> MappedTrackObservation:
    """Map a track center through the captured camera calibration."""
    detected_pt = np.array([[[track.cx, track.cy]]], dtype=np.float32)
    undistorted = cv2.undistortPoints(
        detected_pt,
        intrinsic,
        distortion,
        P=intrinsic,
    )
    bbox = (track.cx, track.cy, track.w, track.h)
    x_pct = track.cx / frame_width
    y_pct = track.cy / frame_height
    return MappedTrackObservation(
        u_px=undistorted[0, 0, 0],
        v_px=undistorted[0, 0, 1],
        bbox_cxcywh=bbox,
        center_score=abs(x_pct - 0.5) + abs(y_pct - 0.5),
    )


__all__ = ["MappedTrackObservation", "map_track_observation"]
