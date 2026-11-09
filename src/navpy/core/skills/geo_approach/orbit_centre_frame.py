"""Which frame may place the HITL orbit's ground-ray centre (HITL step 23b; hitl_dock_detector.md section 4.1).

The pre-series check bounds the ground ray's error at D_max, the farthest distance where the dock's box meets the
acquisition gate's floor (``min_box_px``) at the search step. The acquisition gate enforces that floor only once, on
the acquisition frame; the orbit is placed later, after the fresh lock (step 25), when the aircraft has moved and
the zoom may have changed. So a frame may place the centre only if it would itself pass the floor at the search
step: its box's smaller side in the detector's input pixels, scaled from the frame's own focal length to the search
step's, is at least ``min_box_px``. Its pose must hold what the ground ray reads (legacy_destination_resolver.py):
frame-atomic, with the aircraft's attitude, the gimbal's readback, the camera matrix and the camera's location.

Acquisition geolocation (GPS position and attitude are allowed outside the approach, owner 2026-10-07); box size is
read here, before NAV, never on the NAV command or completion paths (section 4.2).
"""
from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass

from navpy.core.ports.detect_data import DetectedObject


@dataclass(frozen=True)
class OrbitCentreFrame:
    min_box_px: float
    search_fx_px: float
    search_fy_px: float
    # The detector's input pixels per full-frame pixel, across and down, for a frame of this size.
    input_scale: Callable[[int, int], tuple[float, float]]

    def __call__(self, poi: DetectedObject) -> bool:
        try:
            pixel, optics = poi.pixel, poi.optics
            if pixel.pose_is_frame_atomic is not True or poi.pose.aircraft_attitude is None \
                    or poi.pose.gimbal_data is None or poi.geo.camera_location is None \
                    or optics.camera_matrix() is None:
                return False
            _cx, _cy, w, h = poi.tracking.bbox_cxcywh
            scale_x, scale_y = self.input_scale(int(optics.frame_width_px), int(optics.frame_height_px))
            across = float(w) * scale_x * self.search_fx_px / float(pixel.calibration.fx_px)
            down = float(h) * scale_y * self.search_fy_px / float(pixel.calibration.fy_px)
        except (AttributeError, TypeError, ValueError, ZeroDivisionError):
            return False
        smaller = min(across, down)
        return math.isfinite(smaller) and smaller >= self.min_box_px


__all__ = ["OrbitCentreFrame"]
