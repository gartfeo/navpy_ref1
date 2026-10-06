"""Synthetic confirmation-frame capture for simulator detections."""

from __future__ import annotations

from typing import Optional

import numpy as np
import pymap3d

from navpy.modules.common.models.location import Location
from navpy.modules.vision.confirmation_frame import (
    CONFIRMATION_FRAME_MAX_CENTER_SCORE,
    is_confirmation_frame_centered,
    should_replace_center_gated_confirmation_frame,
)
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.models.detection_components import ConfirmationEvidence
from navpy.modules.vision.sim.sim_camera_ports import CaptureCameraPort
from navpy.modules.vision.sim.sim_detector_state import SimCaptureState
from navpy.modules.vision.sim.sim_runtime_ports import (
    BoundingBox,
    DebugSink,
    OverlayPosition,
)
from navpy.modules.vision.simulation_object import SimulationObject
from navpy.modules.vision.vision_class_profile import get_class_detect_size


class ConfirmationCapture:
    """Own best-frame selection through exact camera and renderer ports."""

    def __init__(
        self,
        camera: CaptureCameraPort,
        state: SimCaptureState,
        debug: DebugSink,
    ) -> None:
        self._camera = camera
        self._state = state
        self._debug = debug

    def wants_frame(self) -> bool:
        renderer = self._state.renderer
        return bool(
            self._state.enabled
            and renderer is not None
            and renderer.is_available
        )

    def overlay_position(
        self,
        detection: DetectedObject,
        poi: SimulationObject,
        camera_location: Location,
    ) -> OverlayPosition:
        renderer = self._state.renderer
        if renderer is None:
            raise RuntimeError("confirmation renderer is unavailable")
        frame_size = self._camera.frame_size
        x_fraction = float(detection.pixel.u_px) / frame_size.width_px
        y_fraction = float(detection.pixel.v_px) / frame_size.height_px
        poi_ned = pymap3d.geodetic2ned(
            poi.g_loc.lat,
            poi.g_loc.lng,
            poi.g_loc.alt,
            camera_location.lat,
            camera_location.lng,
            camera_location.alt,
        )
        distance_m = float(np.linalg.norm(poi_ned))
        camera_matrix = self._camera.read_matrix()
        focal_y_px = float(camera_matrix[1, 1])
        projected_px = focal_y_px * get_class_detect_size(0) / max(distance_m, 1.0)
        sprite_height = renderer.sprite_height_for(poi.location_type)
        sprite_width = renderer.sprite_width_for(poi.location_type)
        scale = self.sprite_scale(projected_px, sprite_width, sprite_height)
        return x_fraction, y_fraction, scale

    def capture(
        self,
        detected_pois: list[DetectedObject],
        poi_positions: Optional[list[OverlayPosition]],
    ) -> None:
        renderer = self._state.renderer
        if renderer is None or poi_positions is None or not detected_pois:
            return
        location_type = next(
            (
                poi.classification.location_type
                for poi in detected_pois
                if poi.classification.location_type
            ),
            None,
        )
        result = renderer.generate_frame(poi_positions, location_type=location_type)
        if result is not None:
            frame, bounding_boxes = result
            self.store_candidates(
                detected_pois,
                poi_positions,
                frame,
                bounding_boxes,
            )
        self.attach_best_frames(detected_pois)

    def store_candidates(
        self,
        detected_pois: list[DetectedObject],
        poi_positions: list[OverlayPosition],
        frame: np.ndarray,
        bounding_boxes: list[BoundingBox],
    ) -> None:
        for index, detection in enumerate(detected_pois):
            if index >= len(bounding_boxes):
                continue
            x_fraction, y_fraction = poi_positions[index][:2]
            center_score = abs(x_fraction - 0.5) + abs(y_fraction - 0.5)
            stored = self._state.best_frames.get(detection.identity.obj_id)
            stored_bbox = None if stored is None else stored[1]
            stored_score = 0.0 if stored is None else stored[2]
            stored_context = 0 if stored is None else len(stored[3])
            candidate_bbox = bounding_boxes[index]
            if not should_replace_center_gated_confirmation_frame(
                candidate_bbox,
                center_score,
                stored_bbox,
                stored_score,
                candidate_context_count=len(bounding_boxes),
                stored_context_count=stored_context,
                max_center_score=CONFIRMATION_FRAME_MAX_CENTER_SCORE,
            ):
                continue
            self._state.best_frames[detection.identity.obj_id] = (
                frame.copy(),
                candidate_bbox,
                center_score,
                list(bounding_boxes),
            )
            self._debug(
                f"Updated best frame for P{detection.identity.obj_id} "
                f"(bbox_h={candidate_bbox[3]:.0f}, score={center_score:.2f}, "
                f"pois={len(bounding_boxes)})"
            )

    def attach_best_frames(self, detected_pois: list[DetectedObject]) -> None:
        for detection in detected_pois:
            stored = self._state.best_frames.get(detection.identity.obj_id)
            if stored is None:
                continue
            best_frame, best_bbox, best_score, all_bboxes = stored
            if not is_confirmation_frame_centered(
                best_score,
                CONFIRMATION_FRAME_MAX_CENTER_SCORE,
            ):
                continue
            detection.capture_confirmation(ConfirmationEvidence.capture(
                best_frame,
                best_bbox,
                all_bboxes,
                supports_frame=detection.confirmation.supports_frame,
                degraded=detection.confirmation.degraded,
            ))

    @staticmethod
    def sprite_scale(
        projected_px: float,
        sprite_width: int,
        sprite_height: int,
    ) -> float:
        if sprite_height <= 0 or sprite_width <= 0:
            return 0.02
        sprite_diagonal = float(np.hypot(sprite_width, sprite_height))
        raw_scale = projected_px / sprite_diagonal
        minimum_scale = max(1.0 / sprite_height, 1.0 / sprite_width)
        return min(0.8, max(minimum_scale, raw_scale))


__all__ = ["ConfirmationCapture"]
