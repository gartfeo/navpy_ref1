from __future__ import annotations

from navpy.modules.vision.models.detect_data import DetectedObject


class DetectResponse:
    def __init__(
            self,
            detected_pois: list[DetectedObject],
            primary_poi: DetectedObject | None = None,
    ):
        self.detected_pois = detected_pois
        self.primary_poi = primary_poi
