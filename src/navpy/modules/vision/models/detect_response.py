from __future__ import annotations

from navpy.modules.vision.models.detect_data import DetectedObject


class DetectResponse:
    def __init__(
            self,
            detected_targets: list[DetectedObject],
            primary_target: DetectedObject | None = None,
    ):
        self.detected_targets = detected_targets
        self.primary_target = primary_target
