"""Operator review publication and zoom transition."""

from __future__ import annotations

from collections.abc import Callable

from navpy.core.kernel.config.logger_args import LogStatusDest
from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.skills.geo_approach.confirmation_legacy_prep import LegacyReviewPreparation
from navpy.core.ports.detect_data import DetectedObject
from navpy.core.perception.detection.poi_identity import get_poi_task_id


class OperatorReviewPublisher:
    """Publish one review request after optional legacy preparation."""

    def __init__(
        self,
        guidance_active: Callable[[], bool],
        review: Callable[[list[DetectedObject]], None],
        freeze_guidance_zoom: Callable[[], bool],
        set_tracking_zoom: Callable[[bool], None],
        logger: ILogger,
        legacy: LegacyReviewPreparation | None,
    ) -> None:
        self._guidance_active = guidance_active
        self._review = review
        self._freeze_guidance_zoom = freeze_guidance_zoom
        self._set_tracking_zoom = set_tracking_zoom
        self._logger = logger
        self._legacy = legacy

    def publish(self, poi: DetectedObject) -> None:
        guidance_active = self._guidance_active()
        if guidance_active and not self._freeze_guidance_zoom():
            raise RuntimeError("guidance review requires minimum zoom")
        if not guidance_active and self._legacy is not None:
            self._legacy.prepare(poi)
        self._review([poi])
        self._logger.info(
            f"CONFIRMING: P{get_poi_task_id(poi)}",
            key="nav",
            dest=LogStatusDest.DRONE,
        )
        if not guidance_active:
            self._set_tracking_zoom(False)


__all__ = ["OperatorReviewPublisher"]
