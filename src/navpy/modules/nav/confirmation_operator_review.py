"""Operator review publication and zoom transition."""

from __future__ import annotations

from collections.abc import Callable

from navpy.args.logger_args import LogStatusDest
from navpy.logger.cache_logger import ILogger
from navpy.modules.nav.confirmation_legacy_prep import LegacyReviewPreparation
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.poi_identity import get_poi_task_id


class OperatorReviewPublisher:
    """Publish one review request after optional legacy preparation."""

    def __init__(
        self,
        final_approach_active: Callable[[], bool],
        review: Callable[[list[DetectedObject]], None],
        freeze_final_approach_zoom: Callable[[], bool],
        set_tracking_zoom: Callable[[bool], None],
        logger: ILogger,
        legacy: LegacyReviewPreparation | None,
    ) -> None:
        self._final_approach_active = final_approach_active
        self._review = review
        self._freeze_final_approach_zoom = freeze_final_approach_zoom
        self._set_tracking_zoom = set_tracking_zoom
        self._logger = logger
        self._legacy = legacy

    def publish(self, poi: DetectedObject) -> None:
        final_approach_active = self._final_approach_active()
        if final_approach_active and not self._freeze_final_approach_zoom():
            raise RuntimeError("final-approach review requires minimum zoom")
        if not final_approach_active and self._legacy is not None:
            self._legacy.prepare(poi)
        self._review([poi])
        self._logger.info(
            f"CONFIRMING: P{get_poi_task_id(poi)}",
            key="nav",
            dest=LogStatusDest.DRONE,
        )
        if not final_approach_active:
            self._set_tracking_zoom(False)


__all__ = ["OperatorReviewPublisher"]
