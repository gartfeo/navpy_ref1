"""Operator review publication and zoom transition."""

from __future__ import annotations

from collections.abc import Callable

from navpy.args.logger_args import LogStatusDest
from navpy.logger.cache_logger import ILogger
from navpy.modules.nav.confirmation_legacy_prep import LegacyReviewPreparation
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.target_identity import get_target_task_id


class OperatorReviewPublisher:
    """Publish one review request after optional legacy preparation."""

    def __init__(
        self,
        terminal_active: Callable[[], bool],
        review: Callable[[list[DetectedObject]], None],
        freeze_terminal_zoom: Callable[[], bool],
        set_tracking_zoom: Callable[[bool], None],
        logger: ILogger,
        legacy: LegacyReviewPreparation | None,
    ) -> None:
        self._terminal_active = terminal_active
        self._review = review
        self._freeze_terminal_zoom = freeze_terminal_zoom
        self._set_tracking_zoom = set_tracking_zoom
        self._logger = logger
        self._legacy = legacy

    def publish(self, target: DetectedObject) -> None:
        terminal_active = self._terminal_active()
        if terminal_active and not self._freeze_terminal_zoom():
            raise RuntimeError("terminal review requires minimum zoom")
        if not terminal_active and self._legacy is not None:
            self._legacy.prepare(target)
        self._review([target])
        self._logger.info(
            f"CONFIRMING: T{get_target_task_id(target)}",
            key="nav",
            dest=LogStatusDest.DRONE,
        )
        if not terminal_active:
            self._set_tracking_zoom(False)


__all__ = ["OperatorReviewPublisher"]
