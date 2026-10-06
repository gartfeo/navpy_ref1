"""Local final-approach confirmation publication."""

from __future__ import annotations

from collections.abc import Callable

from navpy.args.logger_args import LogStatusDest
from navpy.logger.cache_logger import ILogger
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.poi_identity import get_poi_task_id


class LocalConfirmationPublisher:
    """Confirm one visual POI and report that explicit local decision."""

    def __init__(
        self,
        confirm: Callable[[DetectedObject], None],
        logger: ILogger,
    ) -> None:
        self._confirm = confirm
        self._logger = logger

    def publish(self, poi: DetectedObject) -> None:
        self._confirm(poi)
        self._logger.info(
            f"CONFIRMED: P{get_poi_task_id(poi)} (vision nav local)",
            key="nav",
            dest=LogStatusDest.DRONE,
        )


__all__ = ["LocalConfirmationPublisher"]
