"""Local terminal confirmation publication."""

from __future__ import annotations

from collections.abc import Callable

from navpy.args.logger_args import LogStatusDest
from navpy.logger.cache_logger import ILogger
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vision.target_identity import get_target_task_id


class LocalConfirmationPublisher:
    """Confirm one visual target and report that explicit local decision."""

    def __init__(
        self,
        confirm: Callable[[DetectedObject], None],
        logger: ILogger,
    ) -> None:
        self._confirm = confirm
        self._logger = logger

    def publish(self, target: DetectedObject) -> None:
        self._confirm(target)
        self._logger.info(
            f"CONFIRMED: T{get_target_task_id(target)} (vision nav local)",
            key="nav",
            dest=LogStatusDest.DRONE,
        )


__all__ = ["LocalConfirmationPublisher"]
