"""Arm the dock contact sensor once per approach episode.

NAV entry arms it for (vehicle, task, own dock uid, episode); NAV exit
disarms it. A new assignment or an approach restart is a new NAV entry, so
it re-arms with a new episode number and never carries a segment across.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Optional

from navpy.core.kernel.config.logger_args import LogStatusDest
from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.perception.detection.poi_identity import get_poi_task_id
from navpy.core.ports.detect_data import DetectedObject
from navpy.core.ports.dock_contact import (
    DockContactEvent,
    DockContactIdentity,
    DockContactSensor,
)


class FinalApproachDockContact:
    def __init__(
        self,
        sensor: Optional[DockContactSensor],
        vehicle_id: Callable[[], int],
        begin_episode: Callable[[DockContactIdentity, Optional[float]], bool],
        latch_docked: Callable[[DockContactEvent], bool],
        logger: ILogger,
        *,
        source_now_s: Callable[[], Optional[float]] = lambda: None,
        end_episode: Callable[[DockContactIdentity, Optional[float]], None] = (
            lambda _identity, _source_s: None
        ),
    ) -> None:
        self._sensor = sensor
        self._vehicle_id = vehicle_id
        self._begin_episode = begin_episode
        self._latch_docked = latch_docked
        self._logger = logger
        # The latest ATTITUDE boot time: evidence rows only, never a command.
        self._source_now_s = source_now_s
        self._end_episode = end_episode
        self._episode = 0
        self._armed: Optional[DockContactIdentity] = None

    def arm(self, poi: Optional[DetectedObject]) -> None:
        """NAV entry: expect contact with this POI's own dock only."""
        if self._sensor is None:
            return
        self._sensor.disarm()
        self._armed = None
        task_id = None if poi is None else get_poi_task_id(poi)
        if poi is None or task_id is None:
            return
        self._episode += 1
        identity = DockContactIdentity(
            vehicle=int(self._vehicle_id()),
            task_id=task_id,
            dock_uid=poi.identity.obj_id,
            episode=self._episode,
        )
        if self._begin_episode(identity, self._source_now_s()):
            self._armed = identity
            self._sensor.arm(identity, self._on_docked)

    def disarm(self) -> None:
        if self._sensor is not None:
            self._sensor.disarm()
        armed, self._armed = self._armed, None
        if armed is not None:
            self._end_episode(armed, self._source_now_s())

    def _on_docked(self, event: DockContactEvent) -> None:
        if not self._latch_docked(event):
            return
        identity = event.identity
        self._logger.info(
            f"DOCKED: task={identity.task_id} dock={identity.dock_uid} "
            f"episode={identity.episode} 3d={event.closest_3d_m:.2f}m",
            key="nav",
            dest=LogStatusDest.DRONE,
        )


__all__ = ["FinalApproachDockContact"]
