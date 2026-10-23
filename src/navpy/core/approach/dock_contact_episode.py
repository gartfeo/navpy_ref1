"""Arm the dock contact sensor once per approach episode.

NAV entry arms it for (vehicle, task, own dock uid, episode); NAV exit
disarms it. A new assignment or an approach restart is a new NAV entry, so
it re-arms with a new episode number and never carries a segment across.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Optional

from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.config.logger_args import LogStatusDest
from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.perception.detection.poi_identity import get_poi_task_id
from navpy.core.ports.detect_data import DetectedObject
from navpy.core.ports.dock_contact import (
    DockContactEvent,
    DockContactIdentity,
    DockContactSensor,
    DockEpisodeEnd,
)

# (armed identity, lifecycle accepted DOCKED) -> how NAV left the episode.
EpisodeEnding = Callable[[DockContactIdentity, bool], DockEpisodeEnd]


def _docked_only(_identity: DockContactIdentity, docked: bool) -> DockEpisodeEnd:
    return DockEpisodeEnd(docked=docked)



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
        ending: EpisodeEnding = _docked_only,
    ) -> None:
        self._sensor = sensor
        self._vehicle_id = vehicle_id
        self._begin_episode = begin_episode
        self._latch_docked = latch_docked
        self._logger = logger
        # The latest ATTITUDE boot time: evidence rows only, never a command.
        self._source_now_s = source_now_s
        self._end_episode = end_episode
        # Display only: the sensor's closest-approach record.
        self._ending = ending
        self._episode = 0
        self._armed: Optional[DockContactIdentity] = None
        # The armed episode's latched contact: the DOCKED state's source.
        self._lock = rt.Lock()
        self._docked: Optional[DockContactEvent] = None

    def arm(self, poi: Optional[DetectedObject]) -> None:
        """NAV entry: expect contact with this POI's own dock only."""
        if self._sensor is None:
            return
        # Detach first, under the lock, so the sensor's report and the latch
        # agree: a contact callback after this finds no armed episode.
        with self._lock:
            armed, self._armed = self._armed, None
            docked = self._docked is not None
            self._docked = None
        self._sensor.disarm(self._end(armed, docked))
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
            with self._lock:
                self._armed = identity
            self._sensor.arm(identity, self._on_docked)

    def disarm(self) -> Optional[DockContactEvent]:
        """NAV exit: stop expecting contact; return the episode's contact.

        The sensor publishes only a closest pass inside the net, and only
        for the armed identity, so the event is the DOCKED evidence.
        """
        with self._lock:
            armed, self._armed = self._armed, None
            docked, self._docked = self._docked, None
        if self._sensor is not None:
            self._sensor.disarm(self._end(armed, docked is not None))
        if armed is not None:
            self._end_episode(armed, self._source_now_s())
        return docked

    def _end(self, armed: Optional[DockContactIdentity], docked: bool) -> DockEpisodeEnd:
        """Read after the contact is detached; never breaks NAV cleanup."""
        if armed is None:
            return DockEpisodeEnd(docked=docked)
        try:
            return self._ending(armed, docked)
        except Exception as error:  # noqa: BLE001 - display evidence only
            self._logger.error("dock episode end unavailable", error)
            return DockEpisodeEnd(docked=docked)

    def _on_docked(self, event: DockContactEvent) -> None:
        # Latch and keep under one lock, so NAV exit sees both or neither.
        # Lock order: this lock, then the mode lock inside the latch.
        with self._lock:
            if event.identity != self._armed or not self._latch_docked(event):
                return
            self._docked = event
        identity = event.identity
        self._logger.info(
            f"DOCKED: task={identity.task_id} dock={identity.dock_uid} "
            f"episode={identity.episode} 3d={event.closest_3d_m:.2f}m",
            key="nav",
            dest=LogStatusDest.DRONE,
        )


__all__ = ["EpisodeEnding", "FinalApproachDockContact"]
