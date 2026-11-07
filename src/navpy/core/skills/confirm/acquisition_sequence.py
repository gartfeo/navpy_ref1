"""Acquisition order before confirm and NAV (HITL step 25).

search -> own detection -> fresh visual lock -> ORBIT on the own-detected pose -> confirm -> NAV.

A navigation task's start (own detection; search stops on it, step 21) only marks the POI: the ORBIT
waits for the first fresh visual lock and is then placed from that frame's own pose (board centre or
ground ray, step 22; never truth). Confirm and NAV each need the lock fresh on this tick and the orbit
established. Whether the gates are still held (the lock kept through a blip, the orbit established) is
published each tick to the confirmation lock (``publish``), which refuses CONFIRMED from any path
while it is False and withdraws a review under way. Without the profile's
gates, ``OPEN_ACQUISITION`` keeps the old order.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Optional, Protocol

from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.ports.detect_data import DetectedObject
from navpy.core.perception.detection.poi_identity import get_poi_task_id
from navpy.core.skills.confirm.fresh_visual_lock import FreshVisualLock
from navpy.core.skills.confirm.orbit_established import OrbitCommand, OrbitEstablished


class AcquisitionGate(Protocol):
    def ready(self, poi: DetectedObject | None) -> bool: ...


class OrbitStart(Protocol):
    def prepare(self, poi: DetectedObject) -> Optional[OrbitCommand]: ...


class _OpenAcquisition:
    """No gates: confirm and NAV as before."""

    def ready(self, poi: DetectedObject | None) -> bool:
        return True


OPEN_ACQUISITION: AcquisitionGate = _OpenAcquisition()


class AcquisitionSequence:
    """Order lock, orbit, confirm and NAV for one navigation task."""

    def __init__(
        self,
        lock: FreshVisualLock,
        orbit: OrbitEstablished,
        orbit_start: OrbitStart,
        logger: ILogger,
        publish: Callable[[int, bool], None],
    ) -> None:
        self._lock = lock
        self._orbit = orbit
        self._orbit_start = orbit_start
        self._logger = logger
        self._publish = publish
        self._task_id: int | None = None
        self._established = False
        self._reported: set[str] = set()

    def begin(self, poi: DetectedObject) -> None:
        """A navigation task started on an own detection: wait for the lock before the ORBIT."""
        self._task_id = get_poi_task_id(poi)
        self._lock.reset()
        self._orbit.reset()
        self._reported = set()
        self._publish(self._task_id, False)
        self._report("detected", f"P{self._task_id}: own detection, waiting for a fresh lock")

    def ready(self, poi: DetectedObject | None) -> bool:
        """Advance on this tick's detection (None: no detection); True when confirm or NAV may proceed."""
        ready = self._advance(poi)
        if self._task_id is not None:
            # Published: whether the gates are still held, through a blip of the lock; a loss
            # withdraws or revokes the confirmation, so the operator is asked again only then.
            self._publish(self._task_id, self._lock.kept and self._established)
        return ready

    def _advance(self, poi: DetectedObject | None) -> bool:
        # The orbit is watched on every tick, frames or not: a vision gap is not a step off the circle.
        established = self._established = self._orbit.established()
        if self._task_id is None:
            return False
        if poi is not None and get_poi_task_id(poi) != self._task_id:
            self._lock.reset()  # another POI's frame: this lock is lost
            return False
        locked = self._lock.locked(poi)  # no detection on this tick: a blip of the lock
        if locked:
            self._report("locked", f"P{self._task_id}: fresh visual lock")
        if locked and not self._orbit.commanded:
            command = self._orbit_start.prepare(poi)
            if command is not None:
                self._orbit.start(command)
                self._report(
                    "orbit", f"P{self._task_id}: ORBIT r={command.radius_m:.0f}m on the own-detected pose",
                )
                established = self._established = self._orbit.established()
        if established:
            self._report("established", f"P{self._task_id}: orbit established")
        return locked and established

    def _report(self, stage: str, text: str) -> None:
        if stage in self._reported:
            return
        self._reported.add(stage)
        self._logger.info(f"ACQ {stage}: {text}", key="nav")


class GatedOrbitStart:
    """Stand in for the task start's ORBIT: only mark the POI; the sequence places the orbit."""

    def __init__(self, sequence: AcquisitionSequence) -> None:
        self._sequence = sequence

    def prepare(self, poi: DetectedObject) -> None:
        self._sequence.begin(poi)


__all__ = [
    "AcquisitionGate",
    "AcquisitionSequence",
    "GatedOrbitStart",
    "OPEN_ACQUISITION",
    "OrbitStart",
]
