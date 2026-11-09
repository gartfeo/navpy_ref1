"""Acquisition order before confirm and NAV (HITL step 25).

search -> own detection -> fresh visual lock -> ORBIT on the own-detected pose -> confirm -> NAV.

A navigation task's start (own detection; search stops on it, step 21) only marks the POI: the ORBIT
waits for the first fresh visual lock and is then placed from the newest frame of the task that the
centre-frame rule accepts (board centre or ground ray, step 22; never truth). Without the whole-dock
detector every frame is accepted, so that is the lock frame. With it (HITL step 23b), only a frame whose
box would still meet the acquisition floor at the search step, with a frame-atomic pose, is accepted, so
the first-detection distance the pre-series check bounds also bounds the frame the ground ray is taken
at, however late the task started or the lock came; until one arrives the orbit waits, and a frame the
orbit start refuses is dropped. Confirm and NAV each need the lock fresh on this tick and the orbit
established. Whether the gates are still held (the lock kept through a blip, the orbit established) is
published each tick to the confirmation lock (``publish``), which refuses CONFIRMED from any path
while it is False and withdraws a review under way. When a task's confirm releases NAV, ``released``
marks it; each NAV entry attempt takes the mark when it starts (``entry_started``), and the attempt that
completes (``entered``) reports it once (``on_release``: HITL step 23b's floor release, a mission-state
event). A failed attempt has used the mark, so a retry releases the floor only after a new confirm release,
which passes the gates on its tick; a held retry (no confirm on its tick) leaves the floor armed (fail
closed), however long or short the entry takes. A tick on which the gates do not hold also drops the mark.
Without the profile's gates, ``OPEN_ACQUISITION`` keeps the old order and reports nothing.
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

    def released(self, poi: DetectedObject) -> None: ...

    def entry_started(self) -> None: ...

    def entered(self) -> None: ...


class OrbitStart(Protocol):
    def prepare(self, poi: DetectedObject) -> Optional[OrbitCommand]: ...


class _OpenAcquisition:
    """No gates: confirm and NAV as before."""

    def ready(self, poi: DetectedObject | None) -> bool:
        return True

    def released(self, poi: DetectedObject) -> None:
        return None

    def entry_started(self) -> None:
        return None

    def entered(self) -> None:
        return None


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
        on_release: Callable[[int], None] = lambda _task_id: None,
        centre_frame: Callable[[DetectedObject], bool] = lambda _poi: True,
    ) -> None:
        self._lock = lock
        self._orbit = orbit
        self._orbit_start = orbit_start
        self._logger = logger
        self._publish = publish
        self._release = _FloorRelease(on_release)
        self._centre_frame = centre_frame
        self._task_id: int | None = None
        self._centre_poi: DetectedObject | None = None
        self._established = False
        self._reported: set[str] = set()

    def begin(self, poi: DetectedObject) -> None:
        """A navigation task started on an own detection: wait for the lock before the ORBIT."""
        self._task_id = get_poi_task_id(poi)
        self._centre_poi = None
        self._take_centre(poi)
        self._lock.reset()
        self._orbit.reset()
        self._reported = set()
        self._release.reset()
        self._publish(self._task_id, False)
        self._report("detected", f"P{self._task_id}: own detection, waiting for a fresh lock")

    def ready(self, poi: DetectedObject | None) -> bool:
        """Advance on this tick's detection (None: no detection); True when confirm or NAV may proceed."""
        ready = self._advance(poi)
        if not ready:
            self._release.pending = False  # the gates do not hold: a confirm must release NAV again
        if self._task_id is not None:
            # Published: whether the gates are still held, through a blip of the lock; a loss
            # withdraws or revokes the confirmation, so the operator is asked again only then.
            self._publish(self._task_id, self._lock.kept and self._established)
        return ready

    def released(self, poi: DetectedObject) -> None:
        """The task's confirm released NAV: report it at the NAV entry that completes (``entered``)."""
        if self._task_id is not None and get_poi_task_id(poi) == self._task_id:
            self._release.mark()

    def entry_started(self) -> None:
        """A NAV entry attempt starts: it takes the confirm's mark (a failed attempt has used it)."""
        self._release.start_attempt()

    def entered(self) -> None:
        """The NAV entry attempt completed: report its release, once per task."""
        if self._task_id is not None and self._release.take():
            self._report("released", f"P{self._task_id}: confirm released NAV, NAV entered")
            self._release.on_release(self._task_id)

    def _advance(self, poi: DetectedObject | None) -> bool:
        # The orbit is watched on every tick, frames or not: a vision gap is not a step off the circle.
        established = self._established = self._orbit.established()
        if self._task_id is None:
            return False
        if poi is not None and get_poi_task_id(poi) != self._task_id:
            self._lock.reset()  # another POI's frame: this lock is lost
            return False
        if poi is not None and not self._orbit.commanded:
            self._take_centre(poi)
        locked = self._lock.locked(poi)  # no detection on this tick: a blip of the lock
        if locked:
            self._report("locked", f"P{self._task_id}: fresh visual lock")
        if locked and not self._orbit.commanded and self._centre_poi is not None:
            command = self._orbit_start.prepare(self._centre_poi)
            if command is None:
                self._centre_poi = None  # refused: the next accepted frame is tried
            else:
                self._orbit.start(command)
                self._report(
                    "orbit", f"P{self._task_id}: ORBIT r={command.radius_m:.0f}m on the own-detected pose",
                )
                established = self._established = self._orbit.established()
        if established:
            self._report("established", f"P{self._task_id}: orbit established")
        return locked and established

    def _take_centre(self, poi: DetectedObject) -> None:
        """Keep the newest frame of this task the centre-frame rule accepts."""
        if self._centre_frame(poi):
            self._centre_poi = poi

    def _report(self, stage: str, text: str) -> None:
        if stage in self._reported:
            return
        self._reported.add(stage)
        self._logger.info(f"ACQ {stage}: {text}", key="nav")


class _FloorRelease:
    """A task's floor release: marked by the confirm that releases NAV, reported once by the entry that completes."""

    def __init__(self, on_release: Callable[[int], None]) -> None:
        self.on_release = on_release
        self.pending = False      # a confirm released NAV since the last entry attempt started
        self.attempt = False      # the entry attempt under way took a mark
        self.done = False

    def reset(self) -> None:
        self.pending = self.attempt = self.done = False

    def mark(self) -> None:
        self.pending = True

    def start_attempt(self) -> None:
        self.attempt, self.pending = self.pending, False

    def take(self) -> bool:
        """True once per task, when the completed attempt took a mark."""
        if self.done or not self.attempt:
            return False
        self.attempt, self.done = False, True
        return True


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
