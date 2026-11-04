"""When a zoom reading proves the lens is settled at a calibrated step (HITL step 18).

The camera reports only a zoom reading (on a 0.1 grid); it does not say whether the lens is still moving. The
evidence here is the ordinary command and readback history of one gimbal connection session
(``GimbalCaptureHistory``: readings and zoom commands on the capture clock):

- The latest zoom command opens an epoch. Only an absolute command of a calibrated step can settle; a manual zoom
  or any other command leaves the lens at no known setting (unknown), and so does a session with no command yet.
- Arrival is the first reading after the command equal to the step's reading. A reading can equal the target while
  the lens still travels inside that 0.1 bin, so the step settles only after the reading has held for
  ``dwell_s`` and ``min_held_readings`` readings; a change before that is still travel and restarts the wait.
- Once settled, any reading change without a new command means the lens moved by itself: unknown until the next
  command.
- A frame at lookup time ``t`` is settled only when ``t >= arrival + dwell`` and a held reading arrived at or after
  ``t`` (the frame is bracketed by held readings). A frame before the command is refused too: earlier epochs are
  not kept.
- Freshness is never extended: a gap between consecutive readings or a newest reading older than its limit, or
  history lost between two snapshots, aborts the evidence for the rest of the session (latched; a reconnect starts
  a new session, which needs a new command and a new settle).

Readings, commands and judgements are all on ``rt.perf_counter`` receipts. Settling of autofocus is not modelled.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from typing import Any
from dataclasses import dataclass

from navpy.core.kernel import runtime_time as rt
from navpy.core.perception.camera.reading_intrinsics import zoom_tenths
from navpy.core.perception.zoom.fixed_zoom_steps import FixedZoomSteps, ZoomStep
from navpy.core.ports.gimbal_capture_history import (
    GimbalCaptureHistory,
    ZoomCommandSample,
    ZoomReadingSample,
)

STATUS_ZOOM_SETTLED = "zoom_settled"
STATUS_ZOOM_TRAVEL = "real_frame_zoom_travel"
STATUS_ZOOM_UNBRACKETED = "real_frame_zoom_unbracketed"
STATUS_ZOOM_UNKNOWN = "real_frame_zoom_unknown"
STATUS_ZOOM_ABORTED = "real_frame_zoom_aborted"
STATUS_ZOOM_HISTORY_UNAVAILABLE = "real_frame_zoom_history_unavailable"

ABORT_READING_GAP = "reading_gap"
ABORT_READING_AGE = "reading_age"
ABORT_HISTORY_LOST = "history_lost"


@dataclass(frozen=True)
class ZoomSettleLimits:
    """My values (HITL step 18), not measured; the run record freezes them.

    ``dwell_s``: a reading held this long proves the lens left its last 0.1 bin, at any travel speed above
    0.1 / dwell_s = 0.33 reading/s (the emulator seeks at 2 per s). ``min_held_readings``: three readings, so one
    late reply cannot settle a step. ``reading_gap_max_s`` / ``reading_age_max_s``: the 20 Hz poll gives a reading
    every 50 ms; 0.5 s matches step 11's frame-gap and frame-age limits.
    """

    dwell_s: float = 0.3
    min_held_readings: int = 3
    reading_gap_max_s: float = 0.5
    reading_age_max_s: float = 0.5

    def __post_init__(self) -> None:
        for name in ("dwell_s", "reading_gap_max_s", "reading_age_max_s"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) \
                    or value <= 0.0:
                raise ValueError(f"{name} must be a positive finite number")
        if isinstance(self.min_held_readings, bool) or not isinstance(self.min_held_readings, int) \
                or self.min_held_readings < 1:
            raise ValueError("min_held_readings must be a positive integer")


@dataclass(frozen=True)
class ZoomJudgement:
    status: str
    step: ZoomStep | None = None
    session: int | None = None
    command_sent_s: float | None = None
    arrival_seq: int | None = None
    arrival_s: float | None = None
    held_until_s: float | None = None
    abort_reason: str | None = None

    @property
    def settled(self) -> bool:
        return self.status == STATUS_ZOOM_SETTLED

    def record(self) -> dict[str, object]:
        return {
            "zoom_status": self.status,
            "zoom_step": None if self.step is None else self.step.command,
            "zoom_session": self.session,
            "zoom_command_sent_s": self.command_sent_s,
            "zoom_arrival_seq": self.arrival_seq,
            "zoom_arrival_s": self.arrival_s,
            "zoom_held_until_s": self.held_until_s,
            "zoom_abort_reason": self.abort_reason,
        }


class _Epoch:
    def __init__(self, command: ZoomCommandSample, step: ZoomStep | None) -> None:
        self.command = command
        self.step = step
        self.arrival: ZoomReadingSample | None = None
        self.held = 0
        self.held_until_s: float | None = None
        self.confirmed = False
        self.broken = False

    def reading(self, sample: ZoomReadingSample, limits: ZoomSettleLimits) -> None:
        if self.step is None or self.broken:
            return
        at_target = zoom_tenths(sample.reading) == self.step.reading_tenths
        if not at_target:
            if self.confirmed:
                self.broken = True          # the lens moved without a command
            else:
                self.arrival = None         # still travelling
                self.held = 0
                self.held_until_s = None
            return
        if self.arrival is None:
            self.arrival = sample
            self.held = 0
        self.held += 1
        self.held_until_s = sample.receipt_s
        if self.held >= limits.min_held_readings and sample.receipt_s - self.arrival.receipt_s >= limits.dwell_s:
            self.confirmed = True


class ZoomSettleEvidence:
    """Judges frames against the zoom history of the current gimbal session; shared by command and perception."""

    def __init__(self, steps: FixedZoomSteps, limits: ZoomSettleLimits | None = None) -> None:
        self.steps = steps
        self.limits = limits or ZoomSettleLimits()
        self._lock = rt.Lock()
        self._session: int | None = None
        self._reset()

    def _reset(self) -> None:
        self._epoch: _Epoch | None = None
        self._last_reading: ZoomReadingSample | None = None
        self._last_command: ZoomCommandSample | None = None
        self._aborted: str | None = None

    def judge(self, history: GimbalCaptureHistory | None, lookup_s: float, now_s: float) -> ZoomJudgement:
        """The zoom state of a frame looked up at ``lookup_s`` (receipt axis), given the newest history."""
        with self._lock:
            if not isinstance(history, GimbalCaptureHistory):
                return ZoomJudgement(STATUS_ZOOM_HISTORY_UNAVAILABLE)
            if self._session is not None and history.session < self._session:
                # Sessions only grow: a snapshot taken before a reconnect must not reset the newer session.
                return ZoomJudgement(STATUS_ZOOM_HISTORY_UNAVAILABLE, session=history.session)
            if history.session != self._session:
                self._session = history.session
                self._reset()
            if self._aborted is None:
                self._ingest(history)
            if self._aborted is None and self._last_reading is not None \
                    and now_s - self._last_reading.receipt_s > self.limits.reading_age_max_s:
                self._aborted = ABORT_READING_AGE
            return self._judge(lookup_s)

    def judge_latest(self, history: GimbalCaptureHistory | None, now_s: float) -> ZoomJudgement:
        """The zoom state at the newest reading: what the command side may decide from."""
        newest = now_s
        if isinstance(history, GimbalCaptureHistory):
            readings = [r.receipt_s for r in history.zoom_readings if r.session == history.session]
            if readings:
                newest = readings[-1]
        return self.judge(history, newest, now_s)

    def _ingest(self, history: GimbalCaptureHistory) -> None:
        readings = [r for r in history.zoom_readings if r.session == history.session]
        commands = [c for c in history.zoom_commands if c.session == history.session]
        if self._lost(readings, self._last_reading, lambda r: r.receipt_s) \
                or self._lost(commands, self._last_command, lambda c: c.sent_s):
            self._aborted = ABORT_HISTORY_LOST
            return
        last_reading_s = -math.inf if self._last_reading is None else self._last_reading.receipt_s
        last_command_s = -math.inf if self._last_command is None else self._last_command.sent_s
        events: list[tuple[float, int, object]] = [(r.receipt_s, 0, r) for r in readings if r.receipt_s > last_reading_s]
        events += [(c.sent_s, 1, c) for c in commands if c.sent_s > last_command_s]
        # A reading received at the same instant as a command still shows the old state.
        for _time, _kind, event in sorted(events, key=lambda item: (item[0], item[1])):
            if isinstance(event, ZoomCommandSample):
                self._last_command = event
                self._epoch = _Epoch(event, self.steps.by_command(event.absolute_tenths))
                continue
            if not isinstance(event, ZoomReadingSample):
                continue
            if self._last_reading is not None \
                    and event.receipt_s - self._last_reading.receipt_s > self.limits.reading_gap_max_s:
                self._aborted = ABORT_READING_GAP
                return
            self._last_reading = event
            if self._epoch is not None:
                self._epoch.reading(event, self.limits)

    @staticmethod
    def _lost(samples: list, last: object, time_of: Callable[[Any], float]) -> bool:
        """True when the snapshot no longer holds the last processed sample and starts after it: samples between
        the two snapshots fell out of the bounded history unseen."""
        if last is None or not samples:
            return False
        return time_of(samples[0]) > time_of(last) and last not in samples

    def _judge(self, lookup_s: float) -> ZoomJudgement:
        session = self._session
        if self._aborted is not None:
            return ZoomJudgement(STATUS_ZOOM_ABORTED, session=session, abort_reason=self._aborted)
        epoch = self._epoch
        if epoch is None or epoch.step is None or epoch.broken:
            return ZoomJudgement(STATUS_ZOOM_UNKNOWN, session=session,
                                 command_sent_s=None if epoch is None else epoch.command.sent_s)
        base = dict(step=epoch.step, session=session, command_sent_s=epoch.command.sent_s,
                    arrival_seq=None if epoch.arrival is None else epoch.arrival.seq,
                    arrival_s=None if epoch.arrival is None else epoch.arrival.receipt_s,
                    held_until_s=epoch.held_until_s)
        if not epoch.confirmed or epoch.arrival is None or lookup_s < epoch.command.sent_s \
                or lookup_s < epoch.arrival.receipt_s + self.limits.dwell_s:
            return ZoomJudgement(STATUS_ZOOM_TRAVEL, **base)
        if epoch.held_until_s is None or epoch.held_until_s < lookup_s:
            return ZoomJudgement(STATUS_ZOOM_UNBRACKETED, **base)
        return ZoomJudgement(STATUS_ZOOM_SETTLED, **base)


__all__ = [
    "ABORT_HISTORY_LOST",
    "ABORT_READING_AGE",
    "ABORT_READING_GAP",
    "STATUS_ZOOM_ABORTED",
    "STATUS_ZOOM_HISTORY_UNAVAILABLE",
    "STATUS_ZOOM_SETTLED",
    "STATUS_ZOOM_TRAVEL",
    "STATUS_ZOOM_UNBRACKETED",
    "STATUS_ZOOM_UNKNOWN",
    "ZoomJudgement",
    "ZoomSettleEvidence",
    "ZoomSettleLimits",
]
