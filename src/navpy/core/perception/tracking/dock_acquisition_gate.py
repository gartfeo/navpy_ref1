"""Acquisition and ambiguity gates of the whole-dock detector (HITL step 23b; hitl_dock_detector.md section 3).

A box detector has no fiducial check, so a track becomes the POI only once it is a clear, single dock:

- acquisition: one dock track, confirmed, seen in every frame for ``min_lock_s`` (the tracking clock), each frame with
  confidence at least the detector's frozen threshold and the box's smaller side at least ``min_box_px`` in the
  detector's own input pixels (full-frame pixels times its input scale). The box-size floor bounds the
  first-detection distance, and so the ground-ray orbit centre's error (section 4.1);
- ambiguity: more than one dock track in view means no acquisition (the search holds, it is logged), and after
  acquisition it holds every track back on that frame (fail closed: the lock, orbit and confirm gates lose their
  frames, so confirmation and NAV wait or end), for as long as more than one is in view.

Until acquisition the gate passes no track on, so no POI exists, nothing locks and no navigation task starts; it
reports whether a dock candidate is in view, so the search sweep holds still on it (step 21) while the dwell runs.
Once acquired, only that track passes, whatever its box: box size is detector output used here, in acquisition, and
never reaches the NAV command or completion paths (section 4.2). While the acquired track is out of the identity
layer's view the acquisition is kept, so the same identity passes again when it returns, whatever its box;
another dock in view meanwhile is held back and never replaces it. The acquisition ends only when the identity's
ownership is released: the detector's ``stop_tracking`` (the same call that resets the POI lock, on task teardown
or rejection), its pipeline reset (the tracker and identity restart their ids), or, while it is out of view,
neither a task nor the POI lock owning it (the lock's own loss budget spent, before any task or after its
``stop_tracking``: ``release_unowned``). A task owns the identity from the detector's ``start_tracking`` on it
(``own``) until ``stop_tracking`` (``reset``). A lock guards the state: the
tracking thread admits, the navigation thread releases.

The profile sets, beside its rfdetr block::

    "acquisition": {"min_lock_s": 1.0, "min_box_px": 16.0, "recall_min_box_px": 16.0, "recall_evidence": "..."}

``recall_min_box_px`` and its evidence pointer are the measured detection limit the HITL harness's pre-series
detector check reads; navpy does not use them.
"""
from __future__ import annotations

import math
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass

from navpy.core.kernel import runtime_time as rt
from navpy.core.ports.tracks import TrackedObject

ACQUISITION_KEY = "acquisition"
_FIELDS = ("min_lock_s", "min_box_px", "recall_min_box_px", "recall_evidence")

InputScale = Callable[[int, int], tuple[float, float]]


@dataclass(frozen=True)
class DockAcquisitionSettings:
    min_lock_s: float
    min_box_px: float
    min_confidence: float
    dock_class_id: int
    # The detector's input pixels per full-frame pixel, across and down, for a frame of this size.
    input_scale: InputScale


def acquisition_block(detector: Mapping[str, object]) -> Mapping[str, object] | None:
    """The profile's detector.acquisition block, validated; None when it sets none."""
    block = detector.get(ACQUISITION_KEY) if isinstance(detector, Mapping) else None
    if block is None:
        return None
    if not isinstance(block, Mapping) or set(block) != set(_FIELDS):
        raise ValueError(f"detector.{ACQUISITION_KEY} must set exactly {list(_FIELDS)}")
    for name in _FIELDS[:3]:
        value = block[name]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"detector.{ACQUISITION_KEY}.{name} must be a positive number")
    if not isinstance(block["recall_evidence"], str) or not block["recall_evidence"]:
        raise ValueError(f"detector.{ACQUISITION_KEY}.recall_evidence must name the measurement")
    return block


class DockAcquisitionGate:
    """Pass the tracks on only once one dock track is acquired; then only that track."""

    def __init__(self, settings: DockAcquisitionSettings, log: Callable[[str], None]) -> None:
        self._settings = settings
        self._log = log
        self._mutex = rt.RLock()
        self._owned: int | None = None  # the identity a task tracks (start_tracking), until stop_tracking
        self._acquired: int | None = None
        self._dwell_id: int | None = None
        self._since_s: float | None = None
        self._ambiguous = False

    @property
    def acquired_id(self) -> int | None:
        return self._acquired

    def reset(self) -> None:
        """Forget the acquisition and the dwell (stop_tracking, or the pipeline reset: track ids start again)."""
        with self._mutex:
            self._acquired = self._dwell_id = self._since_s = self._owned = None
            self._ambiguous = False

    def own(self, obj_id: int) -> None:
        """A task tracks this identity (the detector's start_tracking): never released while it is out of view."""
        with self._mutex:
            self._owned = int(obj_id)

    def release_unowned(self, owner_id: int | None, present_ids: set[int]) -> None:
        """Release the acquired identity when it is out of view and neither a task nor the POI lock owns it."""
        with self._mutex:
            if self._acquired is None or self._acquired == self._owned or owner_id == self._acquired \
                    or self._acquired in present_ids:
                return
            self._log(f"DOCK ACQ: track {self._acquired} out of view, owned by no task or lock; released")
            self._acquired = None

    def admit(self, tracks: Sequence[TrackedObject], width: int, height: int,
              now_s: float) -> tuple[list[TrackedObject], bool]:
        """This frame's tracks to pass on, and whether a dock candidate is in view."""
        with self._mutex:
            return self._admit(tracks, width, height, now_s)

    def _admit(self, tracks: Sequence[TrackedObject], width: int, height: int,
               now_s: float) -> tuple[list[TrackedObject], bool]:
        docks = [t for t in tracks if t.class_id == self._settings.dock_class_id and t.missed == 0]
        self._report_ambiguity(len(docks))
        if len(docks) > 1:
            self._dwell_id = self._since_s = None
            return [], True                     # ambiguous: nothing passes while more than one is in view
        if self._acquired is not None:
            kept = [track for track in tracks if track.id == self._acquired]
            self._dwell_id = self._since_s = None
            return kept, bool(kept or docks)    # the acquired identity, whatever its box; never another
        if not docks or not self._eligible(docks[0], width, height):  # one dock: more returned above
            self._dwell_id = self._since_s = None
            return [], bool(docks)
        dock = docks[0]
        if dock.id != self._dwell_id:
            self._dwell_id, self._since_s = dock.id, float(now_s)
        if float(now_s) - self._since_s < self._settings.min_lock_s:
            return [], True
        self._acquired, self._dwell_id, self._since_s = dock.id, None, None
        self._log(f"DOCK ACQ: track {dock.id} acquired")
        return [dock], True

    def _eligible(self, track: TrackedObject, width: int, height: int) -> bool:
        scale_x, scale_y = self._settings.input_scale(width, height)
        smaller = min(float(track.w) * scale_x, float(track.h) * scale_y)
        return (bool(track.is_confirmed) and float(track.confidence) >= self._settings.min_confidence
                and math.isfinite(smaller) and smaller >= self._settings.min_box_px)

    def _report_ambiguity(self, count: int) -> None:
        ambiguous = count > 1
        if ambiguous and not self._ambiguous:
            self._log(f"DOCK ACQ: {count} dock tracks in view; nothing passes"
                      f"{'' if self._acquired is None else f' (track {self._acquired} acquired)'}")
        self._ambiguous = ambiguous


__all__ = ["ACQUISITION_KEY", "DockAcquisitionGate", "DockAcquisitionSettings", "InputScale", "acquisition_block"]
