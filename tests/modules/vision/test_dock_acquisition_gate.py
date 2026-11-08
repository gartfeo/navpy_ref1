"""The whole-dock detector's acquisition and ambiguity gates (HITL step 23b; hitl_dock_detector.md section 3)."""
from __future__ import annotations

from dataclasses import replace
from unittest.mock import Mock

import pytest

from navpy.core.perception.tracking.dock_acquisition_gate import (
    DockAcquisitionGate,
    DockAcquisitionSettings,
    acquisition_block,
)
from navpy.core.ports.tracks import TrackedObject

W, H = 1920, 1080
SQUASH = DockAcquisitionSettings(min_lock_s=1.0, min_box_px=16.0, min_confidence=0.45, dock_class_id=0,
                                 input_scale=lambda w, h: (704 / w, 704 / h))


def _track(track_id: int = 7, w: float = 60.0, h: float = 40.0, confidence: float = 0.8, missed: int = 0,
           confirmed: bool = True, class_id: int = 0) -> TrackedObject:
    return TrackedObject(id=track_id, cx=900.0, cy=500.0, w=w, h=h, confidence=confidence, class_id=class_id,
                         age=5, hits=5, missed=missed, is_confirmed=confirmed, timestamp=0.0, vx=0.0, vy=0.0)


def _run(gate, frames, step_s=0.1):
    """Feed frames (each a list of tracks) at step_s; the last result."""
    out = None
    for i, tracks in enumerate(frames):
        out = gate.admit(tracks, W, H, i * step_s)
    return out


def test_one_dock_held_for_min_lock_s_is_acquired_then_passes_alone():
    log = []
    gate = DockAcquisitionGate(SQUASH, log.append)
    assert _run(gate, [[_track()]] * 10) == ([], True)                 # 0.9 s: not yet, the search holds
    assert gate.admit([_track()], W, H, 1.0) == ([_track()], True)     # 1.0 s: acquired
    assert gate.acquired_id == 7 and log[-1] == "DOCK ACQ: track 7 acquired"


def test_a_short_track_is_not_acquired():
    gate = DockAcquisitionGate(SQUASH, lambda _m: None)
    assert _run(gate, [[_track()]] * 5 + [[]] + [[_track()]] * 6) == ([], True)   # the gap restarts the dwell
    assert gate.acquired_id is None


def test_two_dock_tracks_mean_no_acquisition_logged_once():
    log = []
    gate = DockAcquisitionGate(SQUASH, log.append)
    assert _run(gate, [[_track(7), _track(8)]] * 30) == ([], True)
    assert gate.acquired_id is None
    assert log == ["DOCK ACQ: 2 dock tracks in view; no acquisition, the search holds"]


def test_a_second_track_restarts_the_dwell():
    gate = DockAcquisitionGate(SQUASH, lambda _m: None)
    frames = [[_track(7)]] * 8 + [[_track(7), _track(8)]] + [[_track(7)]] * 9
    assert _run(gate, frames) == ([], True)                           # the dwell restarted at 0.9 s
    assert gate.admit([_track(7)], W, H, 1.8) == ([], True)
    assert gate.admit([_track(7)], W, H, 2.0) == ([_track(7)], True)


@pytest.mark.parametrize("bad", [
    {"w": 40.0, "h": 40.0},           # 40 px across the 1920 frame is 14.7 input px: under the floor
    {"h": 24.0},                      # 24 px down the 1080 frame is 15.6 input px
    {"confidence": 0.44},
    {"confirmed": False},
    {"class_id": 3},                  # not the dock class: no candidate either
    {"missed": 1},                    # coasting: not in view
])
def test_an_ineligible_track_is_never_acquired(bad):
    gate = DockAcquisitionGate(SQUASH, lambda _m: None)
    tracks, candidate = _run(gate, [[_track(**bad)]] * 30)
    assert tracks == [] and gate.acquired_id is None
    assert candidate is (bad.get("class_id", 0) == 0 and bad.get("missed", 0) == 0)


def test_the_box_floor_is_in_the_detector_input_pixels():
    tiles = replace(SQUASH, input_scale=lambda w, h: (1.0, 1.0))
    gate = DockAcquisitionGate(tiles, lambda _m: None)
    assert _run(gate, [[_track(w=40.0, h=20.0)]] * 11)[0] != []       # 20 native px: above 16 on a tile


def test_after_acquisition_box_size_and_other_tracks_do_not_matter():
    gate = DockAcquisitionGate(SQUASH, lambda _m: None)
    _run(gate, [[_track()]] * 11)
    tiny = _track(w=2.0, h=2.0, confidence=0.1, missed=3)
    assert gate.admit([tiny, _track(8)], W, H, 2.0) == ([tiny], True)  # only the acquired track, whatever its box


def test_a_lost_track_is_acquired_again_from_the_start():
    log = []
    gate = DockAcquisitionGate(SQUASH, log.append)
    _run(gate, [[_track()]] * 11)
    assert gate.admit([], W, H, 2.0) == ([], False)
    assert gate.acquired_id is None and "DOCK ACQ: track 7 lost; acquiring again" in log
    assert gate.admit([_track()], W, H, 2.1) == ([], True)


@pytest.mark.parametrize("block", [
    {"min_lock_s": 1.0}, {"min_lock_s": 0.0, "min_box_px": 16, "recall_min_box_px": 12, "recall_evidence": "x"},
    {"min_lock_s": 1.0, "min_box_px": True, "recall_min_box_px": 12, "recall_evidence": "x"},
    {"min_lock_s": 1.0, "min_box_px": 16, "recall_min_box_px": 12, "recall_evidence": ""},
    {"min_lock_s": 1.0, "min_box_px": 16, "recall_min_box_px": float("nan"), "recall_evidence": "x"},
])
def test_malformed_acquisition_blocks_are_refused(block):
    with pytest.raises(ValueError):
        acquisition_block({"acquisition": block})
    assert acquisition_block({}) is None


def test_the_processor_holds_tracks_back_and_the_search_holds_on_a_candidate():
    """Through the real association processor and navigation sink: no POI before acquisition, the sweep held."""
    from navpy.core.perception.pipeline.real_detector_state import FreshnessPolicy
    from navpy.core.perception.pipeline.real_tracking import TrackingNavigationSink
    from navpy.core.perception.pipeline.real_tracking_batch import TrackingAssociationProcessor, TrackingModels
    from navpy.core.perception.tracking.poi_lock import PoiLock

    track = _track()
    identity = Mock()
    identity.update.return_value = [track]
    tracker = Mock()
    tracker.update.return_value = [track]
    poi_lock = PoiLock()
    generation = Mock()
    generation.is_current.return_value = True
    recovery = Mock()
    recovery.compute_embeddings.return_value = None
    deep = Mock()
    deep.take_for_tracking.return_value = []
    processor = TrackingAssociationProcessor(TrackingModels(tracker, identity, poi_lock), recovery, deep, Mock(),
                                             generation, True, DockAcquisitionGate(SQUASH, lambda _m: None))
    batch = Mock()
    batch.detections = []
    batch.association.frame = object()
    batch.association.frame_width, batch.association.frame_height = W, H
    first = processor.process(batch, 0.0)
    assert first.tracks == () and first.locked is None and first.candidate is True
    assert processor.process(batch, 1.0).tracks == (track,)

    navigation = Mock()
    navigation.tracking_obj_id = None
    TrackingNavigationSink(navigation, FreshnessPolicy(1 / 60)).update([], new_frame=True, candidate=True)
    navigation.search_tick.assert_called_once_with(True, True)
