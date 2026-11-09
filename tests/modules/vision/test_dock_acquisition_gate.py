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
    assert log == ["DOCK ACQ: 2 dock tracks in view; nothing passes"]


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


def test_after_acquisition_box_size_and_confidence_do_not_matter():
    gate = DockAcquisitionGate(SQUASH, lambda _m: None)
    _run(gate, [[_track()]] * 11)
    tiny = _track(w=2.0, h=2.0, confidence=0.1)
    assert gate.admit([tiny], W, H, 2.0) == ([tiny], True)            # the acquired identity, whatever its box
    coasting = _track(missed=3)
    assert gate.admit([coasting, _track(8, missed=2)], W, H, 2.1) == ([coasting], True)


def test_a_second_dock_in_view_after_acquisition_holds_every_track_back():
    """Codex round 1: the ambiguity rule holds through the orbit and confirm, fail closed."""
    log = []
    gate = DockAcquisitionGate(SQUASH, log.append)
    _run(gate, [[_track()]] * 11)
    assert gate.admit([_track(7), _track(8)], W, H, 2.0) == ([], True)
    assert gate.admit([_track(7), _track(8)], W, H, 2.1) == ([], True)
    assert log[-1] == "DOCK ACQ: 2 dock tracks in view; nothing passes (track 7 acquired)"
    assert gate.admit([_track(7)], W, H, 2.2) == ([_track(7)], True)   # the other one left: the acquired passes


def test_the_acquired_identity_returning_after_a_gap_passes_whatever_its_box():
    """Codex round 1: a brief loss keeps the acquisition; the same id returning small (zoomed out) still passes."""
    gate = DockAcquisitionGate(SQUASH, lambda _m: None)
    _run(gate, [[_track()]] * 11)
    assert gate.admit([], W, H, 2.0) == ([], False)
    assert gate.acquired_id == 7
    small = _track(w=30.0, h=20.0)                                    # 11 x 13 input px: under the floor
    assert gate.admit([small], W, H, 2.1) == ([small], True)


def test_another_dock_never_replaces_the_acquired_identity():
    """Codex round 3: the acquired identity keeps its entitlement while the POI lock may still own it."""
    gate = DockAcquisitionGate(SQUASH, lambda _m: None)
    _run(gate, [[_track()]] * 11)
    for i in range(30):                                                # 9 alone for 3 s while 7 is out of view
        assert gate.admit([_track(9)], W, H, 2.0 + 0.1 * i) == ([], True)
    assert gate.acquired_id == 7
    small = _track(w=30.0, h=20.0)
    assert gate.admit([small], W, H, 5.0) == ([small], True)          # 7 returns small: it passes


def test_an_identity_out_of_view_and_no_longer_locked_is_released():
    """Codex round 4: acquired before any task, then lost until the POI lock's own budget is spent: released, so
    another dock can be acquired; while the lock or the frame still holds it, it is kept."""
    log = []
    gate = DockAcquisitionGate(SQUASH, log.append)
    _run(gate, [[_track()]] * 11)
    gate.release_unowned(7, set())                                   # the lock still holds it
    gate.release_unowned(None, {7})                                  # still in view
    assert gate.acquired_id == 7
    gate.release_unowned(None, set())
    assert gate.acquired_id is None and log[-1] == "DOCK ACQ: track 7 out of view and no longer locked; released"
    assert _run(gate, [[_track(9)]] * 11) == ([_track(9)], True)


def test_the_processor_releases_when_the_poi_lock_gives_up():
    from navpy.core.perception.pipeline.real_tracking_batch import TrackingAssociationProcessor, TrackingModels
    from navpy.core.perception.tracking.poi_lock import PoiLock

    current = [[_track()]]
    identity = Mock()
    identity.update.side_effect = lambda *a, **k: current[0]
    generation = Mock()
    generation.is_current.return_value = True
    recovery = Mock()
    recovery.compute_embeddings.return_value = None
    deep = Mock()
    deep.take_for_tracking.return_value = []
    gate = DockAcquisitionGate(SQUASH, lambda _m: None)
    processor = TrackingAssociationProcessor(TrackingModels(Mock(), identity, PoiLock(max_lost_frames=3)), recovery,
                                             deep, Mock(), generation, True, gate)
    batch = Mock()
    batch.detections = []
    batch.association.frame = object()
    batch.association.frame_width, batch.association.frame_height = W, H
    for i in range(11):
        processor.process(batch, 0.1 * i)
    assert gate.acquired_id == 7
    current[0] = []
    for i in range(3):                                               # within the lock's loss budget: kept
        processor.process(batch, 2.0 + 0.1 * i)
    assert gate.acquired_id == 7
    processor.process(batch, 2.5)                                    # the lock gives up: released
    assert gate.acquired_id is None


def test_stop_tracking_releases_the_acquired_identity():
    from navpy.core.perception.pipeline.real_detector_controls import DetectorTrackingControl

    gate = DockAcquisitionGate(SQUASH, lambda _m: None)
    _run(gate, [[_track()]] * 11)
    poi_lock = Mock()
    DetectorTrackingControl(None, poi_lock, gate).stop_tracking()
    poi_lock.reset.assert_called_once_with()
    assert gate.acquired_id is None
    assert _run(gate, [[_track(9)]] * 11) == ([_track(9)], True)       # now another dock can be acquired


def test_a_brief_other_dock_does_not_erase_the_acquired_identity():
    """Codex round 2: 7 out of view, 9 seen briefly, then 7 returns small (zoomed out): 7 still passes."""
    gate = DockAcquisitionGate(SQUASH, lambda _m: None)
    _run(gate, [[_track()]] * 11)
    assert gate.admit([_track(9)], W, H, 2.0) == ([], True)
    assert gate.admit([], W, H, 2.1) == ([], False)
    small = _track(w=30.0, h=20.0)
    assert gate.admit([small], W, H, 2.2) == ([small], True)
    assert gate.acquired_id == 7
    assert gate.admit([_track(9)], W, H, 2.3) == ([], True)          # 9's dwell starts again from here
    assert gate.admit([_track(9)], W, H, 3.2) == ([], True)


def test_a_pipeline_reset_forgets_the_acquisition_and_its_dwell():
    """Codex round 1: after a reset the tracker reissues id 7 for another object; it must be acquired again."""
    gate = DockAcquisitionGate(SQUASH, lambda _m: None)
    _run(gate, [[_track()]] * 11)
    gate.reset()
    assert gate.acquired_id is None
    assert gate.admit([_track(w=30.0, h=20.0)], W, H, 5.0) == ([], True)


def test_the_detector_reset_resets_the_gate():
    from navpy.core.perception.pipeline.real_detector_controls import DetectorResetController
    from navpy.core.perception.pipeline.real_detector_runtime_state import DetectorRunState

    gate = Mock()
    gate_lock = Mock()
    gate_lock.__enter__ = Mock(return_value=None)
    gate_lock.__exit__ = Mock(return_value=None)
    DetectorResetController(DetectorRunState(), Mock(), Mock(), Mock(), Mock(), Mock(), Mock(), Mock(), gate_lock,
                            gate).refresh()
    gate.reset.assert_called_once_with()


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
