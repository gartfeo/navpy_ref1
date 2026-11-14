"""Which frame places the HITL orbit (HITL step 23b, Codex round 2).

With the whole-dock detector the ground ray is taken only from a frame the acquisition floor still bounds (its box
at least ``min_box_px`` scaled to the search step) with a usable pose; the newest such frame of the task places the
orbit, so a late task start or lock never moves the centre's distance past D_max. Without it every frame is taken.
"""
from __future__ import annotations

from dataclasses import replace
from unittest.mock import Mock

import pytest

from navpy.app.perception.rfdetr_profile import orbit_centre_frame
from navpy.core.kernel.attitude import Attitude
from navpy.core.kernel.location import Location
from navpy.core.ports.gimbal_abc import GimbalData
from navpy.core.perception.profile.vision_profile_loader import load_profiles
from navpy.core.skills.confirm.acquisition_sequence import AcquisitionSequence
from navpy.core.skills.geo_approach.orbit_centre_frame import OrbitCentreFrame
from tests.detection_factory import make_detected_poi

TILES = OrbitCentreFrame(min_box_px=24.0, search_fx_px=3273.0, search_fy_px=3263.0, input_scale=lambda w, h: (1, 1))
K10 = [[16971.0, 0.0, 960.0], [0.0, 16812.0, 540.0], [0.0, 0.0, 1.0]]
K2 = [[3273.0, 0.0, 960.0], [0.0, 3263.0, 540.0], [0.0, 0.0, 1.0]]


def _poi(box=(960.0, 540.0, 40.0, 30.0), k=K2, atomic=True, attitude=True, camera=True, gimbal=True, task_id=1,
         ts=1.0):
    return make_detected_poi(
        task_id=task_id, k=k, tracking_bbox_cxcywh=box, pose_is_frame_atomic=atomic, timestamp=ts,
        g_data=GimbalData(att=Attitude(pitch=-15.0, yaw=0.0, roll=0.0), name="zr10") if gimbal else None,
        uas_att=Attitude(2.0, 0.0, 1.0) if attitude else None, frame_width_px=1920.0, frame_height_px=1080.0,
        c_g_loc=Location(40.31, 44.45, 80.0) if camera else None,
    )


def test_a_frame_whose_box_meets_the_floor_at_the_search_step_is_taken():
    assert TILES(_poi(box=(960.0, 540.0, 40.0, 30.0)))           # 40 x 30 at the search step itself
    assert not TILES(_poi(box=(960.0, 540.0, 40.0, 20.0)))       # 20 px down: under 24


def test_a_zoomed_in_frame_is_judged_as_the_search_step_would_see_it():
    """At 10x a 120 px box is 23 px at 2x: farther than D_max, so it may not place the centre."""
    assert not TILES(_poi(box=(960.0, 540.0, 200.0, 120.0), k=K10))
    assert TILES(_poi(box=(960.0, 540.0, 200.0, 130.0), k=K10))  # 25.2 px at 2x
    assert not TILES(_poi(box=(960.0, 540.0, 120.0, 300.0), k=K10))  # 23.1 px across at 2x
    assert TILES(_poi(box=(960.0, 540.0, 130.0, 300.0), k=K10))      # 25.1 px across


def test_the_squash_judges_in_its_own_input_pixels():
    squash = replace(TILES, input_scale=lambda w, h: (704 / w, 704 / h))
    assert not squash(_poi(box=(960.0, 540.0, 60.0, 40.0)))      # 22 x 26 squash px
    assert squash(_poi(box=(960.0, 540.0, 70.0, 40.0)))


@pytest.mark.parametrize("bad", [{"atomic": False}, {"atomic": None}, {"attitude": False}, {"camera": False}, {"gimbal": False},
                                 {"box": None}, {"box": (1.0, 2.0, float("nan"), 30.0)}])
def test_a_frame_without_a_usable_pose_or_box_is_never_taken(bad):
    assert not TILES(_poi(**bad))


def _sequence(centre_frame, start):
    lock = Mock()
    orbit = Mock()
    orbit.commanded = False
    orbit.established.return_value = False
    return AcquisitionSequence(lock, orbit, start, Mock(), lambda *_a: None, centre_frame=centre_frame), lock, orbit


def test_the_newest_accepted_frame_places_the_orbit_and_rejected_ones_never_do():
    start = Mock()
    start.prepare.return_value = None
    sequence, lock, _orbit = _sequence(TILES, start)
    first = _poi(ts=1.0)
    sequence.begin(first)
    lock.locked.return_value = False
    second = _poi(ts=1.1)
    sequence.ready(second)
    far = _poi(box=(960.0, 540.0, 200.0, 120.0), k=K10, ts=1.2)   # the lock comes on a frame beyond D_max
    lock.locked.return_value = True
    sequence.ready(far)
    assert start.prepare.call_args.args[0] is second


def test_a_refused_frame_is_dropped_and_the_next_accepted_one_is_tried():
    start = Mock()
    start.prepare.return_value = None
    sequence, lock, _orbit = _sequence(TILES, start)
    sequence.begin(_poi(ts=1.0))
    lock.locked.return_value = True
    sequence.ready(_poi(atomic=False, ts=1.1))                      # not accepted: the start frame is tried
    assert start.prepare.call_count == 1
    sequence.ready(_poi(atomic=False, ts=1.2))                      # refused, dropped: nothing left to try
    assert start.prepare.call_count == 1
    good = _poi(ts=1.3)
    sequence.ready(good)
    assert start.prepare.call_count == 2 and start.prepare.call_args.args[0] is good


def test_with_no_accepted_frame_the_orbit_waits():
    start = Mock()
    sequence, lock, _orbit = _sequence(TILES, start)
    sequence.begin(_poi(atomic=False))
    lock.locked.return_value = True
    sequence.ready(_poi(atomic=False, ts=1.1))
    start.prepare.assert_not_called()


def test_the_hitl_profile_takes_the_floor_and_the_widest_measured_step():
    profiles, _, _ = load_profiles()
    rule = orbit_centre_frame(profiles["siyi_zr10_hitl_encoded"])
    assert rule.min_box_px == 24.0 and rule.search_fx_px == pytest.approx(3273, abs=1)
    assert rule.input_scale(1920, 1080) == (1.0, 1.0)
    assert orbit_centre_frame(profiles["siyi_zr10"]) is None       # no whole-dock detector: every frame is taken


def test_the_acquisition_composition_wires_the_rule_from_the_profile(monkeypatch):
    from navpy.app.flow import nav_acquisition_composition as composition
    from navpy.core.skills.acquire.approach_strategy import ApproachKind

    captured = []
    monkeypatch.setattr(composition, "AcquisitionSequence", lambda *args: captured.append(args) or Mock())
    profiles, _, _ = load_profiles()
    for name in ("siyi_zr10_hitl_encoded",):
        composition.compose_acquisition(Mock(), Mock(), profiles[name], Mock(), Mock(), Mock(), Mock(), Mock(),
                                        ApproachKind.ORBIT, Mock(), Mock())
    rule = captured[-1][-1]
    assert isinstance(rule, OrbitCentreFrame) and rule.min_box_px == 24.0
    gates_only = {"guidance_gates": profiles["siyi_zr10_hitl_encoded"]["guidance_gates"]}
    composition.compose_acquisition(Mock(), Mock(), gates_only, Mock(), Mock(), Mock(), Mock(), Mock(),
                                    ApproachKind.ORBIT, Mock(), Mock())
    assert captured[-1][-1](Mock()) is True                         # no whole-dock detector: every frame
