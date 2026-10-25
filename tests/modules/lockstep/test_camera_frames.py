"""Camera latency and frame drops: the frame schedule (``deterministic_dev_sim.md`` step 5)."""

from __future__ import annotations

import math
from collections import Counter

import pytest

from navpy.sim.lockstep.camera_frames import (
    CAMERA_FRAMES,
    TICK_S,
    CameraFrameSchedule,
    FrameDraw,
    camera_frame_factory,
)
from navpy.sim.lockstep.participant_budget import participant_models
from navpy.sim.lockstep.sim_profile import CATALOGUE, load_profile, model_stream, parse_profile, realize

HW = {"min_latency_ticks": 4, "max_latency_ticks": 6, "tail_latency_ticks": 8, "tail_pct": 1.0,
      "min_gap_ticks": 2, "max_gap_ticks": 3, "drop_pct": 2.0}


def draws(schedule: CameraFrameSchedule, n: int) -> list[FrameDraw]:
    return [schedule.next_frame() for _ in range(n)]


def test_draws_stay_in_range_reach_every_value_and_hit_tail_and_drop_at_their_rates():
    frames = draws(CameraFrameSchedule(HW, model_stream(1, CAMERA_FRAMES, "t")), 50_000)
    latencies = Counter(f.latency_ticks for f in frames)
    assert set(latencies) == {4, 5, 6, 7, 8}
    tail = (latencies[7] + latencies[8]) / len(frames)
    assert 0.007 < tail < 0.013  # tail_pct 1 %
    assert {f.gap_ticks for f in frames} == {2, 3}
    assert 0.016 < sum(f.dropped for f in frames) / len(frames) < 0.024  # drop_pct 2 %
    for value in (4, 5, 6):
        assert abs(latencies[value] / len(frames) - 0.99 / 3) < 0.01  # uniform within [min, max]


def test_a_slot_draws_drop_tail_latency_then_gap_from_its_stream():
    stream = model_stream(3, CAMERA_FRAMES, "participant.2.1.camera.0")
    schedule = CameraFrameSchedule(HW, model_stream(3, CAMERA_FRAMES, "participant.2.1.camera.0"))
    for _ in range(200):
        dropped = stream.random() * 100.0 < 2.0
        tail = stream.random() * 100.0 < 1.0
        latency = (7 + math.floor(stream.random() * 2)) if tail else 4 + math.floor(stream.random() * 3)
        gap = 2 + math.floor(stream.random() * 2)
        assert schedule.next_frame() == FrameDraw(dropped, latency, gap)


def test_due_and_next_slot_are_whole_ticks_after_the_capture_less_a_half_tick_margin():
    draw = FrameDraw(False, 5, 3)
    assert draw.due_s(10.0) == pytest.approx(10.0 + 5 * TICK_S - TICK_S / 2)
    assert draw.next_slot_s(10.0) == pytest.approx(10.0 + 3 * TICK_S - TICK_S / 2)


def test_each_mount_and_participant_has_its_own_stream():
    models = {CAMERA_FRAMES: HW}
    one = draws(camera_frame_factory(models, 3, 1, 1)(0), 300)
    assert one == draws(camera_frame_factory(models, 3, 1, 1)(0), 300)
    for factory, mount in ((camera_frame_factory(models, 3, 1, 1), 1), (camera_frame_factory(models, 3, 2, 1), 0),
                           (camera_frame_factory(models, 3, 1, 2), 0), (camera_frame_factory(models, 4, 1, 1), 0)):
        assert draws(factory(mount), 300) != one
    expected = draws(CameraFrameSchedule(HW, model_stream(3, CAMERA_FRAMES, "participant.2.1.camera.1")), 300)
    assert draws(camera_frame_factory(models, 3, 2, 1)(1), 300) == expected


def test_the_model_off_builds_no_schedule():
    assert camera_frame_factory({}, 1, 1, 1) is None


@pytest.mark.parametrize("change, message", [
    ({"min_latency_ticks": -1}, "min_latency_ticks must be an int of at least 0"),
    ({"max_latency_ticks": 5.0}, "max_latency_ticks must be an int"),
    ({"tail_latency_ticks": True}, "tail_latency_ticks must be an int"),
    ({"min_gap_ticks": 0}, "min_gap_ticks must be an int of at least 1"),
    ({"tail_pct": 101.0}, "tail_pct must be a percentage"),
    ({"drop_pct": -0.1}, "drop_pct must be a percentage"),
    ({"drop_pct": math.nan}, "drop_pct must be a percentage"),
    ({"min_latency_ticks": 7}, "min <= max <= tail"),
    ({"tail_latency_ticks": 5}, "min <= max <= tail"),
    ({"min_gap_ticks": 4}, "gap ticks need min <= max"),
])
def test_a_bad_camera_entry_is_refused_before_the_run(change, message):
    with pytest.raises(ValueError, match=message):
        camera_frame_factory({CAMERA_FRAMES: {**HW, **change}}, 1, 1, 1)


def test_participant_models_carry_the_camera_draw_and_refuse_a_bad_one():
    catalogue = {CAMERA_FRAMES: CATALOGUE[CAMERA_FRAMES]}
    entry = parse_profile("t", {"models": {CAMERA_FRAMES: {"p_on": 1.0, "params": {**HW, "drop_pct": [0.0, 2.0]}}}},
                          catalogue)
    models = participant_models(realize(entry, 5))
    assert list(models) == [CAMERA_FRAMES] and 0.0 <= models[CAMERA_FRAMES]["drop_pct"] <= 2.0
    bad = parse_profile("t", {"models": {CAMERA_FRAMES: {"p_on": 1.0, "params": {**HW, "min_gap_ticks": 4}}}},
                        catalogue)
    with pytest.raises(ValueError, match="gap ticks"):
        participant_models(realize(bad, 5))


def test_the_profiles_give_the_designed_camera_ranges():
    hw = load_profile("hw").models[CAMERA_FRAMES]
    assert hw.p_on == 0.5
    assert {k: v for k, v in hw.params.items() if k != "drop_pct"} == {k: v for k, v in HW.items() if k != "drop_pct"}
    assert (hw.params["drop_pct"].lo, hw.params["drop_pct"].hi) == (0.0, 2.0)
    quiet = load_profile("quiet").models[CAMERA_FRAMES]
    assert quiet.p_on == 1.0 and quiet.params["min_latency_ticks"] == 4 and quiet.params["tail_latency_ticks"] == 8
    stress = load_profile("stress").models[CAMERA_FRAMES]
    assert stress.p_on == 1.0 and stress.params["tail_latency_ticks"] == 12
    assert CAMERA_FRAMES not in load_profile("none").models
    for seed in range(1, 41):  # every draw of every profile is a valid schedule
        for name in ("quiet", "hw", "stress"):
            participant_models(realize(load_profile(name), seed))
