"""Camera latency and frame drops: the frame schedule (``deterministic_dev_sim.md`` step 5)."""

from __future__ import annotations

import math
import random
import statistics
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

HW = {"min_latency_ticks": 4, "max_latency_ticks": 5, "tail_latency_ticks": 8, "tail_pct": 0.5,
      "gap_ticks": 2, "long_gap_pct": 3.0, "drop_pct": 2.0}
#: The shipped `hw` and `quiet` camera (`sim_profiles.json`), fitted to the ZR10 report.
SHIPPED = {"min_latency_ticks": 4, "max_latency_ticks": 4, "tail_latency_ticks": 6, "tail_pct": 0.6,
           "gap_ticks": 2, "long_gap_pct": 3.0}


def draws(schedule: CameraFrameSchedule, n: int) -> list[FrameDraw]:
    return [schedule.next_frame() for _ in range(n)]


def publication_intervals_ms(schedule: CameraFrameSchedule, n: int) -> list[float]:
    """The gaps between publications, as the camera loop publishes: in capture order, each at its due tick."""
    captured, latest, published = 0, -1, []
    for draw in draws(schedule, n):
        if not draw.dropped:
            latest = max(latest, captured + draw.latency_ticks)
            published.append(latest)
        captured += draw.gap_ticks
    return [(later - earlier) * TICK_S * 1000.0 for earlier, later in zip(published, published[1:])]


def test_the_camera_holds_one_latency_with_a_rare_tail_and_hits_gap_and_drop_at_their_rates():
    schedule = CameraFrameSchedule(HW, model_stream(1, CAMERA_FRAMES, "t"))
    frames = draws(schedule, 50_000)
    latencies = Counter(f.latency_ticks for f in frames)
    assert schedule.latency_ticks in (4, 5)
    assert set(latencies) <= {schedule.latency_ticks, *range(schedule.latency_ticks + 1, 9)}
    assert 0.003 < 1 - latencies[schedule.latency_ticks] / len(frames) < 0.007  # tail_pct 0.5 %
    assert {f.gap_ticks for f in frames} == {2, 3}
    assert 0.027 < sum(f.gap_ticks == 3 for f in frames) / len(frames) < 0.033  # long_gap_pct 3 %
    assert 0.016 < sum(f.dropped for f in frames) / len(frames) < 0.024  # drop_pct 2 %
    held = Counter(CameraFrameSchedule(HW, model_stream(1, CAMERA_FRAMES, f"c{k}")).latency_ticks for k in range(400))
    assert set(held) == {4, 5} and abs(held[4] - held[5]) < 80  # uniform within [min, max]


def test_the_publication_intervals_match_the_zr10_report():
    # REPORT.md (docs/measurements/zr10-latency-20260830), "Cadence and continuity": median 41.815 ms,
    # p95 46.974 ms, p99 53.724 ms; 32 of 7226 intervals below 20 ms (0.44 %), 23 above 62.5 ms (0.32 %).
    # On 20 ms ticks the median and p95 round to 40 ms and the p99 to 60 ms.
    params = load_profile("quiet").models[CAMERA_FRAMES].params
    intervals: list[float] = []
    for camera in range(20):
        intervals += publication_intervals_ms(
            CameraFrameSchedule(params, model_stream(camera, CAMERA_FRAMES, "zr10")), 5_000)
    ranked = sorted(intervals)
    assert statistics.median(intervals) == pytest.approx(40.0)
    assert ranked[int(0.95 * len(ranked))] == pytest.approx(40.0)
    assert ranked[int(0.99 * len(ranked))] == pytest.approx(60.0)
    # The report counts only 32 and 23 such intervals, so each share is known to about +-35 % (two sigma);
    # the sim must stay within a factor of two of each.
    assert 0.0044 / 2 < sum(i < 20.0 for i in intervals) / len(intervals) < 0.0044 * 2
    assert 0.0032 / 2 < sum(i > 62.5 for i in intervals) / len(intervals) < 0.0032 * 2


def test_the_event_latency_matches_the_zr10_report():
    # REPORT.md, "Result": LED event to first published frame, median 99.841 ms, p99 121.540 ms,
    # max 154.339 ms. An event lands anywhere in the frame interval before the capture that sees it,
    # so the sim's equivalent is the capture-to-publication latency plus a uniform wait over that interval.
    params = load_profile("quiet").models[CAMERA_FRAMES].params
    phase = random.Random(0)
    latencies: list[float] = []
    for camera in range(20):
        captured, latest, previous_gap = 0, -1, 2
        for draw in draws(CameraFrameSchedule(params, model_stream(camera, CAMERA_FRAMES, "zr10")), 5_000):
            latest = max(latest, captured + draw.latency_ticks)
            latencies.append((latest - captured + phase.uniform(0.0, previous_gap)) * TICK_S * 1000.0)
            captured, previous_gap = captured + draw.gap_ticks, draw.gap_ticks
    ranked = sorted(latencies)
    assert abs(statistics.median(latencies) - 99.8) < 5.0
    assert abs(ranked[int(0.99 * len(ranked))] - 121.5) < TICK_S * 1000.0  # within one tick of the report's p99


def test_a_camera_draws_its_latency_first_then_drop_tail_latency_then_gap_per_slot():
    stream = model_stream(3, CAMERA_FRAMES, "participant.2.1.camera.0")
    schedule = CameraFrameSchedule(HW, model_stream(3, CAMERA_FRAMES, "participant.2.1.camera.0"))
    held = 4 + math.floor(stream.random() * 2)
    assert schedule.latency_ticks == held
    for _ in range(2_000):
        dropped = stream.random() * 100.0 < 2.0
        tail = stream.random() * 100.0 < 0.5
        tail_latency = held + 1 + math.floor(stream.random() * (8 - held))
        gap = 2 + (stream.random() * 100.0 < 3.0)
        assert schedule.next_frame() == FrameDraw(dropped, tail_latency if tail else held, gap)


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
    ({"gap_ticks": 0}, "gap_ticks must be an int of at least 1"),
    ({"tail_pct": 101.0}, "tail_pct must be a percentage"),
    ({"long_gap_pct": -1.0}, "long_gap_pct must be a percentage"),
    ({"drop_pct": -0.1}, "drop_pct must be a percentage"),
    ({"drop_pct": math.nan}, "drop_pct must be a percentage"),
    ({"min_latency_ticks": 7}, "min <= max <= tail"),
    ({"tail_latency_ticks": 4}, "min <= max <= tail"),
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
    bad = parse_profile("t", {"models": {CAMERA_FRAMES: {"p_on": 1.0, "params": {**HW, "tail_latency_ticks": 3}}}},
                        catalogue)
    with pytest.raises(ValueError, match="min <= max <= tail"):
        participant_models(realize(bad, 5))


def test_the_profiles_give_the_designed_camera_ranges():
    hw = load_profile("hw").models[CAMERA_FRAMES]
    assert hw.p_on == 0.5
    assert {k: v for k, v in hw.params.items() if k != "drop_pct"} == SHIPPED
    assert (hw.params["drop_pct"].lo, hw.params["drop_pct"].hi) == (0.0, 2.0)
    quiet = load_profile("quiet").models
    # quiet is calibrated against quiet wall runs: hw's camera with no drops and no line-rate limit.
    assert quiet[CAMERA_FRAMES].p_on == 1.0 and dict(quiet[CAMERA_FRAMES].params) == {**SHIPPED, "drop_pct": 0.0}
    assert quiet["baud_limit"].p_on == 0.0
    stress = load_profile("stress").models[CAMERA_FRAMES]
    assert stress.p_on == 1.0 and stress.params["tail_latency_ticks"] == 12
    assert CAMERA_FRAMES not in load_profile("none").models
    for seed in range(1, 41):  # every draw of every profile is a valid schedule
        for name in ("quiet", "hw", "stress"):
            participant_models(realize(load_profile(name), seed))
