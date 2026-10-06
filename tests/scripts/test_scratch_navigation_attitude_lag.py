"""Guards on the attitude sampler, where the wrong answer looks like a number.

Every failure mode here returns a plausible attitude rather than an error: a
nearest sample instead of an interpolated one, a stale sample when the feed
stalls, or an averaged pair across a backward clock step. Each produces a
complete run with an ordinary-looking miss, so each is pinned.
"""

from __future__ import annotations

import random

import pytest

from scripts.scratch_navigation_attitude_lag import (
    AttitudeLagSampler,
    AttitudeSample,
)


def _feed(sampler, count=5, step=0.025, rate_deg_s=40.0):
    """A clean stream at the measured 25 ms attitude interval."""
    for index in range(count):
        t_s = 100.0 + index * step
        sampler.observe(AttitudeSample(t_s, 10.0, rate_deg_s * index * step))
    return 100.0 + (count - 1) * step


def test_zero_lag_returns_the_newest_sample_exactly():
    """The undelayed arm must be bit-identical to no sampler at all.

    Interpolating a zero lag would make the control arm depend on
    floating-point luck, and every comparison in this bench is against it.
    """
    sampler = AttitudeLagSampler(0.0)
    newest_t = _feed(sampler)
    got = sampler.at(newest_t)
    assert got is not None
    assert got.t_s == newest_t
    assert got.roll_deg == 40.0 * 4 * 0.025
    assert sampler.unbracketed == 0


def test_a_lag_lands_between_samples_and_is_interpolated():
    """Not snapped to a neighbour, which is the defect being removed.

    A 25 ms interval cannot express a 10 ms lag by choosing a sample: the two
    candidates are 15 ms apart, and at 40 deg/s that is 0.6 deg of roll -- the
    same size as the effect this bench measures.
    """
    sampler = AttitudeLagSampler(0.010)
    newest_t = _feed(sampler)
    got = sampler.at(newest_t)
    assert got is not None
    # 10 ms before the newest sample, at a constant 40 deg/s.
    assert got.roll_deg == pytest.approx(40.0 * (4 * 0.025 - 0.010), abs=1e-9)
    # Strictly between the two bracketing samples, not equal to either.
    assert 40.0 * 3 * 0.025 < got.roll_deg < 40.0 * 4 * 0.025


def test_a_lag_reaching_before_the_stream_is_refused_not_clamped():
    """The first moments of an scoring interval are where clamping hurts most.

    Substituting the oldest sample would apply the LARGEST pairing error exactly
    at scoring interval entry, and it would do so silently on every run.
    """
    sampler = AttitudeLagSampler(0.200)
    newest_t = _feed(sampler, count=4)
    assert sampler.at(newest_t) is None
    assert sampler.unbracketed == 1


def test_a_stalled_feed_is_refused_rather_than_extrapolated():
    """A lag landing in the stream's future means the feed stopped.

    Asking for `t - lag` when the newest sample is older than that is not a
    rounding problem: it says no attitude has arrived for longer than the lag,
    so the alternative to refusing is flying on an attitude of unknown age.
    """
    sampler = AttitudeLagSampler(0.050)
    _feed(sampler, count=4)
    # The ray's clock has advanced well past the last attitude sample.
    assert sampler.at(100.0 + 3 * 0.025 + 1.0) is None
    assert sampler.unbracketed == 1


def test_two_samples_are_required_before_anything_is_returned():
    sampler = AttitudeLagSampler(0.010)
    sampler.observe(AttitudeSample(100.0, 5.0, 1.0))
    assert sampler.at(100.0) is None
    assert sampler.unbracketed == 1


def test_a_backward_clock_step_is_dropped_not_sorted_in():
    """Interpolating across it would average two different instants.

    The stream is monotonic on the autopilot clock, so a backward stamp means
    something upstream is wrong -- and the blended attitude it would produce is
    indistinguishable from a real one.
    """
    sampler = AttitudeLagSampler(0.0)
    sampler.observe(AttitudeSample(100.0, 10.0, 0.0))
    sampler.observe(AttitudeSample(100.025, 10.0, 1.0))
    sampler.observe(AttitudeSample(100.010, 10.0, 99.0))   # backward
    sampler.observe(AttitudeSample(100.025, 10.0, 99.0))   # repeat
    got = sampler.at(100.025)
    assert got is not None and got.roll_deg == 1.0


def test_interpolation_takes_the_short_way_around_a_wrap():
    """Or a wrap becomes a full-authority reversal at the worst moment.

    -179 to +179 is a 2 degree step. Interpolating the raw difference makes it a
    358 degree sweep, and the command that follows is a hard reversal precisely
    where the aircraft is least tolerant of one.
    """
    sampler = AttitudeLagSampler(0.0125)
    sampler.observe(AttitudeSample(100.0, 0.0, -179.0))
    sampler.observe(AttitudeSample(100.025, 0.0, 179.0))
    got = sampler.at(100.025)
    assert got is not None
    # Halfway across the SHORT arc from -179 is -180 (equivalently +180).
    assert abs(got.roll_deg) == pytest.approx(180.0, abs=1e-9)


def test_a_negative_lag_is_refused_at_construction():
    with pytest.raises(ValueError, match="not reported yet"):
        AttitudeLagSampler(-0.010)


def test_the_two_timing_knobs_cannot_both_be_set():
    """They are one axis measured two ways.

    A run carrying both produces a number neither knob can claim, and it looks
    like a valid result for whichever one the reader is studying.
    """
    import argparse

    from scripts.scratch_navigation_attitude_lag import check_lag_options

    both = argparse.Namespace(estimate_lag_s=0.05, estimate_delay_poses=2)
    with pytest.raises(SystemExit, match="not both"):
        check_lag_options(both)

    # Either alone is fine, and so is neither.
    check_lag_options(argparse.Namespace(estimate_lag_s=0.05,
                                         estimate_delay_poses=0))
    check_lag_options(argparse.Namespace(estimate_lag_s=0.0,
                                         estimate_delay_poses=3))
    check_lag_options(argparse.Namespace(estimate_lag_s=0.0,
                                         estimate_delay_poses=0))
    with pytest.raises(SystemExit, match="not be negative"):
        check_lag_options(argparse.Namespace(estimate_lag_s=-0.1,
                                            estimate_delay_poses=0))


def test_the_atomic_buffer_returns_a_whole_earlier_pose_or_nothing():
    """The arm that separates delay compensation from residual injection.

    Every angle-lag arm keeps the ray current while the attitude ages, which
    leaves the aircraft's own motion over the lag window inside the perceived
    error. This buffer instead delays the WHOLE observation. If its arm does
    NOT reproduce the angle-lag win, the win was the residual, not the delay.
    """
    from types import SimpleNamespace

    from scripts.scratch_navigation_estimate import AtomicDelayBuffer

    buffer = AtomicDelayBuffer(0.06)
    poses = [SimpleNamespace(t_s=100.0 + 0.025 * i) for i in range(6)]

    # Warm-up: nothing old enough yet -- and this is what makes atomic warm-up
    # identical to the unbracketable-attitude path in the caller.
    assert buffer.push(poses[0]) is None
    assert buffer.push(poses[1]) is None
    assert buffer.push(poses[2]) is None
    # 100.075 - 0.06 = 100.015: only pose[0] (100.0) qualifies.
    assert buffer.push(poses[3]).t_s == poses[0].t_s
    # Newest-not-younger, never interpolated: cutoffs 100.04 then 100.065.
    assert buffer.push(poses[4]).t_s == poses[1].t_s
    assert buffer.push(poses[5]).t_s == poses[2].t_s


def test_a_pose_delay_arm_waits_for_its_full_depth_before_flying():
    """A ramping delay at scoring interval onset is not the treatment asked for.

    The deque starts empty, so its oldest retained pose is the CURRENT one on
    the first call: an N=3 arm would apply 0, then 1, then 2 poses of delay
    while the scoring interval clock started, the POI was placed and scoring
    began -- the same onset contamination already fixed for the continuous-lag
    arm, still present on this path. Depth 1 is the undelayed arm and must not
    be made to wait for anything.
    """
    import argparse
    from collections import deque
    from types import SimpleNamespace

    from scripts.scratch_navigation_attitude_lag import AttitudeLagSampler
    from scripts.scratch_navigation_estimate import de_rotating_attitude

    def pose(index):
        return SimpleNamespace(
            t_s=100.0 + 0.025 * index, pitch_deg=1.0 * index, roll_deg=0.0,
            est_pitch_deg=1.0 * index, est_roll_deg=0.0,
            est_roll_rate_rad_s=0.0, est_pitch_rate_rad_s=0.0,
            est_yaw_rate_rad_s=0.0,
        )

    def run(depth, count):
        options = argparse.Namespace(
            estimate_source="attitude", estimate_delay_poses=depth,
            estimate_lag_s=0.0, estimate_lag_scope="angles",
            estimate_jitter_deg=0.0,
        )
        estimates = deque(maxlen=depth + 1)
        sampler = AttitudeLagSampler(0.0)
        jitter = random.Random(0)
        return [
            de_rotating_attitude(options, pose(i), estimates, sampler, jitter)
            for i in range(count)
        ]

    # Depth 3: the first three poses are refused, and the first ACCEPTED one
    # already carries the full three-pose delay (pose 3 sees pose 0's angle).
    delayed = run(3, 5)
    assert [state is None for state in delayed] == [
        True, True, True, False, False]
    assert delayed[3].pitch_deg == pytest.approx(0.0)
    assert delayed[4].pitch_deg == pytest.approx(1.0)

    # The undelayed arm is never made to wait.
    assert all(state is not None for state in run(0, 3))


def test_atomic_scope_does_not_lag_the_estimates_a_second_time():
    """The buffer already delayed the pose; a second lag would double it."""
    import argparse
    import inspect

    import scripts.scratch_navigation_estimate as estimate

    source = inspect.getsource(estimate.de_rotating_attitude)
    assert 'estimate_lag_scope != "atomic"' in source, (
        "atomic scope must bypass the attitude sampler, or the interval "
        "is applied twice and the arm tests nothing nameable"
    )
