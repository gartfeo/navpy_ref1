"""Conservation checks for the visual-rate filter and pitch integrator together."""

import math
from dataclasses import replace

import pytest

from navpy.modules.navigation.nav.vision_nav.frame import FinalApproachVisionFrame
from navpy.modules.navigation.nav.vision_nav.law import (
    FixedFinalApproachLawConfigProvider,
    FinalApproachLawConfig,
    VisionNavLaw,
    VERTICAL_PN_NAVIGATION_CONSTANT,
    VERTICAL_RATE_FILTER_TAU_S,
)


def frame(timestamp, elevation_deg):
    angle = math.radians(elevation_deg)
    return FinalApproachVisionFrame(
        "cam", 0, 7, 9, timestamp, 1., 0., 0.,
        math.cos(angle), 0., math.sin(angle), 0., 25., 0., 0.,
    )


def drive(points):
    law = VisionNavLaw(FixedFinalApproachLawConfigProvider(
        FinalApproachLawConfig(-55., 25., 45., .5, None)
    ))
    law.seed(frame(0., 0.))
    plans = {}
    for timestamp, elevation in points:
        plan = law.plan(frame(timestamp, elevation))
        assert plan.origin.reason == "normal"
        assert plan.within_limits
        law.commit(plan)
        plans[timestamp] = plan
    return plans


@pytest.mark.parametrize("dt", [.001, .02, .025, .04, .1])
def test_constant_elevation_slope_matches_continuous_filter_integral(dt):
    # A linear elevation ramp has a constant derivative r. Starting from f=0,
    # integral(f,0..dt) = r*(dt-tau*(1-exp(-dt/tau))). This tests the combined
    # filter/integrator against its continuous model, not an endpoint rectangle.
    slope_deg_s = 2.
    tau = VERTICAL_RATE_FILTER_TAU_S
    plan = drive([(dt, slope_deg_s * dt)])[dt]
    area_deg = slope_deg_s * (dt + tau * math.expm1(-dt / tau))
    assert plan.command.cmd_pitch_deg == pytest.approx(
        -VERTICAL_PN_NAVIGATION_CONSTANT * area_deg, abs=1e-12
    )


def test_zero_net_elevation_excursion_leaves_no_pitch_offset_after_settling():
    # The camera's ordinary 20/40 ms intervals must not turn a closed visual
    # excursion into a permanent command offset. Five seconds is 50 filter
    # time constants, so the remaining physical filter transient is negligible.
    points = [(.02, .1), (.06, 0.)]
    points += [(.06 + i * .02, 0.) for i in range(1, 251)]
    final = drive(points)[points[-1][0]]
    assert abs(final.rate.next_state.filtered_rate_rad_s) < 1e-20
    assert final.command.cmd_pitch_deg == pytest.approx(0., abs=1e-12)


def test_subdividing_the_same_linear_elevation_segments_preserves_commands():
    coarse = drive([(.02, .1), (.06, 0.), (.10, 0.), (.14, -.1), (.18, 0.)])
    fine = drive([
        (.01, .05), (.02, .1), (.04, .05), (.06, 0.),
        (.08, 0.), (.10, 0.), (.12, -.05), (.14, -.1),
        (.16, -.05), (.18, 0.),
    ])
    for timestamp, plan in coarse.items():
        other = fine[timestamp]
        assert other.rate.next_state.filtered_rate_rad_s == pytest.approx(
            plan.rate.next_state.filtered_rate_rad_s, abs=1e-12
        )
        assert other.command.cmd_pitch_deg == pytest.approx(
            plan.command.cmd_pitch_deg, abs=1e-12
        )


def test_integral_conservation_on_irregular_intervals():
    points = [(.02, .1), (.06, -.03), (.08, .05), (.10, .02), (.14, 0.)]
    plans = drive(points)
    tau = VERTICAL_RATE_FILTER_TAU_S
    for timestamp, elevation in points:
        plan = plans[timestamp]
        # Integrating tau*f' + f = elevation' gives this identity independent
        # of interval partition, with zero initial elevation/filter/command.
        area_deg = elevation - tau * math.degrees(
            plan.rate.next_state.filtered_rate_rad_s
        )
        assert plan.command.cmd_pitch_deg == pytest.approx(
            -VERTICAL_PN_NAVIGATION_CONSTANT * area_deg, abs=1e-12
        )


def test_equal_interval_ramp_has_the_declared_continuous_time_constant():
    # An endpoint rectangle has a different transient even at constant cadence.
    # This pins the intended continuous filter, not an airborne accuracy claim.
    dt, slope = .02, 2.
    points = [(i * dt, slope * i * dt) for i in range(1, 26)]
    for timestamp, plan in drive(points).items():
        tau = VERTICAL_RATE_FILTER_TAU_S
        area_deg = slope * (timestamp + tau * math.expm1(-timestamp / tau))
        assert plan.command.cmd_pitch_deg == pytest.approx(
            -VERTICAL_PN_NAVIGATION_CONSTANT * area_deg, abs=1e-12
        )


@pytest.mark.parametrize("uncommitted", [False, True])
def test_closed_excursion_preserves_area_past_a_held_or_uncommitted_frame(uncommitted):
    law = VisionNavLaw(FixedFinalApproachLawConfigProvider(
        FinalApproachLawConfig(-55., 25., 45., .5, None)
    ))
    law.seed(frame(0., 0.))
    first = law.plan(frame(.02, .1))
    law.commit(first)
    if uncommitted:
        # A preview that was not issued cannot advance the filter or anchor.
        law.preview(frame(.04, .3))
    else:
        held = law.plan(frame(.01, -.2))
        assert held.origin.reason == "dt_regressed"
        assert held.command.cmd_pitch_deg == first.command.cmd_pitch_deg
        law.commit(held)
    for timestamp in [.06] + [.06 + i * .02 for i in range(1, 251)]:
        plan = law.plan(frame(timestamp, 0.))
        assert plan.origin.reason == "normal" and plan.within_limits
        law.commit(plan)
    assert plan.command.cmd_pitch_deg == pytest.approx(0., abs=1e-12)


def test_outlier_reseed_starts_a_new_area_identity():
    law = VisionNavLaw(FixedFinalApproachLawConfigProvider(
        FinalApproachLawConfig(-55., 25., 45., .5, None)
    ))
    law.seed(frame(0., 0.))
    first = law.plan(frame(.02, .1))
    law.commit(first)
    assert first.rate.next_state.filtered_rate_rad_s != 0.
    held = law.plan(frame(.04, 40.))
    assert held.origin.reason == "outlier" and held.reseed
    law.commit(held)
    # Reseeding intentionally discards the old filter tail. Conservation only
    # applies within uninterrupted, unclamped intervals after this new seed.
    plan = law.plan(frame(.06, 40.1))
    assert plan.origin.reason == "normal" and plan.within_limits
    area_deg = .1 - VERTICAL_RATE_FILTER_TAU_S * math.degrees(
        plan.rate.next_state.filtered_rate_rad_s
    )
    assert plan.command.cmd_pitch_deg == pytest.approx(
        held.command.cmd_pitch_deg - VERTICAL_PN_NAVIGATION_CONSTANT * area_deg,
        abs=1e-12,
    )


def test_slow_first_frame_retains_the_remaining_filter_tail():
    law = VisionNavLaw(FixedFinalApproachLawConfigProvider(
        FinalApproachLawConfig(-55., 25., 45., .5, None)
    ))
    # Keep sufficient ceiling clearance to observe the full filter area. The
    # single-frame increment bound is not a bound on the settled response.
    law.seed(replace(frame(0., 4.6), aircraft_pitch_deg=-10.))
    first = law.plan(frame(.14, 3.5))
    assert 0. < first.command.cmd_pitch_deg + 10. < 2.612
    assert first.rate.next_state.filtered_rate_rad_s < 0.
    law.commit(first)
    for i in range(1, 251):
        plan = law.plan(frame(.14 + i * .02, 3.5))
        assert plan.within_limits
        law.commit(plan)
    assert plan.command.cmd_pitch_deg + 10. == pytest.approx(
        VERTICAL_PN_NAVIGATION_CONSTANT * 1.1, abs=1e-12
    )
    assert plan.command.cmd_pitch_deg + 10. > 2.612
