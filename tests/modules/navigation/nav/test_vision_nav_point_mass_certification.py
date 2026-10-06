"""Ideal spherical-pixel final approach point-mass certification."""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from scripts.vision_static_point_mass import (
    StaticPointMassCase,
    run_case,
)


RUN_SCRIPT = Path("scripts/vision_static_point_mass_run.py").resolve()
BEAM_WIND_SLANT_BOUND_M = 3.36
QUARTERING_WIND_SLANT_BOUND_M = 6.90


def _case(name: str, **overrides: object) -> StaticPointMassCase:
    values: dict[str, object] = {
        "name": name,
        "wind_dir_from_deg": 0.0,
        "wind_speed_mps": 0.0,
        "airspeed_mps": 28.0,
        "command_interval_s": 0.02,
        "max_t_s": 120.0,
    }
    values.update(overrides)
    return StaticPointMassCase(**values)


def test_zero_wind_ideal_vision_hits_strictly() -> None:
    miss = run_case(
        StaticPointMassCase(
            name="zero-wind",
            wind_speed_mps=0.0,
            wind_dir_from_deg=0.0,
        )
    )
    assert miss.passed_poi
    assert not miss.timed_out
    assert miss.slant_m < 0.25


def test_visual_pn_rejects_unmodeled_flight_path_pitch_offset() -> None:
    miss = run_case(
        _case(
            "unmodeled-flight-path-pitch-offset",
            start_ned_m=(-580.0, 0.0, -165.0),
            pitch_deg=-5.0,
            attitude_time_constant_s=1.6,
            command_interval_s=0.05,
            flight_path_pitch_offset_deg=4.2,
            max_t_s=80.0,
        )
    )

    assert miss.passed_poi
    assert not miss.timed_out
    assert miss.slant_m < 1.0


def test_crosswind_challenge_is_deterministic_and_bounded() -> None:
    case = StaticPointMassCase(
        name="crosswind",
        wind_speed_mps=8.0,
        wind_dir_from_deg=120.0,
        max_t_s=140.0,
    )
    first = run_case(case)
    second = run_case(case)
    assert first == second
    assert first.passed_poi
    assert first.slant_m < 10.0


def test_certification_script_has_no_near_closest_approach_command_hold() -> None:
    source = RUN_SCRIPT.read_text(encoding="utf-8")
    assert "norm(position_ned_m)) >=" not in source
    assert "law_fpa_offset" not in source


def test_navigation_only_variants_keep_strict_component_miss() -> None:
    cases = (
        _case("collision-triangle", start_ned_m=(-700.0, 0.0, -154.0), pitch_deg=-12.4),
        _case("initial-pitch-shallow", start_ned_m=(-700.0, 0.0, -154.0), pitch_deg=-5.0),
        _case("initial-pitch-steep", start_ned_m=(-700.0, 0.0, -154.0), pitch_deg=-25.0),
        _case("east-cross-track", start_ned_m=(-700.0, 56.0, -154.0), pitch_deg=-12.0),
        _case("west-cross-track", start_ned_m=(-700.0, -56.0, -154.0), pitch_deg=-12.0),
        _case("yaw-left", start_ned_m=(-700.0, 0.0, -154.0), yaw_deg=-15.0, pitch_deg=-12.0),
    )

    for case in cases:
        miss = run_case(case)
        assert miss.passed_poi, case.name
        assert not miss.timed_out, case.name
        # 0.001, not 0.002. The bound was relaxed alongside the removal of the
        # directly-astern case, but nothing here needed it: the worst lateral
        # miss across these cases is 7.8e-06 m, 128x inside the original bound.
        # A tolerance widened without a case that requires it stops being a
        # measurement and becomes room for a future regression to hide in.
        assert miss.lateral_m < 0.001, case.name
        assert miss.longitudinal_m < 0.02, case.name
        assert miss.vertical_m < 0.02, case.name


def test_final_approach_pn_does_not_claim_rear_hemisphere_acquisition() -> None:
    """DIRECTLY ASTERN AND RADIALLY RECEDING, which is narrower than the name.

    This is not a statement about the rear hemisphere. The law acquires at 118
    deg bearing and hits -- see
    `test_far_off_boresight_ideal_360_acquisition_converges` below. What it
    cannot do is the degenerate case pinned here: a POI exactly astern, on a
    reciprocal bearing, receding radially. PN commands on LOS RATE, and that
    geometry presents none, so there is nothing to turn on.

    Recorded because it is a real limit that an acquisition phase upstream has
    to respect, not because rear aspects are out of scope generally.

    Decided 2026-08-17 (owner-approved) in favour of the rate-only law, so
    the limit stands and `tests/test_point_mass_solid_regressions.py` pins
    refusal for this geometry. Acquisition from dead astern belongs to the
    navigation task layer, upstream of this law.

    The wind-accuracy half of that decision is UNCONFIRMED: it came from a
    single SITL batch per arm, an instrument later measured at up to 5.5x
    batch spread on identical code (shared note 20260817-0030, correction).
    The deterministic half -- no LOS rate here, hence no command -- is what
    this test actually pins.
    """
    miss = run_case(_case(
        "behind-aircraft-requires-acquisition",
        start_ned_m=(700.0, 0.0, -154.0),
        pitch_deg=0.0,
        attitude_time_constant_s=0.01,
    ))

    assert not miss.passed_poi
    assert miss.timed_out
    # Behavioral pin, not just flags: closest approach is the first command
    # interval at full starting range -- the aircraft never turned back.
    assert miss.t_s == 0.02
    assert miss.slant_m > 700.0


def test_ideal_pixel_navigation_crosswind_is_symmetric_and_tightly_bounded() -> None:
    base = {
        "start_ned_m": (-700.0, 0.0, -154.0),
        "yaw_deg": 0.0,
        "pitch_deg": -5.0,
        "max_t_s": 80.0,
    }
    beam_right = _case("beam-wind-right", wind_dir_from_deg=90.0, wind_speed_mps=8.0, **base)
    beam_left = _case("beam-wind-left", wind_dir_from_deg=270.0, wind_speed_mps=8.0, **base)
    quartering = _case("quartering-tail-right", wind_dir_from_deg=240.0, wind_speed_mps=8.0, **base)

    right = run_case(beam_right)
    left = run_case(beam_left)
    quarter = run_case(quartering)
    assert right == run_case(beam_right)
    assert left == run_case(beam_left)
    assert quarter == run_case(quartering)
    assert right.passed_poi and left.passed_poi and quarter.passed_poi
    assert not right.timed_out and not left.timed_out and not quarter.timed_out
    assert right.slant_m == pytest.approx(left.slant_m, abs=1e-9)
    assert right.lateral_m == pytest.approx(left.lateral_m, abs=1e-9)
    assert right.slant_m < BEAM_WIND_SLANT_BOUND_M
    assert left.slant_m < BEAM_WIND_SLANT_BOUND_M
    assert quarter.slant_m < QUARTERING_WIND_SLANT_BOUND_M


def test_far_off_boresight_ideal_360_acquisition_converges() -> None:
    horizontal_range_m = 1750.0
    poi_bearing_deg = 118.0
    bearing_rad = math.radians(poi_bearing_deg)
    miss = run_case(
        _case(
            "far-right-rear-ideal-360-acquisition",
            start_ned_m=(
                -horizontal_range_m * math.cos(bearing_rad),
                -horizontal_range_m * math.sin(bearing_rad),
                -141.0,
            ),
            pitch_deg=-1.7,
            roll_deg=42.0,
            attitude_time_constant_s=0.5,
        )
    )
    assert miss.passed_poi
    assert not miss.timed_out
    assert miss.lateral_m < 0.15
    assert miss.slant_m < 0.5


def test_coaltitude_cross_track_horizon_reconstruction_hits_poi() -> None:
    miss = run_case(
        _case(
            "coaltitude-cross-track-horizon",
            start_ned_m=(-700.0, 56.0, 0.0),
            pitch_deg=0.0,
            command_interval_s=0.1,
            max_t_s=40.0,
        )
    )
    assert miss.passed_poi
    assert not miss.timed_out
    assert miss.slant_m < 0.5
