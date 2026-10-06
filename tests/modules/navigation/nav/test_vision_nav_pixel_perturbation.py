"""Pixel-boundary robustness checks for visual-rate final approach.

The old tests injected separate plant and law FPA truth.  These regressions
perturb only the rendered pixel. Navigation never receives the bias value. Pure
PN must reject a constant angular offset because it commands measured LOS rate,
not an assumed absolute pitch calibration.
"""

from __future__ import annotations

import math
from functools import partial

from scripts.vision_static_point_mass_run import run_case_analysis
from scripts.vision_static_point_mass_sensor import (
    FOCAL_Y_PX,
    build_static_detection,
)
from scripts.vision_static_point_mass_types import (
    StaticPointMassCase,
    StaticPointMassMiss,
)


LARGE_PITCH_BIAS_DEG = 4.2
CALIBRATION_ERROR_DEG = 1.3
CALIBRATION_ERROR_SLANT_BOUND_M = 0.25
CROSSWIND_BIAS_SLANT_BOUND_M = 5.59


def _case(
    name: str,
    *,
    wind_dir_from_deg: float = 0.0,
    wind_speed_mps: float = 0.0,
    attitude_time_constant_s: float = 1.6,
) -> StaticPointMassCase:
    return StaticPointMassCase(
        name=name,
        wind_dir_from_deg=wind_dir_from_deg,
        wind_speed_mps=wind_speed_mps,
        start_ned_m=(-580.0, 0.0, -165.0),
        yaw_deg=0.0,
        pitch_deg=-5.0,
        airspeed_mps=28.0,
        roll_limit_deg=60.0,
        command_interval_s=0.05,
        max_t_s=80.0,
        attitude_time_constant_s=attitude_time_constant_s,
    )


def _run_with_pitch_pixel_bias(
    case: StaticPointMassCase,
    bias_deg: float,
) -> StaticPointMassMiss:
    sensor = partial(
        build_static_detection,
        vertical_pixel_bias_px=FOCAL_Y_PX * math.radians(bias_deg),
    )
    return run_case_analysis(case, detection_factory=sensor).miss


def test_constant_vertical_pixel_bias_does_not_create_a_false_pitch_command() -> None:
    miss = _run_with_pitch_pixel_bias(
        _case("vertical-pixel-bias-high-pass"),
        LARGE_PITCH_BIAS_DEG,
    )
    assert miss.passed_target
    assert not miss.timed_out
    assert miss.slant_m < 0.15


def test_error_free_pixel_calibration_hits_without_a_correction_knob() -> None:
    miss = _run_with_pitch_pixel_bias(
        _case("error-free-pixel-calibration"),
        0.0,
    )
    assert miss.passed_target
    assert not miss.timed_out
    assert miss.slant_m < 0.15


def test_small_pixel_pitch_miscalibration_remains_bounded() -> None:
    for bias_deg in (-CALIBRATION_ERROR_DEG, CALIBRATION_ERROR_DEG):
        miss = _run_with_pitch_pixel_bias(
            _case(f"pixel-pitch-bias-{bias_deg:+g}"),
            bias_deg,
        )
        assert miss.passed_target, bias_deg
        assert not miss.timed_out, bias_deg
        assert miss.slant_m < CALIBRATION_ERROR_SLANT_BOUND_M, bias_deg


def test_vertical_pixel_bias_with_crosswind_remains_tightly_bounded() -> None:
    case = _case(
        "vertical-pixel-bias-beam-wind",
        wind_dir_from_deg=90.0,
        wind_speed_mps=8.0,
        attitude_time_constant_s=0.8,
    )
    miss = _run_with_pitch_pixel_bias(case, LARGE_PITCH_BIAS_DEG)
    assert miss == _run_with_pitch_pixel_bias(case, LARGE_PITCH_BIAS_DEG)
    assert miss.passed_target
    assert not miss.timed_out
    assert miss.slant_m < CROSSWIND_BIAS_SLANT_BOUND_M
