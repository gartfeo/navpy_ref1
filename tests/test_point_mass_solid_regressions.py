"""Structural and cadence regressions for the ideal pixel point-mass tool."""

from __future__ import annotations

import ast
import importlib
from pathlib import Path

import pytest

from scripts import vision_static_point_mass as point_mass


REPO_ROOT = Path(__file__).resolve().parents[1]


def test_point_mass_facade_uses_production_pixel_sensor_exactly() -> None:
    expected_modules = (
        "scripts/vision_static_point_mass_types.py",
        "scripts/vision_static_point_mass_sensor.py",
        "scripts/vision_static_point_mass_plant.py",
        "scripts/vision_static_point_mass_run.py",
        "scripts/vision_static_point_mass_cli.py",
    )
    missing = [
        relative
        for relative in expected_modules
        if not (REPO_ROOT / relative).is_file()
    ]
    assert not missing
    sensor = importlib.import_module("scripts.vision_static_point_mass_sensor")
    types = importlib.import_module("scripts.vision_static_point_mass_types")
    run = importlib.import_module("scripts.vision_static_point_mass_run")
    assert point_mass.StaticPointMassCase is types.StaticPointMassCase
    assert point_mass.StaticPointMassMiss is types.StaticPointMassMiss
    assert point_mass.build_static_detection is sensor.build_static_detection
    assert point_mass.run_case is run.run_case
    assert not hasattr(point_mass.StaticPointMassCase, "plant_fpa_offset_deg")

    tree = ast.parse(
        (REPO_ROOT / "scripts/vision_static_point_mass_sensor.py").read_text(
            encoding="utf-8"
        )
    )
    called = {
        node.func.id
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
    }
    assert "body_ray_to_pixel" in called
    assert "FinalApproachVisionFrame" not in called
    assert "FinalApproachFrameProjector" not in {
        node.id for node in ast.walk(tree) if isinstance(node, ast.Name)
    }
    run_tree = ast.parse(
        (REPO_ROOT / "scripts/vision_static_point_mass_run.py").read_text(
            encoding="utf-8"
        )
    )
    assert "FinalApproachFrameProjector" in {
        node.id for node in ast.walk(run_tree) if isinstance(node, ast.Name)
    }


def test_point_mass_far_flyby_is_a_completed_miss_not_timeout() -> None:
    miss = point_mass.run_case(
        point_mass.StaticPointMassCase(
            name="far-flyby",
            wind_speed_mps=0.0,
            wind_dir_from_deg=0.0,
            start_ned_m=(-500.0, 50.0, -40.0),
            roll_limit_deg=0.0,
            max_t_s=40.0,
        )
    )
    assert miss.slant_m > 40.0
    assert miss.passed_poi
    assert not miss.timed_out


def test_point_mass_directly_astern_receding_never_closes_and_times_out() -> None:
    """The law must never CLAIM this geometry: no hit, no closing, timeout.

    PN commands on LOS RATE. A POI directly astern, on a reciprocal
    bearing, receding radially, presents zero LOS rate by construction --
    there is nothing to null. The law still runs: every cycle it plans a
    zero-increment command that holds the anchor, so commands ARE issued;
    what never happens is a turn back toward the POI. This is not a
    defect to tune away: it is a property of the law's input, pinned here on
    a DETERMINISTIC instrument that reproduces exactly. Acquisition from
    this geometry is the approach layer's job: turn toward the POI
    FIRST, then hand over. The law acquiring 118 deg off-nose is certified
    separately (test_far_off_boresight_ideal_360_acquisition_converges).

    CAVEAT on WHY the rate-only law was kept: the 2026-08-17 bearing-gain
    matrix reported a 4.2x-13.2x wind-accuracy penalty for the bearing term,
    but it flew ONE SITL batch per arm, and that instrument was later
    measured at up to 5.5x batch spread on IDENTICAL code (shared note
    20260817-0030, correction section), so those ratios are UNCONFIRMED.
    What is not in doubt, being deterministic: with the bearing term this
    geometry is acquired, without it there is no signal to act on. If the
    wind comparison is re-run with alternating batches and gain=1 wins,
    THIS test is the one to revisit.

    This previously asserted `t_s > 1.0` -- a demand that the law steer a
    geometry it structurally cannot observe. The pinned contract is
    behavioral, not just the timeout flags: closest approach is the FIRST
    command interval at the full starting range, i.e. the aircraft never
    came closer than where it began -- it never started navigation.
    """
    miss = point_mass.run_case(
        point_mass.StaticPointMassCase(
            name="behind-aircraft-ideal-360",
            wind_speed_mps=0.0,
            wind_dir_from_deg=0.0,
            start_ned_m=(700.0, 0.0, -154.0),
            pitch_deg=0.0,
            attitude_time_constant_s=0.01,
            command_interval_s=0.02,
            max_t_s=120.0,
        )
    )
    assert not miss.passed_poi
    assert miss.timed_out
    assert miss.t_s == 0.02
    assert miss.slant_m > 700.0


def test_point_mass_navigation_cadence_never_scales_source_timestamps() -> None:
    fast = point_mass.run_case_analysis(
        point_mass.StaticPointMassCase(
            name="fast-scheduler",
            wind_speed_mps=0.0,
            wind_dir_from_deg=0.0,
            dt_s=0.02,
            command_interval_s=0.04,
        )
    )
    slow = point_mass.run_case_analysis(
        point_mass.StaticPointMassCase(
            name="slow-scheduler",
            wind_speed_mps=0.0,
            wind_dir_from_deg=0.0,
            dt_s=0.02,
            command_interval_s=0.2,
        )
    )
    assert fast.command_timestamps_s[:4] == pytest.approx((0.0, 0.04, 0.08, 0.12))
    assert slow.command_timestamps_s[:4] == pytest.approx((0.0, 0.2, 0.4, 0.6))
    assert len(fast.command_timestamps_s) > len(slow.command_timestamps_s)
    assert fast.miss.passed_poi and slow.miss.passed_poi
    assert fast.miss.t_s == pytest.approx(slow.miss.t_s)
    assert fast.miss.slant_m < 0.25
    assert slow.miss.slant_m < 0.25


def test_point_mass_modules_and_facade_stay_bounded() -> None:
    paths = [
        REPO_ROOT / "scripts/vision_static_point_mass.py",
        *sorted(REPO_ROOT.glob("scripts/vision_static_point_mass_*.py")),
    ]
    assert len(paths) >= 6
    assert len(paths[0].read_text(encoding="utf-8").splitlines()) <= 100
    assert all(
        len(path.read_text(encoding="utf-8").splitlines()) <= 300
        for path in paths[1:]
    )


def test_point_mass_default_case_factory_resolves_numeric_defaults() -> None:
    cases = point_mass.default_cases([0.0], [0.0])
    assert len(cases) == 1
    assert cases[0].dt_s == pytest.approx(0.02)
    assert cases[0].command_interval_s == pytest.approx(0.1)
    assert cases[0].max_t_s == pytest.approx(90.0)
