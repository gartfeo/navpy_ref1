"""The achieved-roll reversal gate, on synthetic logs through analyze_vehicle.

Every trace here is synthetic (written by `_write_episode`), not a flight log.
Each episode docks (valid DOCKED) and passes every other vehicle gate, so the
achieved-roll gate is the only one that can fail it.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Callable

import pytest

from scripts import eval_gcs_navigation_demo as evaluator
from tests.scripts.test_eval_gcs_navigation_demo import _write_episode

# Synthetic: a +-20 deg S-turn, 5 deg steps, never near the 45 deg limit.
_RISE = [0.0, 5.0, 10.0, 15.0, 20.0, 15.0, 10.0, 5.0, 0.0]
_FALL = [-5.0, -10.0, -15.0, -20.0, -15.0, -10.0, -5.0]
ONE_CORRECTION = _RISE + _FALL + [0.0, 0.0]
CYCLE = _RISE + _FALL + [0.0, 5.0, 10.0, 15.0, 20.0, 15.0, 10.0]


def _debug(log_dir: Path) -> Path:
    return log_dir / "uav_1_navigation_debug.csv"


def _response_rows(log_dir: Path) -> list[str]:
    return [
        line
        for line in _debug(log_dir).read_text(encoding="utf-8").splitlines()
        if ",EVENT:FINAL_APPROACH_RESPONSE_STATE," in line
    ]


def _edit_debug(log_dir: Path, edit: Callable[[list[str]], list[str]]) -> None:
    lines = _debug(log_dir).read_text(encoding="utf-8").splitlines()
    _debug(log_dir).write_text("\n".join(edit(lines)) + "\n", encoding="utf-8")


def _analyze(log_dir: Path):
    return evaluator.analyze_vehicle(log_dir, 1, "owner", approval_count=1)


def test_unsaturated_achieved_roll_cycle_fails_only_this_gate(tmp_path):
    """Synthetic: the aircraft flies a full +-20 deg cycle (two reversals)."""
    _write_episode(tmp_path, 1, CYCLE, snap_m=0.3, achieved_rolls=CYCLE)

    report = _analyze(tmp_path)

    assert report.passed is False
    assert report.errors == ["significant achieved-roll reversals 2 > 1"]
    assert report.metrics.achieved_roll_reversals == 2
    assert report.metrics.saturation_fraction == 0.0
    assert report.metrics.max_roll_step_deg == pytest.approx(5.0)


def test_one_achieved_roll_correction_passes(tmp_path):
    """Synthetic: one overshoot and correction is not an oscillation."""
    _write_episode(
        tmp_path, 1, ONE_CORRECTION, snap_m=0.3, achieved_rolls=ONE_CORRECTION,
    )

    report = _analyze(tmp_path)

    assert report.passed is True, report.errors
    assert report.metrics.achieved_roll_reversals == 1


def test_command_spikes_under_a_steady_achieved_bank_pass(tmp_path):
    """Synthetic: single-sample command flips (LOS-rate spikes) while the
    aircraft holds its bank are a command diagnostic, not a failure."""
    commands = [20.0] * 4 + [-15.0] + [20.0] * 4 + [-15.0] + [20.0] * 4
    _write_episode(
        tmp_path, 1, commands, snap_m=0.3, achieved_rolls=[21.0] * len(commands),
    )

    report = _analyze(tmp_path)

    assert report.passed is True, report.errors
    assert report.metrics.significant_reversals == 4
    assert report.metrics.achieved_roll_reversals == 0


def _drop_second_response(lines: list[str]) -> list[str]:
    second = [i for i, line in enumerate(lines) if "RESPONSE_STATE" in line][1]
    return lines[:second] + lines[second + 1:]


def _duplicate_second_response(lines: list[str]) -> list[str]:
    second = [i for i, line in enumerate(lines) if "RESPONSE_STATE" in line][1]
    return lines[: second + 1] + [lines[second]] + lines[second + 1:]


def _add_unmatched_response(lines: list[str]) -> list[str]:
    second = [i for i, line in enumerate(lines) if "RESPONSE_STATE" in line][1]
    stray = lines[second].replace("obs_ts=1000.100;", "obs_ts=999.000;")
    return lines[: second + 1] + [stray] + lines[second + 1:]


def _repeat_second_roll_field(lines: list[str]) -> list[str]:
    second = [i for i, line in enumerate(lines) if "RESPONSE_STATE" in line][1]
    edited = lines[second].replace(
        "aircraft_roll_deg=", "aircraft_roll_deg=nan;aircraft_roll_deg=", 1,
    )
    assert edited != lines[second]
    return lines[:second] + [edited] + lines[second + 1:]


_SEVEN = ONE_CORRECTION[:7] + [None] + ONE_CORRECTION[8:]
_NAN = ONE_CORRECTION[:7] + [math.nan] + ONE_CORRECTION[8:]


@pytest.mark.parametrize(
    ("achieved", "edit", "reason"),
    [
        (
            _SEVEN,
            None,
            "achieved-roll evidence missing for 1 issued command(s) "
            "(first source=camera obs_ts=1000.7)",
        ),
        (
            None,
            _drop_second_response,
            "achieved-roll evidence missing for 1 issued command(s) "
            "(first source=camera obs_ts=1000.1)",
        ),
        (
            _NAN,
            None,
            "achieved-roll evidence non-finite for 1 issued command(s) "
            "(first source=camera obs_ts=1000.7)",
        ),
        (
            None,
            _duplicate_second_response,
            "achieved-roll evidence duplicated for 1 issued command(s) "
            "(first source=camera obs_ts=1000.1)",
        ),
        (
            None,
            _add_unmatched_response,
            "achieved-roll evidence unmatched: 1 response row(s) with no "
            "issued command (first source=camera obs_ts=999.0)",
        ),
        (
            None,
            _repeat_second_roll_field,
            "achieved-roll evidence repeated-field for 1 issued command(s) "
            "(first source=camera obs_ts=1000.1)",
        ),
    ],
    ids=[
        "missing-field", "missing-row", "non-finite", "duplicate", "unmatched",
        "repeated-field",
    ],
)
def test_invalid_achieved_roll_evidence_fails_closed_by_name(
    tmp_path, achieved, edit, reason,
):
    """Synthetic: bad response evidence fails the gate, never skips a sample."""
    _write_episode(
        tmp_path, 1, ONE_CORRECTION, snap_m=0.3,
        achieved_rolls=ONE_CORRECTION if achieved is None else achieved,
    )
    if edit is not None:
        _edit_debug(tmp_path, edit)
    assert _response_rows(tmp_path)

    report = _analyze(tmp_path)

    assert report.passed is False
    assert report.errors == [reason]
    assert report.metrics.achieved_roll_reversals is None
