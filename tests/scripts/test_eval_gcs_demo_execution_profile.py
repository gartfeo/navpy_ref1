"""Step 10g: the resolved execution profile and the docking gate split
(``lockstep_fleet.md``, Execution profile and gates)."""

from __future__ import annotations

from pathlib import Path

import pytest

from scripts import eval_gcs_demo_analysis as analysis
from scripts.eval_gcs_demo_constants import SIM_SPEEDUP
from scripts.eval_gcs_demo_evidence import (
    SNAP_WITHOUT_DOCKED,
    parse_final_approach_command_episodes,
    parse_final_approach_episodes,
)
from scripts.eval_gcs_demo_fleet_gates import certified_lockstep_gate_errors
from scripts.eval_gcs_demo_gate_profiles import (
    CERTIFIED_LOCKSTEP_PROFILE,
    CERTIFIED_PROFILE,
    GATE_PROFILES,
    LOCKSTEP,
    OWNER_ONLY_SLICE_PROFILE,
    PROFILE_SIM_SPEEDUP,
    WALL,
    execution_profile,
)
from scripts.eval_gcs_demo_role_gates import ROLE_GATES
from scripts.eval_gcs_demo_slice_gates import JOURNAL_FILE
from tests.scripts.test_eval_gcs_navigation_demo import (
    CERTIFIED,
    _approval_records,
    _write_episode,
    _write_resolved_plan,
)

ROLLS = [12, 11, 9, 7, 5, 3, 1, 0, -1, -2, -2, -1]
#: An oscillating, saturating trace: it fails the roll-quality gates.
BAD_ROLLS = [24.76, 25.46, 20.96, 11.23, -0.50, -10.88, -25.65, -42.95, -45.0, -45.0,
             -32.99, 22.79, 45.0, 45.0, 45.0, 45.0, 30.10, -30.71, -25.63, 3.07]


def _three_uav_run(log_dir: Path) -> None:
    for sys_id, role in ((4, "owner"), (5, "peer"), (6, "peer")):
        _write_episode(log_dir, sys_id, ROLLS, snap_m=0.4, role=role)
    _write_resolved_plan(log_dir)


def _drop_docked(log_dir: Path, sys_id: int) -> None:
    debug = log_dir / f"uav_{sys_id}_navigation_debug.csv"
    debug.write_text("".join(line for line in debug.read_text(encoding="utf-8").splitlines(True)
                             if "EVENT:DOCKED" not in line), encoding="utf-8")


def _mark_terminal_miss(log_dir: Path, sys_id: int, *, late_s: float = 0.0) -> None:
    """The runtime's miss format: the last (unissued) command row has
    missed=True; ``late_s`` delays it after the previous row."""
    debug = log_dir / f"uav_{sys_id}_navigation_debug.csv"
    rows = debug.read_text(encoding="utf-8").splitlines(True)
    last = max(i for i, row in enumerate(rows) if "EVENT:FINAL_APPROACH_CMD" in row)
    assert "issued=False" in rows[last] and "missed=False" in rows[last], rows[last]
    row = rows[last].replace("missed=False", "missed=True")
    if late_s:
        stamp = row.split(",", 1)[0]
        hours, minutes, seconds = stamp.split(":")
        moved = float(seconds) + late_s
        row = f"{hours}:{minutes}:{moved:06.3f}" + row[len(stamp):]
    rows[last] = row
    debug.write_text("".join(rows), encoding="utf-8")


@pytest.mark.parametrize(("profile", "mode", "resolved"), [
    (CERTIFIED_PROFILE, WALL, CERTIFIED_PROFILE),
    (CERTIFIED_PROFILE, LOCKSTEP, CERTIFIED_LOCKSTEP_PROFILE),
    (OWNER_ONLY_SLICE_PROFILE, WALL, OWNER_ONLY_SLICE_PROFILE),
    (OWNER_ONLY_SLICE_PROFILE, LOCKSTEP, OWNER_ONLY_SLICE_PROFILE),
])
def test_only_the_certified_lockstep_pair_resolves_to_a_new_profile(profile, mode, resolved):
    assert execution_profile(profile, mode) == resolved


@pytest.mark.parametrize(("profile", "mode"), [
    (CERTIFIED_LOCKSTEP_PROFILE, LOCKSTEP), ("bogus", WALL), (CERTIFIED_PROFILE, "bogus")])
def test_an_unknown_pair_has_no_execution_profile(profile, mode):
    with pytest.raises(ValueError, match="no execution profile"):
        execution_profile(profile, mode)


def test_certified_lockstep_is_not_a_scenario_profile_and_keeps_the_certified_roles():
    assert CERTIFIED_LOCKSTEP_PROFILE not in GATE_PROFILES
    assert PROFILE_SIM_SPEEDUP[CERTIFIED_LOCKSTEP_PROFILE] == 1.0
    assert PROFILE_SIM_SPEEDUP[CERTIFIED_PROFILE] == SIM_SPEEDUP
    assert ROLE_GATES[CERTIFIED_LOCKSTEP_PROFILE] is ROLE_GATES[CERTIFIED_PROFILE]
    assert (analysis._ASSIGNMENT_EVIDENCE[CERTIFIED_LOCKSTEP_PROFILE]
            is analysis.global_assignment_evidence)


def test_a_wall_certified_run_keeps_its_profile_and_speedup(tmp_path):
    _three_uav_run(tmp_path)
    report = analysis.analyze_run(tmp_path, scenario=CERTIFIED, sys_ids=(4, 5, 6),
                                  approvals=_approval_records())
    assert report.passed is True, (report.errors, [v.errors for v in report.vehicles])
    assert report.sim_speedup == SIM_SPEEDUP


def test_a_certified_lockstep_run_runs_its_journal_gates(tmp_path):
    """10g-3: the (certified_lockstep, lockstep) entry exists; a run without
    the fleet's sysids and evidence fails its gates, it is not refused."""
    assert analysis._RUN_GATES[(CERTIFIED_LOCKSTEP_PROFILE, LOCKSTEP)] is certified_lockstep_gate_errors
    _three_uav_run(tmp_path)
    (tmp_path / JOURNAL_FILE).write_text("", encoding="utf-8")
    assert analysis.execution_mode(tmp_path) == LOCKSTEP
    report = analysis.analyze_run(tmp_path, scenario=CERTIFIED, sys_ids=(4, 5, 6),
                                  approvals=_approval_records())
    assert report.passed is False
    assert "certified lockstep: sysids [4, 5, 6] are not the fleet's 1..3" in report.errors
    with pytest.raises(ValueError, match="'certified_lockstep' has no navigation_speedup override"):
        analysis.analyze_run(tmp_path, scenario=CERTIFIED, sys_ids=(4, 5, 6),
                             approvals=_approval_records(), navigation_speedup=10.0)


def test_a_certified_lockstep_run_analyzes_its_vehicles_on_the_resolved_profile(
        tmp_path, monkeypatch):
    """With run gates supplied, the vehicles get certified_lockstep and speedup 1."""
    _three_uav_run(tmp_path)
    (tmp_path / JOURNAL_FILE).write_text("", encoding="utf-8")
    monkeypatch.setitem(analysis._RUN_GATES, (CERTIFIED_LOCKSTEP_PROFILE, LOCKSTEP),
                        lambda *_args: [])
    seen = []
    real = analysis.analyze_vehicle

    def spy(*args, **kwargs):
        seen.append((kwargs["gate_profile"], kwargs["launch_speedup"]))
        return real(*args, **kwargs)

    monkeypatch.setattr(analysis, "analyze_vehicle", spy)
    report = analysis.analyze_run(tmp_path, scenario=CERTIFIED, sys_ids=(4, 5, 6),
                                  approvals=_approval_records())
    assert report.sim_speedup == 1.0
    assert seen == [(CERTIFIED_LOCKSTEP_PROFILE, 1.0)] * 3


def test_the_parser_flags_an_undocked_episode_and_the_strict_parser_still_rejects_it(tmp_path):
    _write_episode(tmp_path, 1, ROLLS, snap_m=0.4)
    debug = tmp_path / "uav_1_navigation_debug.csv"
    [docked] = parse_final_approach_episodes(debug, vehicle_id=1)
    assert docked.docked is True and docked.commands
    _drop_docked(tmp_path, 1)
    [undocked] = parse_final_approach_episodes(debug, vehicle_id=1)
    assert undocked.docked is False and undocked.commands == docked.commands
    with pytest.raises(ValueError, match=SNAP_WITHOUT_DOCKED):
        parse_final_approach_command_episodes(debug, vehicle_id=1)


def test_a_missed_dock_is_its_own_gate_and_hides_no_other_check(tmp_path):
    """Before the split, a missing DOCKED raised in the parser and the vehicle
    report stopped there: its command and SNAP checks never ran."""
    # 1x observations where the wall profile expects 10x: a cadence failure.
    _write_episode(tmp_path, 1, BAD_ROLLS, snap_m=float("nan"), wall_step_s=0.1)
    _drop_docked(tmp_path, 1)
    _mark_terminal_miss(tmp_path, 1)
    report = analysis.analyze_vehicle(tmp_path, 1, "owner", approval_count=1)
    assert report.passed is False
    assert SNAP_WITHOUT_DOCKED in report.errors
    assert not any("invalid final-approach evidence" in e for e in report.errors), report.errors
    assert any("roll-command step" in e for e in report.errors), report.errors
    assert report.metrics.achieved_roll_reversals is not None  # the achieved-roll gate ran
    assert any("truth SNAP distance is non-finite" in e for e in report.errors), report.errors
    assert any("observation speedup" in e for e in report.errors), report.errors
    assert report.metrics.sample_count == len(BAD_ROLLS)


def test_an_undocked_episode_still_fails_the_achieved_roll_gate(tmp_path):
    """Fix D's gate: a weaving aircraft fails it whether or not it docked."""
    weave = [20.0 if i % 2 else -20.0 for i in range(len(ROLLS))]
    _write_episode(tmp_path, 1, ROLLS, snap_m=0.4, achieved_rolls=weave)
    _drop_docked(tmp_path, 1)
    _mark_terminal_miss(tmp_path, 1)
    report = analysis.analyze_vehicle(tmp_path, 1, "owner", approval_count=1)
    assert SNAP_WITHOUT_DOCKED in report.errors
    assert report.metrics.achieved_roll_reversals == len(ROLLS) - 1
    assert any("significant achieved-roll reversals" in e for e in report.errors), report.errors


def test_a_late_terminal_miss_row_keeps_its_cadence_gap(tmp_path):
    """The miss row stays in the cadence: a gap before it is still a failure."""
    _write_episode(tmp_path, 1, ROLLS, snap_m=0.4)
    _drop_docked(tmp_path, 1)
    _mark_terminal_miss(tmp_path, 1, late_s=0.6)
    report = analysis.analyze_vehicle(tmp_path, 1, "owner", approval_count=1)
    assert SNAP_WITHOUT_DOCKED in report.errors
    assert any("max observation gap" in e for e in report.errors), report.errors
    assert report.metrics.max_wall_gap_s > 0.5


def test_a_missed_dock_fails_only_its_vehicle_not_the_poi_binding(tmp_path):
    _three_uav_run(tmp_path)
    _drop_docked(tmp_path, 5)
    report = analysis.analyze_run(tmp_path, scenario=CERTIFIED, sys_ids=(4, 5, 6),
                                  approvals=_approval_records())
    assert report.passed is False
    assert report.errors == []
    assert [v.errors for v in report.vehicles] == [[], [SNAP_WITHOUT_DOCKED], []]
