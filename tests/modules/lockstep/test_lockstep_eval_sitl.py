"""9g acceptance on a real serial-step SITL (opt-in: ``NAVPY_LOCKSTEP_SITL=1``).

Needs WSL with step 7's serial-step firmware. Each run claims its own
eval-band slot and WSL root (``eval_gcs_navigation_demo.py --lockstep``): two
same-seed runs, the second with seeded host delays in the relay, the
coordinator and the participants, give the same journal digest and
byte-equal artifacts (the pose-cadence CSVs among them); a different seed
changes the digest. Each run ends ``ok`` and passes the slice's journal gates
that do not depend on the approach docking, and each docks: the workflow
order through ``RESET: PASSED POI`` and the retained vehicle scoring pass
(todo 46: EKF launch gate plus lockstep terrain).
"""

from __future__ import annotations

import os
from pathlib import Path

import pytest

from scripts.eval_certificate_source_time import SOURCE_TIME_SUBDIR
from scripts.eval_gcs_demo_analysis import analyze_run
from scripts.eval_gcs_demo_lockstep import LockstepOptions, execute_lockstep_run
from scripts.eval_gcs_demo_scenario import load_resolved_plan, load_scenario
from scripts.eval_gcs_demo_slice_gates import (
    approval_errors,
    assignment_errors,
    load_slice_evidence,
    pose_cadence_errors,
    run_errors,
    sensor_errors,
    snap_errors,
    workflow_tick_errors,
)
from scripts.gcs_demo_artifacts import compare_inventories, without_host_only
from scripts.gcs_demo_comparison import _inventory

pytestmark = pytest.mark.skipif(os.environ.get("NAVPY_LOCKSTEP_SITL") != "1",
                                reason="real serial-step SITL run: set NAVPY_LOCKSTEP_SITL=1")

ROOT = Path(__file__).resolve().parents[3]
SLICE = load_scenario(ROOT / "scripts" / "eval_lockstep_slice_scenario.json")
RUNS = {"plain": (1, 0), "delayed": (1, 7), "seed2": (2, 0)}


@pytest.fixture(scope="module")
def runs(tmp_path_factory):
    base = tmp_path_factory.mktemp("lockstep_eval")
    return {name: (base / name, execute_lockstep_run(
                base / name, SLICE, LockstepOptions(seed, 600.0, 15.0, delay)))
            for name, (seed, delay) in RUNS.items()}


def test_same_seed_runs_repeat_under_host_delays_and_another_seed_differs(runs):
    (plain_dir, plain), (delayed_dir, delayed), (_dir, other) = (runs[n] for n in RUNS)
    assert plain.digest == delayed.digest != other.digest
    inventory = without_host_only(_inventory(plain_dir))
    assert any(path.startswith(f"{SOURCE_TIME_SUBDIR}/") and path.endswith("_navpy.csv")
               for path in inventory), sorted(inventory)
    artifacts = compare_inventories(inventory, without_host_only(_inventory(delayed_dir)))
    assert artifacts["status"] == "equal", artifacts


@pytest.mark.parametrize("name", list(RUNS))
def test_each_run_ends_ok_and_passes_the_journal_gates(runs, name):
    run_dir, run = runs[name]
    assert (run.outcome, run.operator_error) == ("ok", None)
    plan = load_resolved_plan(run_dir / "demo_mission_plan.json", SLICE)
    evidence = load_slice_evidence(run_dir)
    records, lines = evidence.records, evidence.lines
    poi, owner = plan.pois[0].task_id, plan.owner.sys_id
    assert (run_errors(records) + sensor_errors(lines, poi) + assignment_errors(records, lines, owner)
            + approval_errors(records, lines, run.approvals, owner, poi) + snap_errors(records, lines)
            + pose_cadence_errors(run_dir, owner)) == []


@pytest.mark.parametrize("name", list(RUNS))
def test_the_approach_docks(runs, name):
    """Docking-dependent: the workflow order through RESET: PASSED POI and the
    retained vehicle scoring (RESET, DOCKED before SNAP)."""
    run_dir, run = runs[name]
    lines = load_slice_evidence(run_dir).lines
    assert workflow_tick_errors(lines) == []
    assert analyze_run(run_dir, scenario=SLICE, approvals=run.approvals).passed
