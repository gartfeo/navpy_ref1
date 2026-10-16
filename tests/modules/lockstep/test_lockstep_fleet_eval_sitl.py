"""10h acceptance on three real serial-step SITLs (opt-in: ``NAVPY_LOCKSTEP_SITL=1``).

Needs WSL with step 7's serial-step firmware. Each run claims its own
eval-band slot and WSL root (``eval_gcs_navigation_demo.py --lockstep`` on the
certified scenario): two same-seed runs, the second with seeded host delays in
the relays, the coordinator and the participants, give the same journal digest
and byte-equal artifacts (each UAV's lines file and pose-cadence CSVs among
them); a different seed changes the digest. Each run ends ``ok`` and passes
every certified lockstep gate (``analyze_run``: run, assignment and
forwarding, approval timing, workflow ticks, pose cadence, SNAP, and the
retained vehicle gates, docking included).
"""

from __future__ import annotations

import os
from pathlib import Path

import pytest

from scripts.eval_certificate_source_time import SOURCE_TIME_SUBDIR
from scripts.eval_gcs_demo_analysis import analyze_run
from scripts.eval_gcs_demo_fleet_gates import navpy_lines_file
from scripts.eval_gcs_demo_lockstep import LockstepOptions, execute_lockstep_run
from scripts.eval_gcs_demo_scenario import CERTIFIED_SCENARIO_PATH, load_scenario
from scripts.gcs_demo_artifacts import compare_inventories, without_host_only
from scripts.gcs_demo_comparison import _inventory

pytestmark = pytest.mark.skipif(os.environ.get("NAVPY_LOCKSTEP_SITL") != "1",
                                reason="real serial-step SITL run: set NAVPY_LOCKSTEP_SITL=1")

CERTIFIED = load_scenario(CERTIFIED_SCENARIO_PATH)
RUNS = {"plain": (1, 0), "delayed": (1, 7), "seed2": (2, 0)}


@pytest.fixture(scope="module")
def runs(tmp_path_factory):
    base = tmp_path_factory.mktemp("lockstep_fleet_eval")
    return {name: (base / name, execute_lockstep_run(
                base / name, CERTIFIED, LockstepOptions(seed, 600.0, 15.0, delay)))
            for name, (seed, delay) in RUNS.items()}


def test_same_seed_fleet_runs_repeat_under_host_delays_and_another_seed_differs(runs):
    (plain_dir, plain), (delayed_dir, delayed), (_dir, other) = (runs[n] for n in RUNS)
    assert plain.digest == delayed.digest != other.digest
    inventory = without_host_only(_inventory(plain_dir))
    for sys_id in (1, 2, 3):
        assert navpy_lines_file(sys_id) in inventory, sorted(inventory)
        assert any(path.startswith(f"{SOURCE_TIME_SUBDIR}/") and f"_{sys_id}_" in path
                   for path in inventory), sorted(inventory)
    artifacts = compare_inventories(inventory, without_host_only(_inventory(delayed_dir)))
    assert artifacts["status"] == "equal", artifacts


@pytest.mark.parametrize("name", list(RUNS))
def test_each_fleet_run_ends_ok_and_passes_every_gate(runs, name):
    run_dir, run = runs[name]
    assert (run.outcome, run.operator_error) == ("ok", None)
    report = analyze_run(run_dir, scenario=CERTIFIED, approvals=run.approvals)
    assert report.passed, report.errors
    assert len(run.approvals) == 3
