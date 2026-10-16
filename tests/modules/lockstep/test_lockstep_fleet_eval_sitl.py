"""10h acceptance on three real serial-step SITLs (opt-in: ``NAVPY_LOCKSTEP_SITL=1``).

Needs WSL with step 7's serial-step firmware. Each run goes through the CLI
(``eval_gcs_navigation_demo.py --lockstep`` on the certified scenario), so it
writes ``report.json``, and claims its own eval-band slot and WSL root: two
same-seed runs, the second with seeded host delays in the relays, the
coordinator and the participants, give the same journal digest and byte-equal
artifacts (``report.json``, each UAV's lines file and pose-cadence CSVs among
them); a different seed changes the digest. Each run's report passes every
certified lockstep gate (run, assignment and forwarding, approval timing,
workflow ticks, pose cadence, SNAP, and the retained vehicle gates, docking
included) with three approvals.
"""

from __future__ import annotations

import json
import os

import pytest

from scripts import eval_gcs_demo_cli as cli
from scripts.eval_certificate_source_time import SOURCE_TIME_SUBDIR
from scripts.eval_gcs_demo_fleet_gates import navpy_lines_file
from scripts.gcs_demo_artifacts import compare_inventories, without_host_only
from scripts.gcs_demo_comparison import _inventory

pytestmark = pytest.mark.skipif(os.environ.get("NAVPY_LOCKSTEP_SITL") != "1",
                                reason="real serial-step SITL run: set NAVPY_LOCKSTEP_SITL=1")

RUNS = {"plain": (1, 0), "delayed": (1, 7), "seed2": (2, 0)}


@pytest.fixture(scope="module")
def runs(tmp_path_factory):
    base = tmp_path_factory.mktemp("lockstep_fleet_eval")
    statuses = {}
    for name, (seed, delay) in RUNS.items():
        statuses[name] = cli.main(["--lockstep", "--seed", str(seed), "--host-delay-seed", str(delay),
                                   "--run-dir", str(base / name)])
    return {name: (base / name, status) for name, status in statuses.items()}


def _digest(run_dir) -> str:
    return (run_dir / "lockstep_journal_digest.txt").read_text(encoding="utf-8").strip()


def test_same_seed_fleet_runs_repeat_under_host_delays_and_another_seed_differs(runs):
    (plain_dir, _), (delayed_dir, _), (other_dir, _) = (runs[n] for n in RUNS)
    assert _digest(plain_dir) == _digest(delayed_dir) != _digest(other_dir)
    inventory = without_host_only(_inventory(plain_dir))
    assert "report.json" in inventory, sorted(inventory)
    for sys_id in (1, 2, 3):
        assert navpy_lines_file(sys_id) in inventory, sorted(inventory)
        assert any(path.startswith(f"{SOURCE_TIME_SUBDIR}/") and f"_{sys_id}_" in path
                   for path in inventory), sorted(inventory)
    artifacts = compare_inventories(inventory, without_host_only(_inventory(delayed_dir)))
    assert artifacts["status"] == "equal", artifacts


@pytest.mark.parametrize("name", list(RUNS))
def test_each_fleet_run_reports_every_gate_passed(runs, name):
    run_dir, status = runs[name]
    report = json.loads((run_dir / "report.json").read_text(encoding="utf-8"))
    assert (status, report["passed"], report["errors"]) == (0, True, [])
    assert len(report["approvals"]) == 3
