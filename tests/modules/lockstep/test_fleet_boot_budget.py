"""The fleet manifest's boot budget, set from the 10d recording (``lockstep_fleet.md``, Boot barrier).

``BOOT_BUDGET_US`` = the latest ``helpers_quiet`` latch of the recorded
3-SITL runs (sim time after frame 1) plus the named margin. Every recorded
boot completes within it; a SITL that never quiets faults at the first
frame at or past it.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from navpy.sim.lockstep.coordinator_latches import BootLatch
from navpy.sim.lockstep.coordinator_types import CoordinatorFault
from scripts.eval_lockstep_fleet_auto import fleet_manifest
from scripts.lockstep_wsl_sitl import BOOT_BUDGET_MARGIN_US, BOOT_BUDGET_US, BOOT_LATCH_RECORDED_US
from tests.modules.lockstep.test_grid_check_recording import recorded_frames

RECORDING = json.loads((Path(__file__).resolve().parents[2] / "fixtures" / "lockstep"
                        / "fleet_boot_latches.json").read_text(encoding="utf-8"))["runs"]


def test_the_budget_is_the_recorded_latest_latch_plus_the_margin():
    assert len(RECORDING) >= 4 and {run["delay_seed"] for run in RECORDING} >= {0, 7}
    assert len({run["slot"] for run in RECORDING}) >= 2
    assert BOOT_LATCH_RECORDED_US == max(run["barrier_t_us"] - run["frame1_t_us"] for run in RECORDING)
    assert BOOT_BUDGET_MARGIN_US > 0
    assert BOOT_BUDGET_US == BOOT_LATCH_RECORDED_US + BOOT_BUDGET_MARGIN_US
    assert fleet_manifest(1).boot_budget_us == BOOT_BUDGET_US


@pytest.mark.parametrize("run", RECORDING, ids=lambda run: run["run"].rsplit("/", 2)[-2])
def test_every_recorded_boot_completes_within_the_budget(run):
    times = dict(recorded_frames())
    latch = BootLatch(3, BOOT_BUDGET_US)
    for k in range(1, max(run["latches"]) + 1):
        latch.observe(k, times[k], [k >= tick for tick in run["latches"]])
    assert latch.complete and latch.ticks == run["latches"]


def test_a_sitl_that_never_quiets_faults_at_the_first_frame_past_the_budget():
    times = dict(recorded_frames())
    latch = BootLatch(3, BOOT_BUDGET_US)
    expected = next(k for k in sorted(times) if times[k] - times[1] >= BOOT_BUDGET_US)
    with pytest.raises(CoordinatorFault, match="not quiet") as raised:
        for k in sorted(times):
            latch.observe(k, times[k], [True, False, True])
    assert (raised.value.kind, raised.value.tick, raised.value.context) == (
        "boot_barrier", expected, {"waiting": [2]})


def test_a_slice_whose_sitl_never_quiets_ends_as_a_boot_barrier_fault(tmp_path):
    """Codex 10d round 1 P2-2: the slice's own manifest carries the budget, so a
    serial-step SITL that never latches ``helpers_quiet`` ends the production
    coordinator's run instead of booting forever; the backend, registered
    before the launch, is stopped and the SITL too."""
    from gcs.backend.instance_registry_runtime import pid_alive
    from navpy.sim.lockstep.coordinator_members import Roster
    from navpy.sim.lockstep.coordinator_run import LockstepCoordinator
    from navpy.sim.lockstep.coordinator_spawn import PROCESSES_HOST_FILE, HostSidecar, LocalSpawner
    from navpy.sim.lockstep.operator_baton import OperatorBaton
    from navpy.sim.lockstep.participant_hooks import RequestIds
    from scripts.eval_gcs_demo_lockstep import (
        WALL_BUDGETS,
        LockstepOptions,
        manifest,
        operator_script,
    )
    from scripts.eval_gcs_demo_scenario import load_scenario
    from scripts.lockstep_wsl_sitl import exchange_deadline
    from tests.modules.lockstep.coordinator_doubles import PYTHON_PATH, AllowAll, ScriptedSitl

    scenario = load_scenario(Path(__file__).resolve().parents[3] / "scripts"
                             / "eval_lockstep_slice_scenario.json")
    options = LockstepOptions(seed=1, timeout_s=600.0, confirmation_delay_s=15.0)
    plan = manifest(scenario, options)
    assert plan.boot_budget_us == BOOT_BUDGET_US
    sitl = ScriptedSitl(quiet_from=10**9)  # 20 ms frames, never quiet
    roster = Roster(plan, LocalSpawner(tmp_path / "host", PYTHON_PATH), AllowAll(),
                    HostSidecar(tmp_path / PROCESSES_HOST_FILE), WALL_BUDGETS,
                    deadline=exchange_deadline())
    ids = RequestIds(0)
    operator = OperatorBaton(ids, operator_script(scenario, tmp_path, options))
    coordinator = LockstepCoordinator(plan, (sitl,), operator, roster, ids, tmp_path / "host")
    with pytest.raises(CoordinatorFault, match="not quiet"):
        coordinator.run()
    expected = BOOT_BUDGET_US // 20_000 + 1  # the first frame at or past the budget
    assert coordinator.journal.records[-1] == {"fault": "CoordinatorFault", "kind": "boot_barrier",
                                               "tick": expected, "ordinal": None,
                                               "context": {"waiting": [1]}}
    assert coordinator.journal.records[0]["grid"]["boot_budget_us"] == BOOT_BUDGET_US
    assert sitl.stopped and [m.ordinal for m in roster.members] == [2]
    assert not any(pid_alive(m.child.pid) for m in roster.members)
