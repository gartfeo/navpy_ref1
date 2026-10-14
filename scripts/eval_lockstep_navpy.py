"""Step 9f on a real serial-step SITL: NavPy spawned, booted and stopped through the API.

The 9d coordinator runs one serial-step ArduPlane SITL (step 7's firmware and
relay, in this run's own WSL root) with the backend participant on
``lockstep:serial1`` and, spawned through the backend's launcher port, the
NavPy participant on ``lockstep:serial0``. The scripted API actor
(``scripts.lockstep_slice_actor.companion_script``) makes 9e's setup calls,
POST start-all, waits for readiness through the status route, then stop-all
(``STOP_ACKED``). The run then drains the backend and parks the SITL.

``--runs 2`` (the default) claims two eval-band slots and runs once in each,
the second with seeded host delays (relay and coordinator); the journal
digests, the journals and NavPy's log lines must match. Per run directory:
``lockstep_journal.json`` and ``lockstep_digest.txt``; host facts (slot,
WSL root, PIDs, exit codes) only in ``*_host`` files.

Usage: python scripts/eval_lockstep_navpy.py --run-dir DIR [--runs N] [--seed N]
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _path in (ROOT, ROOT / "src"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

from gcs.backend import instance_registry_lockstep as lockstep  # noqa: E402
from navpy.sim.lockstep.coordinator_members import Roster  # noqa: E402
from navpy.sim.lockstep.coordinator_run import LockstepCoordinator, RunResult  # noqa: E402
from navpy.sim.lockstep.coordinator_spawn import (  # noqa: E402
    PROCESSES_HOST_FILE,
    HostSidecar,
    LocalSpawner,
)
from navpy.sim.lockstep.coordinator_types import Manifest, WallBudgets  # noqa: E402
from navpy.sim.lockstep.operator_baton import OperatorBaton  # noqa: E402
from navpy.sim.lockstep.participant_hooks import RequestIds  # noqa: E402
from scripts import lockstep_wsl  # noqa: E402
from scripts.eval_gcs_demo_scenario import load_scenario  # noqa: E402
from scripts.eval_gcs_demo_sitl_root import SITL_START_TIME_UTC  # noqa: E402
from scripts.lockstep_wsl_sitl import (  # noqa: E402
    DRAIN_PERIOD_US,
    MAX_FRAME_GAP_US,
    HostDelays,
    WslSitl,
)
from scripts.lockstep_slice_actor import (  # noqa: E402
    BACKEND,
    NAVPY,
    SYS_ID,
    companion_script,
    slice_roles,
)

SLICE_SCENARIO = ROOT / "scripts" / "eval_lockstep_slice_scenario.json"
#: Host supervision of the participants (registration, a DONE, an exit).
WALL_BUDGETS = WallBudgets(register_s=120, done_s=120, exit_s=60)


@dataclass(frozen=True)
class RunOutcome:
    digest: str
    result: dict
    navpy_lines: int
    wall_s: float


def manifest(seed: int) -> Manifest:
    scenario = load_scenario(SLICE_SCENARIO)
    return Manifest(
        seed=seed, epoch_us=SITL_START_TIME_UTC * 1_000_000, max_frame_gap_us=MAX_FRAME_GAP_US,
        drain_period_us=DRAIN_PERIOD_US,
        # No SNAP in 9f: the mission phase ends at its first tick, and
        # finalization lasts until the actor returns.
        mission_budget_us=0, roles=slice_roles(scenario, navpy=True),
        initial=(("backend", BACKEND),), companion_roles=("navpy",),
    )


def run_once(run_dir: Path, slot: int, owner: dict, *, seed: int, delay_seed: int) -> RunOutcome:
    run_dir.mkdir(parents=True, exist_ok=False)
    scenario = load_scenario(SLICE_SCENARIO)
    plan = manifest(seed)
    delays = HostDelays(delay_seed)
    sitl = WslSitl(run_dir, slot, owner, seed=seed, delay_seed=delay_seed,
                   defaults=lockstep_wsl.merged_defaults())
    roster = Roster(plan, LocalSpawner(run_dir / "host", (str(ROOT / "src"), str(ROOT))),
                    lockstep.RegistryChildRecorder(slot, owner),
                    HostSidecar(run_dir / PROCESSES_HOST_FILE), WALL_BUDGETS)
    ids = RequestIds(0)
    operator = OperatorBaton(ids, companion_script(scenario))
    coordinator = LockstepCoordinator(plan, (sitl,), operator, roster, ids, run_dir / "host",
                                      delay=delays)
    started = time.perf_counter()
    try:
        sitl.start(scenario.home, SYS_ID)
        result: RunResult = coordinator.run()
    finally:
        sitl.stop()  # idempotent after the coordinator's own stop
    wall_s = time.perf_counter() - started
    journal = coordinator.journal
    navpy_lines = operator.journal.log_lines.get(NAVPY, [])
    (run_dir / "lockstep_journal.json").write_text(json.dumps(journal.records, indent=1) + "\n",
                                                   encoding="utf-8")
    (run_dir / "lockstep_digest.txt").write_text(journal.digest + "\n", encoding="utf-8")
    (run_dir / "navpy_log_lines.txt").write_text("\n".join(navpy_lines) + "\n", encoding="utf-8")
    (run_dir / "lockstep_run_host.json").write_text(json.dumps(
        {"slot": slot, "root": sitl.root, "wall_s": wall_s, "delay_seed": delay_seed,
         "result": asdict(result)}, indent=2) + "\n", encoding="utf-8")
    return RunOutcome(journal.digest, {"run": asdict(result), "actor": operator.result},
                      len(navpy_lines), wall_s)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--runs", type=int, default=2)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--delay-seed", type=int, default=7,
                        help="host delays of every run after the first")
    args = parser.parse_args()
    claims = []
    outcomes: list[RunOutcome] = []
    try:
        for _ in range(args.runs):  # every slot held at once: each run gets its own
            claims.append(lockstep.claim(run_token=f"seed-{args.seed}", clone=str(ROOT),
                                         owner=f"dir:{ROOT}"))
        for index, (slot, owner) in enumerate(claims):
            outcomes.append(run_once(args.run_dir / f"run{index + 1}", slot, owner, seed=args.seed,
                                     delay_seed=args.delay_seed if index else 0))
    finally:
        for slot, owner in claims:
            lockstep.release_if_owner(slot, owner)
    digests = [outcome.digest for outcome in outcomes]
    journals = {(args.run_dir / f"run{i + 1}" / "lockstep_journal.json").read_bytes()
                for i in range(len(outcomes))}
    lines = {(args.run_dir / f"run{i + 1}" / "navpy_log_lines.txt").read_bytes()
             for i in range(len(outcomes))}
    same = len(set(digests)) == 1 and len(journals) == 1 and len(lines) == 1
    print(json.dumps({"digests": digests, "repeats": same,
                      "navpy_log_lines": [o.navpy_lines for o in outcomes],
                      "wall_s": [round(o.wall_s, 1) for o in outcomes],
                      "results": [o.result["run"] for o in outcomes]}, indent=2))
    return 0 if same else 1


if __name__ == "__main__":
    os.environ.setdefault("PYTHONUTF8", "1")
    raise SystemExit(main())
