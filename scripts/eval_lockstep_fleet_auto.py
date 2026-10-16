"""Step 10d: three serial-step SITLs boot under the fleet barrier and fly AUTO.

Infrastructure check, not a mission eval (``lockstep_fleet.md``, sub-step
10d): a :class:`~scripts.lockstep_wsl_sitl.WslFleet` (one WSL root, three
instances, three relays, the wall homes) launched on the exchange deadline,
the fleet barrier and clock (``SitlFleet``) and step 7's scripted operator
on each SITL's serial1 (``ScriptedFleetRun``: param, mission, GPS fix, AUTO,
ARM, a flight check, x3). Every run claims its own eval-band slot (all held
at once, run one after another); each gets the delay seed given for it
(relays, coordinator and firmware helper jitter; 0 is none). The journals
must be byte-equal. Per run directory: ``lockstep_journal.jsonl`` and its
digest, ``lockstep_steps.json``, ``lockstep_inputs.json`` (topology and SITL
inputs); host facts (slot, WSL root, wall time) only in ``*_host`` files.

Usage: python scripts/eval_lockstep_fleet_auto.py --run-dir DIR
       [--delay-seeds 0 0 7] [--seed 1] [--ticks N]
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from dataclasses import asdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _path in (ROOT, ROOT / "src"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

from gcs.backend import instance_registry_lockstep as lockstep  # noqa: E402
from navpy.sim.lockstep.coordinator_fleet import SitlFleet  # noqa: E402
from navpy.sim.lockstep.coordinator_types import ROUTER_DEVICE, Manifest, fleet_sitl_ports  # noqa: E402
from navpy.sim.lockstep.scripted_fleet import ScriptedFleetRun  # noqa: E402
from navpy.sim.lockstep.scripted_operator import ScriptedFleetOperator  # noqa: E402
from scripts import lockstep_topology as topology  # noqa: E402
from scripts import lockstep_wsl  # noqa: E402
from scripts.eval_gcs_demo_lockstep_inputs import lockstep_sitl_inputs  # noqa: E402
from scripts.eval_gcs_demo_scenario import CERTIFIED_SCENARIO_PATH, load_scenario  # noqa: E402
from scripts.eval_gcs_demo_sitl_root import SITL_START_TIME_UTC, terrain_tiles  # noqa: E402
from scripts.lockstep_wsl_sitl import (  # noqa: E402
    BOOT_BUDGET_US,
    DRAIN_PERIOD_US,
    MAX_FRAME_GAP_US,
    FleetLaunch,
    HostDelays,
    WslFleet,
    exchange_deadline,
)

SYS_IDS = (1, 2, 3)
TICK_HZ = 50
#: Sim seconds flown after every vehicle's script finished (step 7's).
FLY_AFTER_SCRIPT_S = 5.0


def fleet_manifest(seed: int, boot_budget_us: int | None = BOOT_BUDGET_US) -> Manifest:
    """The SITL side of the 3-SITL lockstep manifest: no participants yet (10e, 10f)."""
    return Manifest(seed=seed, epoch_us=SITL_START_TIME_UTC * 1_000_000,
                    max_frame_gap_us=MAX_FRAME_GAP_US, drain_period_us=DRAIN_PERIOD_US,
                    mission_budget_us=0, roles={}, sitl_ports=fleet_sitl_ports(len(SYS_IDS)),
                    router=ROUTER_DEVICE, boot_budget_us=boot_budget_us)


def run_once(run_dir: Path, slot: int, owner: dict, *, seed: int, delay_seed: int, ticks: int,
             boot_budget_us: int | None) -> dict:
    run_dir.mkdir(parents=True, exist_ok=False)
    script_sha256 = topology.run_swarm_sha256()
    topology.require_pinned_script(script_sha256)
    homes = topology.fleet_homes(load_scenario(CERTIFIED_SCENARIO_PATH).home, len(SYS_IDS))
    vehicles = tuple(zip(SYS_IDS, homes))
    defaults, tiles = lockstep_wsl.merged_defaults(), terrain_tiles()
    plan = fleet_manifest(seed, boot_budget_us)
    (run_dir / "lockstep_inputs.json").write_text(json.dumps({
        "seed": seed, "boot_budget_us": boot_budget_us, "max_ticks": ticks,
        "sitl": lockstep_sitl_inputs(defaults, tiles),
        "topology": topology.topology_inputs(homes, script_sha256)}, indent=2) + "\n", encoding="utf-8")
    wsl = WslFleet(run_dir, slot, owner, FleetLaunch(vehicles, seed, delay_seed, defaults, tiles))
    deadline = exchange_deadline()
    operator = ScriptedFleetOperator(vehicles)
    finished: list[int] = []

    def stop_when() -> bool:
        if operator.done and not finished:
            finished.append(operator.tick)
        return bool(finished) and operator.tick >= finished[0] + FLY_AFTER_SCRIPT_S * TICK_HZ

    run = ScriptedFleetRun(plan, SitlFleet(plan, wsl.sitls, deadline, launch=wsl.launch), deadline,
                           operator, max_ticks=ticks, stop_when=stop_when,
                           delay=HostDelays(delay_seed))
    started = time.perf_counter()
    result = fault = None
    try:
        result = run.run()
    except Exception as error:  # noqa: BLE001 - journaled by the run; re-raised below
        fault = error
    finally:
        try:
            operator.close()
        finally:
            wsl.stop()  # idempotent after the run's own stop
    wall_s = time.perf_counter() - started
    run.journal.write(run_dir / "lockstep_journal.jsonl")
    (run_dir / "lockstep_steps.json").write_text(json.dumps(
        {str(v.sysid): {"steps": [asdict(s) for s in v.steps],
                        "statustext": [asdict(s) for s in v.statustexts]}
         for v in operator.vehicles}, indent=2) + "\n", encoding="utf-8")
    rows = [r for r in run.journal.records if "k" in r]
    sim_s = (rows[-1]["t_us"] - rows[0]["t_us"]) / 1e6 if rows else 0.0
    host = {"slot": slot, "wsl_root": wsl.root, "wall_s": wall_s, "delay_seed": delay_seed,
            "real_time_factor": sim_s / wall_s if wall_s else None,
            "fault": None if fault is None else f"{type(fault).__name__}: {fault}"}
    (run_dir / "lockstep_run_host.json").write_text(json.dumps(host, indent=2) + "\n", encoding="utf-8")
    if fault is not None:
        raise RuntimeError(f"fleet run fault: {type(fault).__name__}: {fault}") from fault
    barrier = next(r for r in rows if r["k"] == result.barrier_tick)
    return {"digest": run.journal.digest, "result": asdict(result), "ticks": len(rows),
            "barrier_after_frame1_us": barrier["t_us"] - rows[0]["t_us"], "sim_s": sim_s, **host}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--delay-seeds", type=int, nargs="+", default=[0, 0, 7],
                        help="one run per value, each in its own slot")
    parser.add_argument("--seed", type=int, default=1, help="the run seed (boot ids)")
    parser.add_argument("--ticks", type=int, default=12_000)
    parser.add_argument("--no-boot-budget", action="store_true",
                        help="record the barrier without a budget (how BOOT_BUDGET_US was set)")
    args = parser.parse_args()
    budget = None if args.no_boot_budget else BOOT_BUDGET_US
    claims, outcomes = [], []
    try:
        for _ in args.delay_seeds:  # every slot held at once: each run gets its own
            claims.append(lockstep.claim(run_token=f"seed-{args.seed}", clone=str(ROOT),
                                         owner=f"dir:{ROOT}"))
        for index, ((slot, owner), delay) in enumerate(zip(claims, args.delay_seeds)):
            outcomes.append(run_once(args.run_dir / f"run{index + 1}", slot, owner, seed=args.seed,
                                     delay_seed=delay, ticks=args.ticks, boot_budget_us=budget))
    finally:
        for slot, owner in claims:
            lockstep.release_if_owner(slot, owner)
    journals = {(args.run_dir / f"run{i + 1}" / "lockstep_journal.jsonl").read_bytes()
                for i in range(len(outcomes))}
    steps = {(args.run_dir / f"run{i + 1}" / "lockstep_steps.json").read_bytes()
             for i in range(len(outcomes))}
    same = len(journals) == 1 and len(steps) == 1
    print(json.dumps({"repeats": same, "runs": outcomes}, indent=2))
    return 0 if same else 1


if __name__ == "__main__":
    os.environ.setdefault("PYTHONUTF8", "1")
    raise SystemExit(main())
