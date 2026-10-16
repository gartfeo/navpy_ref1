"""Steps 10e, 10f: the backend with 3 vehicles on ``lockstep:router``, the router on 3 SITLs from tick 1.

Infrastructure check, not a mission eval (``lockstep_fleet.md``, sub-step
10e). The coordinator launches a :class:`~scripts.lockstep_wsl_sitl.WslFleet`
(three serial-step SITLs at the wall homes) after the backend participant
registered; every SITL's serial1 is a router endpoint from tick 1, and the
backend alone owns the router's ``backend`` endpoint. The scripted fleet
actor (``scripts.lockstep_fleet_actor.fleet_script``) connects sysids 1-3 on
``lockstep:router``, reads each one's command bounds by name, writes each
one's AAS profile, uploads the fixture's three missions with their
readback, then starts the in-process ESP32 simulator; the run then drains
the backend and parks every SITL. No NavPy runs, so the mission ends
``operator_returned``.

With ``--navpy`` (10f) the backend's launcher port spawns NavPy 1-3 (roles
``navpy<i>``, the expected fleet 1-3) and the actor is
``lockstep_fleet_companions.navlink_script``: the companions up, the launch on
ESP32 channels 1-3, every NavPy's check-in, two swarm heartbeat periods,
then stop-all (``operator_returned``, before any SNAP). Every NAVLINK packet
a NavPy sent is traced hop by hop to each peer
(``navpy.sim.lockstep.navlink_trace``) into ``navlink_traces.json``; the run
passes only if every ordered pair of NavPys has a complete trace and every
packet sent before the first stop request reached each peer it is for (a
packet still in flight when stop-all retires its receiver may stop short),
with no ambiguous trace (a byte-identical resend of one UID). Each
NavPy's log lines go to ``navpy<i>_log_lines.txt``; both are compared.

The fixture (``scripts/eval_lockstep_fleet_scenario.json``) is the certified
scenario frozen at step 10e: its home and slots are the certified ones, and
its missions do not follow later changes to the certified scenario. Its
sidecar (``eval_lockstep_fleet_scenario.lock.json``) pins its canonical
sha256 and its source revision; a fixture that differs is refused, and the
hash goes into every run's ``lockstep_inputs.json``.

Every run claims its own eval-band slot (all held at once, run one after
another) with the delay seed given for it (relays, coordinator, participants
and firmware helper jitter; 0 is none). The journals, the actor's results,
the backend's log lines and its ``gcs/`` files must be byte-equal, and every
run must end ``operator_returned`` with no fault record. Acceptance needs at
least two runs; a single run (a negative control, e.g. ``--seed 2``) makes
no repeat claim and passes on its outcome alone. A run whose children (local
participants or SITLs) are not confirmed stopped keeps its slot
(``gcs_stop.py`` stops it from the ownership records); the other slots are
released. Per run directory: ``lockstep_journal.jsonl``,
``lockstep_inputs.json``, ``fleet_actor.json``, ``backend_log_lines.txt``
and ``host/gcs/``; host facts (slot, WSL root, wall time) only in ``*_host``
files.

Usage: python scripts/eval_lockstep_fleet_backend.py --run-dir DIR
       [--delay-seeds 0 7] [--seed 1] [--navpy]
Exit 0: every run ended as intended and, with two or more runs, the artifacts are equal.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
import time
from collections.abc import Callable
from dataclasses import asdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _path in (ROOT, ROOT / "src"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

from gcs.backend import instance_registry_lockstep as lockstep  # noqa: E402
from navpy.sim.lockstep.coordinator_members import Roster  # noqa: E402
from navpy.sim.lockstep.coordinator_run import LockstepCoordinator  # noqa: E402
from navpy.sim.lockstep.coordinator_spawn import PROCESSES_HOST_FILE, HostSidecar, LocalSpawner  # noqa: E402
from navpy.sim.lockstep.coordinator_types import Manifest, WallBudgets  # noqa: E402
from navpy.sim.lockstep.operator_baton import OperatorBaton  # noqa: E402
from navpy.sim.lockstep.participant_hooks import RequestIds  # noqa: E402
from scripts import lockstep_topology as topology  # noqa: E402
from scripts import lockstep_wsl  # noqa: E402
from scripts.eval_gcs_demo_config import build_settings  # noqa: E402
from scripts.eval_gcs_demo_lockstep_inputs import lockstep_sitl_inputs  # noqa: E402
from scripts.eval_gcs_demo_scenario import DemoScenario, load_scenario  # noqa: E402
from scripts.eval_gcs_demo_sitl_root import SITL_START_TIME_UTC, terrain_tiles  # noqa: E402
from scripts.lockstep_fleet_actor import FleetLayout, fleet_roles, fleet_script  # noqa: E402
from scripts.lockstep_fleet_companions import navlink_script  # noqa: E402
from scripts.lockstep_navlink_gate import TRACES_FILE, navlink_ok, write_navlink  # noqa: E402
from scripts.lockstep_wsl_sitl import (  # noqa: E402
    BOOT_BUDGET_US,
    DRAIN_PERIOD_US,
    MAX_FRAME_GAP_US,
    FleetLaunch,
    HostDelays,
    WslFleet,
    exchange_deadline,
)

FLEET_SCENARIO = ROOT / "scripts" / "eval_lockstep_fleet_scenario.json"
#: The fixture's pin: its canonical sha256 and the certified file's revision it was copied from.
FLEET_SCENARIO_LOCK = ROOT / "scripts" / "eval_lockstep_fleet_scenario.lock.json"
#: The one outcome a fleet run may end with: the actor returned (with no
#: NavPy, or with NavPys stopped before any SNAP); ``timeout`` or a fault is
#: a failed run.
EXPECTED_OUTCOME = "operator_returned"
FLEET = FleetLayout(3)
#: The mission's sim-time budget: a backstop only; the mission ends when the
#: actor returns (``operator_returned``: no SNAP).
MISSION_BUDGET_S = 600.0
#: Host supervision of the participants (registration, a DONE, an exit).
WALL_BUDGETS = WallBudgets(register_s=120, done_s=120, exit_s=60)
JOURNAL_FILE = "lockstep_journal.jsonl"
COMPARED_FILES = (JOURNAL_FILE, "fleet_actor.json", "backend_log_lines.txt")
#: ``--navpy``'s further compared files: the traces and each NavPy's log lines.
NAVPY_FILES = (TRACES_FILE, *(f"{role}_log_lines.txt" for role in FLEET.companion_roles))


def canonical_sha256(path: Path) -> str:
    """The sha256 of a JSON file's canonical form (sorted keys, no whitespace, ASCII)."""
    canonical = json.dumps(json.loads(path.read_text(encoding="utf-8")), sort_keys=True,
                           separators=(",", ":"), ensure_ascii=True)
    return hashlib.sha256(canonical.encode("ascii")).hexdigest()


def load_fleet_scenario() -> tuple[DemoScenario, dict]:
    """The frozen fixture, refused unless its canonical sha256 is the pinned one;
    returns it with its pin."""
    pin = json.loads(FLEET_SCENARIO_LOCK.read_text(encoding="utf-8"))
    actual = canonical_sha256(FLEET_SCENARIO)
    if actual != pin["canonical_sha256"]:
        raise ValueError(f"{FLEET_SCENARIO.name} changed ({actual}, pinned {pin['canonical_sha256']} "
                         f"from {pin['source']} at {pin['source_revision']})")
    return load_scenario(FLEET_SCENARIO), pin


def fleet_manifest(scenario: DemoScenario, seed: int, *, navpy: bool = False) -> Manifest:
    """The backend on the router device for sysids 1-3; with ``navpy`` each
    NavPy role its launcher port spawns, and the expected fleet."""
    return Manifest(
        seed=seed, epoch_us=SITL_START_TIME_UTC * 1_000_000, max_frame_gap_us=MAX_FRAME_GAP_US,
        drain_period_us=DRAIN_PERIOD_US, mission_budget_us=round(MISSION_BUDGET_S * 1_000_000),
        roles=fleet_roles(build_settings(scenario), FLEET, navpy=navpy),
        sitl_ports=FLEET.sitl_ports, router=FLEET.router, initial=(("backend", FLEET.backend),),
        boot_budget_us=BOOT_BUDGET_US,
        companion_roles=FLEET.companion_roles if navpy else (),
        expected_fleet=FLEET.expected_fleet if navpy else (),
    )


class UncleanTeardown(RuntimeError):
    """A run's children were not confirmed stopped: its slot keeps its
    ownership records for ``gcs_stop.py`` and is not released."""


def confirm_teardown(slot: int, *stops: Callable[[], None]) -> None:
    """Run every stop (each idempotent), whatever the others do; any failure
    is an :class:`UncleanTeardown` chained to the first one. The coordinator's
    own fault cleanup only logs its failures, so the run confirms here."""
    failures: list[BaseException] = []
    for stop in stops:
        try:
            stop()
        except BaseException as error:  # noqa: BLE001 - raised below as the teardown's cause
            failures.append(error)
    if failures:
        raise UncleanTeardown(f"slot {slot}: teardown not confirmed ({failures[0]}); "
                              "kept for python scripts/gcs_stop.py") from failures[0]


def run_once(run_dir: Path, slot: int, owner: dict, *, seed: int, delay_seed: int, navpy: bool = False) -> dict:
    run_dir.mkdir(parents=True, exist_ok=False)
    scenario, pin = load_fleet_scenario()
    script_sha256 = topology.run_swarm_sha256()
    topology.require_pinned_script(script_sha256)
    homes = topology.fleet_homes(scenario.home, FLEET.size)
    vehicles = tuple(zip(FLEET.sys_ids, homes))
    defaults, tiles = lockstep_wsl.merged_defaults(), terrain_tiles()
    plan = fleet_manifest(scenario, seed, navpy=navpy)
    (run_dir / "lockstep_inputs.json").write_text(json.dumps({
        "seed": seed, "boot_budget_us": plan.boot_budget_us,
        "scenario": {"file": FLEET_SCENARIO.name, "canonical_sha256": pin["canonical_sha256"],
                     "source": pin["source"], "source_revision": pin["source_revision"]},
        "sitl": lockstep_sitl_inputs(defaults, tiles),
        "topology": topology.topology_inputs(homes, script_sha256)}, indent=2) + "\n", encoding="utf-8")
    host_dir = run_dir / "host"
    wsl = WslFleet(run_dir, slot, owner, FleetLaunch(vehicles, seed, delay_seed, defaults, tiles))
    roster = Roster(plan, LocalSpawner(host_dir, (str(ROOT / "src"), str(ROOT)), host_delay_seed=delay_seed),
                    lockstep.RegistryChildRecorder(slot, owner),
                    HostSidecar(run_dir / PROCESSES_HOST_FILE), WALL_BUDGETS,
                    deadline=exchange_deadline())
    ids = RequestIds(0)
    operator = OperatorBaton(ids, (navlink_script if navpy else fleet_script)(scenario, FLEET))
    coordinator = LockstepCoordinator(plan, wsl.sitls, operator, roster, ids, host_dir,
                                      delay=HostDelays(delay_seed), launch=wsl.launch)
    started = time.perf_counter()
    result = fault = None
    try:
        result = coordinator.run()
    except Exception as error:  # noqa: BLE001 - journaled by the coordinator; re-raised below
        fault = error
    finally:
        # Both idempotent after the coordinator's own stops: the local participants, then the SITLs.
        confirm_teardown(slot, roster.stop_all, wsl.stop)
    wall_s = time.perf_counter() - started
    journal = coordinator.journal
    journal.write(run_dir / JOURNAL_FILE)
    (run_dir / "fleet_actor.json").write_text(
        json.dumps(operator.result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    lines = operator.journal.log_lines.get(FLEET.backend, [])
    (run_dir / "backend_log_lines.txt").write_text("".join(f"{line}\n" for line in lines), encoding="utf-8")
    navlink = write_navlink(run_dir, journal.records, operator.journal.log_lines, FLEET) if navpy else None
    rows = [r for r in journal.records if "k" in r and "t_us" in r and r.get("phase") != "drain"]
    sim_s = (rows[-1]["t_us"] - rows[0]["t_us"]) / 1e6 if rows else 0.0
    host = {"slot": slot, "wsl_root": wsl.root, "wall_s": wall_s, "delay_seed": delay_seed,
            "real_time_factor": sim_s / wall_s if wall_s else None,
            "result": None if result is None else asdict(result),
            "fault": None if fault is None else f"{type(fault).__name__}: {fault}"}
    (run_dir / "lockstep_run_host.json").write_text(json.dumps(host, indent=2) + "\n", encoding="utf-8")
    if fault is not None:
        raise RuntimeError(f"fleet backend run fault: {type(fault).__name__}: {fault}") from fault
    faults = [r for r in journal.records if "fault" in r]
    return {"digest": journal.digest, "ticks": len(rows), "sim_s": sim_s,
            "backend_log_lines": len(lines), "fault_records": len(faults), "navlink": navlink, **host}


def run_ok(outcome: dict) -> bool:
    """The run ended as intended: ``operator_returned``, no fault record and,
    with NavPys, a complete NAVLINK trace for every ordered pair, none lost
    before stop-all and none ambiguous."""
    result = outcome.get("result") or {}
    navlink = outcome.get("navlink")
    return (result.get("outcome") == EXPECTED_OUTCOME and outcome["fault"] is None
            and outcome["fault_records"] == 0 and (navlink is None or navlink_ok(navlink)))


def verdict(outcomes: list[dict], same: dict[str, bool]) -> dict:
    """Acceptance needs at least two runs, each ending as intended, with equal
    artifacts; a single run is a negative control: no repeat claim."""
    ok = bool(outcomes) and all(run_ok(o) for o in outcomes)
    if len(outcomes) < 2:
        return {"accepted": False, "repeats": None, "outcomes_ok": ok, "passed": ok}
    repeats = all(same.values())
    return {"accepted": ok and repeats, "repeats": repeats, "outcomes_ok": ok, "passed": ok and repeats}


def release_claims(claims: list[tuple[int, dict]], kept: set[int]) -> None:
    """Release every claim except the slots whose teardown was not confirmed:
    their registry entry still lists the children ``gcs_stop.py`` must stop."""
    for slot, owner in claims:
        if slot not in kept:
            lockstep.release_if_owner(slot, owner)


def _gcs_files(run_dir: Path) -> dict[str, bytes]:
    gcs = run_dir / "host" / "gcs"
    return {path.relative_to(gcs).as_posix(): path.read_bytes()
            for path in sorted(gcs.rglob("*")) if path.is_file()}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--delay-seeds", type=int, nargs="+", default=[0, 7],
                        help="one run per value, each in its own slot; acceptance needs two or more")
    parser.add_argument("--seed", type=int, default=1, help="the run seed (boot ids)")
    parser.add_argument("--navpy", action="store_true",
                        help="10f: NavPy 1-3, the launch, and the NAVLINK traces")
    args = parser.parse_args()
    claims, outcomes, kept = [], [], set()
    try:
        for _ in args.delay_seeds:  # every slot held at once: each run gets its own
            claims.append(lockstep.claim(run_token=f"seed-{args.seed}", clone=str(ROOT),
                                         owner=f"dir:{ROOT}"))
        for index, ((slot, owner), delay) in enumerate(zip(claims, args.delay_seeds)):
            try:
                outcomes.append(run_once(args.run_dir / f"run{index + 1}", slot, owner, seed=args.seed,
                                         delay_seed=delay, navpy=args.navpy))
            except UncleanTeardown:
                kept.add(slot)
                raise
    finally:
        release_claims(claims, kept)
    run_dirs = [args.run_dir / f"run{i + 1}" for i in range(len(outcomes))]
    same = {}
    if len(run_dirs) > 1:
        compared = COMPARED_FILES + (NAVPY_FILES if args.navpy else ())
        same = {name: len({(run / name).read_bytes() for run in run_dirs}) == 1 for name in compared}
        same["gcs"] = all(_gcs_files(run) == _gcs_files(run_dirs[0]) for run in run_dirs)
    result = verdict(outcomes, same)
    print(json.dumps({**result, "same": same, "runs": outcomes}, indent=2))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    os.environ.setdefault("PYTHONUTF8", "1")
    raise SystemExit(main())
