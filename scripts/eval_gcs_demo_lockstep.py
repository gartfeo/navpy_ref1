"""``eval_gcs_navigation_demo.py --lockstep``: the 1-UAV slice in lockstep (step 9g).

One run (``lockstep_slice.md``, "Run phases"):

1. Claim an eval-band lockstep slot in-process (owner ruling 4); the claim's
   owner identity is the coordinator's from the start.
2. Build the hermetic run: a fresh WSL root with the serial-step firmware
   (``lockstep_wsl``), the input manifest, the participants' role configs.
3. The coordinator runs boot, mission, finalization, drain and snapshot. The
   operator (``run_mission``, the wall eval's own workflow) runs on the
   coordinator's baton through the lockstep ports, then stops the companion
   (finalization). An operator workflow that fails is the run's outcome, not
   a host fault: the companion is still stopped and the run drains cleanly.
4. Write the journal (``lockstep_journal.jsonl``, its digest), NavPy's log
   lines with their ticks, the approvals, and ``report.json`` (with the
   run-relative ``log_dir``); host facts only in ``*_host`` files.
5. Release the slot by ownership.

The participants run with the run directory as their working directory, so
NavPy's ``uav_<sys>_*`` files and the backend's ``gcs/`` land where the wall
eval puts them, and :func:`~scripts.eval_gcs_demo_analysis.analyze_run`
reads them with the scenario's gate profile (``owner_only_slice``, which adds
the journal's slice gates). Only that profile can run here: the certified
3-UAV scenario needs step 10.
"""

from __future__ import annotations

import json
import time
from collections.abc import Callable
from dataclasses import asdict, dataclass
from pathlib import Path

from gcs.backend import instance_registry_lockstep as lockstep
from navpy.sim.lockstep.coordinator_journal import RunJournal
from navpy.sim.lockstep.coordinator_members import Roster
from navpy.sim.lockstep.coordinator_run import LockstepCoordinator
from navpy.sim.lockstep.coordinator_spawn import PROCESSES_HOST_FILE, HostSidecar, LocalSpawner
from navpy.sim.lockstep.coordinator_types import Manifest, WallBudgets
from navpy.sim.lockstep.host_paths import HostPathScrubber
from navpy.sim.lockstep.operator_baton import OperatorBaton
from navpy.sim.lockstep.participant_hooks import RequestIds
from scripts import lockstep_wsl
from scripts.eval_certificate_source_time import SOURCE_TIME_SUBDIR
from scripts.eval_gcs_demo_audit import validate_approval_records
from scripts.eval_gcs_demo_confirmation import ConfirmationTimeout
from scripts.eval_gcs_demo_constants import ROOT, SRC
from scripts.eval_gcs_demo_gate_profiles import OWNER_ONLY_SLICE_PROFILE
from scripts.eval_gcs_demo_inputs import dem_tiles, scenario_points
from scripts.eval_gcs_demo_lifecycle import stop_companions
from scripts.eval_gcs_demo_lockstep_inputs import write_lockstep_input_manifest
from scripts.eval_gcs_demo_mission import run_mission
from scripts.eval_gcs_demo_models import RegressionError
from scripts.eval_gcs_demo_ports import JsonValue
from scripts.eval_gcs_demo_scenario import DemoScenario
from scripts.eval_gcs_demo_sitl_root import SITL_START_TIME_UTC, terrain_tiles
from scripts.eval_gcs_demo_slice_gates import JOURNAL_FILE, NAVPY_LINES_FILE
from scripts.lockstep_slice_actor import (
    BACKEND,
    NAVPY,
    ROUTER,
    SITL_PORTS,
    SYS_ID,
    slice_roles,
    stack_context,
)
from scripts.lockstep_wsl_sitl import DRAIN_PERIOD_US, MAX_FRAME_GAP_US, HostDelays, WslSitl

RUN_HOST_FILE = "lockstep_run_host.json"
APPROVALS_FILE = "operator_approvals.json"
#: The manifest epoch: the SITL's ``--start-time``, so ``unix`` = epoch + t_k.
EPOCH_US = SITL_START_TIME_UTC * 1_000_000
#: The coordinator's mission sim-time budget: a backstop only. The operator's
#: own waits (each at most ``--timeout-s`` of sim time) end a stalled mission
#: first, as an ``operator_returned`` outcome; this bounds an operator that
#: keeps waiting past them.
MISSION_BUDGET_S = 3600.0
#: Host supervision of the participants (registration, a DONE, an exit).
WALL_BUDGETS = WallBudgets(register_s=120, done_s=120, exit_s=60)


@dataclass(frozen=True)
class LockstepOptions:
    seed: int
    #: Sim seconds: each operator wait's budget (``--timeout-s``).
    timeout_s: float
    confirmation_delay_s: float
    #: Acceptance (c): seeded host delays in the relay, coordinator and
    #: participants; 0 is none. Never part of the journal.
    host_delay_seed: int = 0


@dataclass(frozen=True)
class LockstepRun:
    digest: str
    outcome: str
    approvals: list[dict[str, JsonValue]]
    #: The operator workflow's failure, its host paths as run tokens (``HostPathScrubber``).
    operator_error: str | None


def require_slice(scenario: DemoScenario) -> None:
    if scenario.gate_profile != OWNER_ONLY_SLICE_PROFILE or len(scenario.slots) != 1:
        raise RegressionError(
            f"--lockstep runs only the 1-UAV {OWNER_ONLY_SLICE_PROFILE!r} scenario "
            f"(scripts/eval_lockstep_slice_scenario.json); {scenario.path.name} has gate profile "
            f"{scenario.gate_profile!r} with {len(scenario.slots)} slots: the 3-UAV lockstep is step 10"
        )


def manifest(scenario: DemoScenario, options: LockstepOptions) -> Manifest:
    return Manifest(
        seed=options.seed, epoch_us=EPOCH_US, max_frame_gap_us=MAX_FRAME_GAP_US,
        drain_period_us=DRAIN_PERIOD_US, mission_budget_us=round(MISSION_BUDGET_S * 1_000_000),
        roles=slice_roles(scenario, navpy=True, pose_cadence_dir=SOURCE_TIME_SUBDIR),
        sitl_ports=SITL_PORTS, router=ROUTER, initial=(("backend", BACKEND),),
        companion_roles=("navpy",), expected_fleet=(NAVPY,),
    )


def operator_script(scenario: DemoScenario, run_dir: Path,
                    options: LockstepOptions) -> Callable[[OperatorBaton], dict]:
    """The wall eval's mission on the baton, then the companion stop (finalization)."""

    scrub = HostPathScrubber(run_dir)

    def script(operator: OperatorBaton) -> dict:
        operator.wait_ready(BACKEND)
        context = stack_context(operator, scenario, epoch_us=EPOCH_US)
        outcome: dict = {"approvals": [], "error": None}
        try:
            mission = run_mission(context, run_dir, options.timeout_s, options.confirmation_delay_s)
            outcome["approvals"] = mission.approvals
        except Exception as error:  # noqa: BLE001 - the workflow's outcome, reported and gated
            outcome["error"] = scrub(f"{type(error).__name__}: {error}")
            if isinstance(error, ConfirmationTimeout):
                outcome["approvals"] = error.approvals
        try:
            stop_companions(context)  # NavPy retires through the participant stop before drain
        except Exception as error:  # noqa: BLE001 - the coordinator then faults: not stopped
            outcome["error"] = (outcome["error"] or "") + scrub(f"; stop: {error}")
        return outcome

    return script


def _write_lines(path: Path, operator: OperatorBaton) -> None:
    lines = operator.journal.log_lines.get(NAVPY, [])
    ticks = operator.journal.log_ticks.get(NAVPY, [])
    path.write_bytes(b"".join(
        json.dumps([k, line], ensure_ascii=False, separators=(",", ":")).encode("utf-8") + b"\n"
        for k, line in zip(ticks, lines)))


def execute_lockstep_run(run_dir: Path, scenario: DemoScenario,
                         options: LockstepOptions) -> LockstepRun:
    require_slice(scenario)
    if not 0.0 < options.timeout_s < MISSION_BUDGET_S:
        raise RegressionError(f"--timeout-s must be within (0, {MISSION_BUDGET_S}) sim seconds")
    if run_dir.exists() and any(run_dir.iterdir()):
        raise RegressionError(f"lockstep run directory is not empty: {run_dir}")
    run_dir.mkdir(parents=True, exist_ok=True)
    dem_tiles(scenario_points(scenario))  # an input fault stops the run before launch
    plan = manifest(scenario, options)
    defaults = lockstep_wsl.merged_defaults()  # the SITL's defaults.parm, hashed and then written
    tiles = terrain_tiles()  # its terrain/, hashed and then written
    write_lockstep_input_manifest(run_dir, scenario, plan, options, defaults, tiles)
    slot, owner = lockstep.claim(run_token=f"seed-{options.seed}", clone=str(ROOT),
                                 owner=f"dir:{ROOT}")
    started = time.perf_counter()
    sitl = WslSitl(run_dir, slot, owner, seed=options.seed, delay_seed=options.host_delay_seed,
                   defaults=defaults, tiles=tiles)
    roster = Roster(plan, LocalSpawner(run_dir, (str(SRC), str(ROOT)),
                                       host_delay_seed=options.host_delay_seed),
                    lockstep.RegistryChildRecorder(slot, owner),
                    HostSidecar(run_dir / PROCESSES_HOST_FILE), WALL_BUDGETS)
    ids = RequestIds(0)
    operator = OperatorBaton(ids, operator_script(scenario, run_dir, options))
    coordinator = LockstepCoordinator(plan, (sitl,), operator, roster, ids, run_dir,
                                      delay=HostDelays(options.host_delay_seed))
    result = fault = None
    try:
        sitl.start(scenario.home, SYS_ID)
        result = coordinator.run()
    except Exception as error:  # noqa: BLE001 - journaled by the coordinator; re-raised below
        fault = error
    finally:
        try:
            sitl.stop()  # idempotent after the coordinator's own stop
        finally:
            lockstep.release_if_owner(slot, owner)
    journal: RunJournal = coordinator.journal
    journal.write(run_dir / JOURNAL_FILE)
    _write_lines(run_dir / NAVPY_LINES_FILE, operator)
    script = operator.result if isinstance(operator.result, dict) else {"approvals": [], "error": None}
    approvals = validate_approval_records(script["approvals"])
    (run_dir / APPROVALS_FILE).write_text(json.dumps({"approvals": approvals}, indent=2),
                                          encoding="utf-8")
    (run_dir / RUN_HOST_FILE).write_text(json.dumps({
        "slot": slot, "wsl_root": sitl.root, "wall_s": time.perf_counter() - started,
        "host_delay_seed": options.host_delay_seed,
        "result": None if result is None else asdict(result),
        "fault": None if fault is None else f"{type(fault).__name__}: {fault}",
    }, indent=2) + "\n", encoding="utf-8")
    if fault is not None:
        raise RegressionError(f"lockstep run fault: {type(fault).__name__}: {fault}") from fault
    return LockstepRun(journal.digest, result.outcome, approvals, script["error"])


__all__ = [
    "APPROVALS_FILE", "JOURNAL_FILE", "LockstepOptions", "MISSION_BUDGET_S", "LockstepRun", "NAVPY_LINES_FILE",
    "RUN_HOST_FILE", "execute_lockstep_run", "manifest", "operator_script", "require_slice",
]
