"""Fly a whole wind matrix at once: one launch, one process per aircraft.

A wind sweep run one cell at a time costs about 3.5 hours for nine cells at
eight repeats. The cells share nothing -- each is only a different
SIM_WIND_SPD/SIM_WIND_DIR -- so the serialisation was never physics, only the
harness flying one aircraft per launch. This flies every cell simultaneously
and finishes in roughly one case time.

WHY NOT THE THREE-UAV HARNESS. It pins `instances=3` via `sysids_for_chat`,
and it never uploads a mission, so it flies the launcher's coverage plan --
whose legs TURN, making wind-relative-to-track meaningless mid-run (which is
why it rightly refuses geometry flags). This module uploads the north line to
every aircraft instead; a straight leg is what makes a wind cell mean one
thing.

PAST THREE AIRCRAFT THE SHARED LAUNCHER IS BLIND. Three per chat is a GCS
convention, not a SITL limit -- but `swarm_run_runner`/`swarm_run_verifier`
verify only `sysids_for_chat(chat)[:instances]` and the shared teardown kills
that same three-id list, so aircraft four and up are neither launch-verified
nor torn down there. `eval_fleet_span` claims every chat in the span
all-or-nothing and tears down the exact sysids launched; `eval_fleet_clock`
certifies EVERY aircraft's rate off the shared link before any of them fly.

IDENTICAL STARTS. `dist=0` collapses the launcher's 50 m grid to a single
point, so every aircraft begins where every other one does. A matrix that
varies only wind must not also vary the start: a run that inherited a
different home once read as an 8 m navigation failure when the law was fine.
Separate SITL processes have no collision model, so co-located aircraft are
safe.

CELLS ARE RANDOMISED WITHIN EACH REPLICATE -- a randomised complete block
design, not a rotation. Launch order has carried real effects, and a fixed
round-robin gives every cell a constant mean launch position that aliases
onto cell index; `eval_fleet_cells.assign` and its tests carry the full
rationale. The seed is recorded so a matrix stays reproducible.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any

WORKTREE = Path(__file__).resolve().parent.parent
SCRIPTS = Path(__file__).resolve().parent
sys.path[:0] = [str(WORKTREE / "src"), str(WORKTREE), str(SCRIPTS)]

from gcs.backend import instance_ports as ip  # noqa: E402
from scripts import eval_direct_pixel_pn as one  # noqa: E402
from scripts import eval_direct_pixel_pn_three_uav as many  # noqa: E402
from scripts.eval_fleet_cells import (  # noqa: E402
    DEFAULT_CELLS, assign, parse_cells,
)
from scripts.eval_fleet_clock import verify_fleet_clock  # noqa: E402
from scripts.eval_fleet_collect import collect_fleet  # noqa: E402
from scripts.eval_fleet_manifest import write_case_manifest  # noqa: E402
from scripts.eval_fleet_cli import build_parser
from scripts.eval_fleet_result import persist_fleet_result
from scripts.eval_fleet_report import report  # noqa: E402
from scripts.eval_fleet_score import (  # noqa: E402
    fleet_verdicts, seal_fleet, truth_recorders,
)
from scripts.eval_fleet_setup import (  # noqa: E402
    AircraftManifest, fleet_source_identity, prepare_aircraft,
    require_same_source, upload_missions,
)
from scripts.eval_fleet_span import (  # noqa: E402
    chats_spanned, check_span, claim_span, cleanup_sysids, fleet_sysids,
    hand_back,
)
import eval_preboot_params as preboot  # noqa: E402
import eval_preboot_profile as profile  # noqa: E402
from scripts.eval_ground_track import GroundTrackRecorder  # noqa: E402
from scripts.eval_navigation_cases import (  # noqa: E402
    CoordinateScorer, stop_own_stack, wait_for_heartbeat,
)
from scripts.eval_sim_cpa_config import resolve_mode  # noqa: E402
from scripts.eval_sim_parameters import sim_parameters  # noqa: E402
from scripts.eval_sim_cpa_stage import SimCpaStage  # noqa: E402
from scripts.pixel_pn_final_approach_speed import launch_speedup  # noqa: E402


def gate_preboot_defaults(
    master: Any, sys_ids: list[int], args: Any, defaults: str | None,
    owned: dict[str, float] | None = None,
) -> None:
    """Check boot readback and reject overrides from later writers."""
    if defaults:
        preboot.verify_defaults(
            master, sys_ids, defaults, owned,
            lambda: sim_parameters(args) + preboot.LATER_WRITERS,
        )


def _parser() -> argparse.ArgumentParser:
    return build_parser(one._parser)


def run_fleet(
    python: Path, root: Path, *, speedup: float, repetition: int,
    args: argparse.Namespace,
    baseline_identity: dict[str, object] | None = None,
) -> dict[str, object]:
    # Each replicate gets its own shuffle; without the repetition the same
    # matrix would repeat one arrangement and the block design would collapse
    # back into a fixed assignment.
    seed = args.assign_seed + repetition
    launch_rate = launch_speedup(args.cruise_speedup, speedup)
    winds = assign(parse_cells(args.cells), args.cell_reps, seed=seed)
    chat = ip.eval_band()[0] if args.chat is None else args.chat
    sys_ids = fleet_sysids(chat, len(winds))
    check_span(chat, sys_ids)
    case_dir = root / f"speed-{speedup:g}-run-{repetition}"
    case_dir.mkdir(parents=True)

    # Captured BEFORE anything launches, so the identity recorded is the code
    # the aircraft will actually fly -- the FLEET closure, which contains this
    # harness itself, not only the single-case files.
    identity = fleet_source_identity()
    if baseline_identity is not None and identity != baseline_identity:
        raise one.SourceChangedError(
            "source changed between fleet launches "
            f"({baseline_identity['sha256'][:12]} -> {identity['sha256'][:12]})"
        )

    # Before the claim: an unreadable profile should cost nothing to
    # recover from, and after it there would be a slot to hand back.
    defaults, owned = profile.defaults_for(args, case_dir)

    # Ownership before any pkill, and held for the whole case -- not a snapshot
    # another session can invalidate between the check and the kill.
    held = claim_span(sys_ids, supervised_by_launcher=chat)
    swarm = master = None
    children: dict[int, object] = {}
    try:
        # Inside the try so a failure here still releases the claim.
        cleanup_sysids(chat, sys_ids)
        swarm, launch = one._start_swarm(
            python, case_dir, launch_rate, instances=len(winds),
            home=getattr(args, "home", None), chat=chat, dist=0,
            sitl_defaults=defaults,
        )
        if launch.chat != chat:
            raise RuntimeError(
                f"launcher used chat {launch.chat}, not the claimed {chat}; "
                f"the span held is {chats_spanned(sys_ids)}"
            )
        device = ip.monitor_device(chat)

        upload_missions(device, sys_ids, args, case_dir)

        master = wait_for_heartbeat(device, 120.0)
        if master is None:
            raise RuntimeError("no evaluator heartbeat")

        gate_preboot_defaults(master, sys_ids, args, defaults, owned)

        # Made first: the stages record pre-flight evidence into these.
        directories = {sys_id: case_dir / f"uav-{sys_id}" for sys_id in sys_ids}
        for directory in directories.values():
            directory.mkdir()
        # One SIM_CPA stage per aircraft, bound at the certified lifecycle
        # point: parameters, then pre_flight, then the child. The stage never
        # raises; a cross-check fault flies stream-scored.
        stages = {sys_id: SimCpaStage(args) for sys_id in sys_ids}
        plan = prepare_aircraft(
            master, sys_ids, winds, args=args, speedup=speedup,
            case_dirs=directories, sim_cpa=stages,
            # Each aircraft also gets the certified single-case `case.json`:
            # the SIM_CPA scorer reads its expected POI back from it, and
            # it records the cell's EFFECTIVE parameter pushes per aircraft.
            manifest=AircraftManifest(
                identity=identity, speedup=speedup,
                launch_speedup=launch_rate, repetition=repetition,
            ),
        )
        pois, assigned = plan.pois, plan.winds
        rates = verify_fleet_clock(master, sys_ids, launch_rate)

        require_same_source(identity, "during launch")
        write_case_manifest(
            case_dir, speedup=speedup, launch_speedup=launch_rate,
            repetition=repetition, chat=chat, span=chats_spanned(sys_ids),
            assign_seed=seed, winds=assigned, clock_rates=rates,
            identity=identity,
        )

        children = {
            sys_id: one._launch_child(
                python, directories[sys_id],
                device=ip.companion_device(sys_id), sysid=sys_id,
                poi=pois[sys_id],
                scoring_start_seq=plan.scoring_start_sequences[sys_id],
                timeout_s=args.timeout,
                speed_plan=plan.speed_plans[sys_id],
            )
            for sys_id in sys_ids
        }
        for sys_id, child in children.items():
            one._wait_ready(directories[sys_id] / "child.out.log", child, 180.0)
        require_same_source(identity, "while the fleet was starting")
        many._start_missions(master, sys_ids)

        scorers = {sys_id: CoordinateScorer(pois[sys_id]) for sys_id in sys_ids}
        tracks = {sys_id: GroundTrackRecorder(pois[sys_id]) for sys_id in sys_ids}
        truths = truth_recorders(sys_ids, pois, plan, args)
        child_results, failures = collect_fleet(
            master, children, directories, scorers, tracks, truths, args.timeout
        )
        # Scoring below runs whatever is on disk NOW: the flown identity
        # must still hold, or old flights get scored by new arithmetic.
        require_same_source(identity, "while the fleet flew")
        verdicts, unscored = fleet_verdicts(
            sys_ids, directories=directories, child_results=child_results,
            failures=failures, scorers=scorers, tracks=tracks, truths=truths,
            stages=stages, plan=plan, args=args, assigned=assigned,
            speedup=speedup,
        )
        # Ordered teardown BEFORE the cross-check reads anything: BINs are
        # only trustworthy once every aircraft process exited and flushed;
        # cleanup_sysids kills the exact sysids launched (the shared teardown
        # stops at three). The finally below backstops earlier exits.
        master.close()
        master = None
        for child in children.values():
            one._terminate(child)
        children = {}
        one._terminate(swarm)
        swarm = None
        cleanup_sysids(chat, sys_ids)
        vehicles = seal_fleet(verdicts, stages, directories, assigned, unscored)
        return persist_fleet_result(
            case_dir, vehicles, args.scoring_policy, identity,
            fleet_source_identity(), rates, sys_ids, seed, chat,
        )
    finally:
        if master is not None:
            master.close()
        for child in children.values():
            one._terminate(child)
        one._terminate(swarm)
        hand_back(
            chat, sys_ids, held,
            release_launcher_chat=lambda: stop_own_stack(python, chat=chat),
        )


def main() -> int:
    args = _parser().parse_args()
    # Resolved (and override-validated) BEFORE anything launches: a
    # redirected cross-check POI must die here, not minutes into a case.
    args.sim_cpa_mode = resolve_mode(args.sim_cpa, args.sitl_param)
    root = (
        one.WORKTREE / ".sitl-runs"
        / f"direct-pixel-pn-fleet-{time.strftime('%Y%m%d-%H%M%S')}"
    )
    root.mkdir(parents=True)
    # One baseline for the whole matrix, like the single-case driver: fleets
    # that keep flying across an edit cannot be compared to each other, which
    # is the failure that voided sweep 20260814-001114.
    baseline = fleet_source_identity()
    (root / "source_identity.json").write_text(
        json.dumps(baseline, indent=2), encoding="utf-8"
    )
    results = []
    aborted = False
    for speedup in one._speeds(args.speedups):
        if aborted:
            break
        for repetition in range(1, args.repetitions + 1):
            try:
                result = run_fleet(
                    args.python.resolve(), root,
                    speedup=speedup, repetition=repetition, args=args,
                    baseline_identity=baseline,
                )
            except Exception as error:
                result = {"passed": False, "errors": [f"{type(error).__name__}: {error}"]}
                aborted = isinstance(error, one.SourceChangedError)
            result.update({"speedup": speedup, "repetition": repetition})
            results.append(result)
            print(("PASS" if result["passed"] else "FAIL"), speedup, repetition, flush=True)
            if result.get("vehicles"):
                report(result["vehicles"], args.goal_distance)
            if aborted:
                print("ABORTED: source changed mid-matrix; remaining cases skipped", flush=True)
                break
    summary = {"passed": all(row["passed"] for row in results), "results": results}
    (root / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(f"ARTIFACT {root}")
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
