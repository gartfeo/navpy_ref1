"""Score a fleet exactly as the single case is scored: same pieces, per aircraft.

The first truth-scoring sweep could not be truth-scored at all: the fleet's
verdict came from the three-UAV `_verdict`, which reads only the EKF stream.
These helpers call the certified single-case pieces once per aircraft --
`TruthRecorder` against that aircraft's own home, `case_verdict` with
persistence deferred, `SimCpaStage.post_teardown` folded in only after the
SITL stack is down -- so a fleet row and a single-case row mean the same
thing.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parent
WORKTREE = SCRIPTS.parent
sys.path[:0] = [str(WORKTREE / "src"), str(WORKTREE), str(SCRIPTS)]

from scripts.eval_direct_pixel_verdict import (  # noqa: E402
    SCORING_POLICY_SITL_TRUTH,
    case_verdict,
    persist_verdict,
    unscored_result,
)
from scripts.eval_fleet_setup import FleetPlan  # noqa: E402
from scripts.eval_navigation_truth import TruthRecorder, salvage_truth  # noqa: E402


def truth_recorders(
    sys_ids: list[int],
    targets: dict[int, object],
    plan: FleetPlan,
    args: argparse.Namespace,
) -> dict[int, TruthRecorder]:
    """One recorder per aircraft, against that aircraft's own home.

    Empty under the estimate policy: a recorder existing implies the row can
    be truth-scored, and the truth verdict fails closed on a missing stream.
    """
    if args.scoring_policy != SCORING_POLICY_SITL_TRUTH:
        return {}
    return {
        sys_id: TruthRecorder(targets[sys_id], plan.home_alts[sys_id])
        for sys_id in sys_ids
    }


def fleet_verdicts(
    sys_ids: list[int],
    *,
    directories: dict[int, Path],
    child_results: dict[int, dict[str, object]],
    failures: dict[int, str],
    scorers: dict[int, object],
    tracks: dict[int, object],
    truths: dict[int, TruthRecorder],
    stages: dict[int, object],
    plan: FleetPlan,
    args: argparse.Namespace,
    assigned: dict[int, tuple[float, float]],
    speedup: float,
) -> tuple[dict[int, dict[str, object]], set[int]]:
    """One verdict per aircraft: certified where it flew, salvaged where not.

    A scored aircraft gets `case_verdict` with persistence deferred, because
    the SIM_CPA cross-check may only read a BIN once the SITL stack is down;
    `seal_fleet` folds it in and performs that aircraft's one atomic write.

    A failed aircraft -- collection failure OR an exception while its own
    verdict was being built -- gets the same schema-complete unscored
    verdict a failed single case gets: truth salvaged from what was
    observed, its child result preserved, the cross-check reduced to its
    error block, persisted immediately (it reads no BIN, so teardown order
    does not bind it). Its id is returned in the second value so
    `seal_fleet` leaves it sealed.
    """
    verdicts: dict[int, dict[str, object]] = {}
    unscored: set[int] = set()
    for sys_id in sys_ids:
        errors: list[str] = []
        if sys_id in failures:
            errors.append(failures[sys_id])
        else:
            try:
                # The recorder arrives finalized: collect_fleet stamps it at
                # this aircraft's own truth-door close. Stamping here would
                # measure the fleet's finish-time spread against the 2 s
                # freshness gate; a path that forgets the stamp trips the
                # "never finalized" certification error instead.
                truth = truths.get(sys_id)
                if truth is not None:
                    truth.write_track(directories[sys_id] / "truth_track.csv")
                verdicts[sys_id] = case_verdict(
                    directories[sys_id],
                    child_result=child_results[sys_id],
                    scorer=scorers[sys_id],
                    track=tracks[sys_id],
                    truth=truth,
                    truth_stream_accepted=plan.truth_stream_accepted.get(sys_id),
                    scoring_policy=args.scoring_policy,
                    max_distance_m=args.max_distance,
                    goal_distance_m=args.goal_distance,
                    wind_speed=assigned[sys_id][0],
                    wind_dir_deg=assigned[sys_id][1],
                    speedup=speedup,
                    persist=False,
                )
                continue
            except Exception as error:
                # Verdict construction legitimately raises on missing
                # evidence (a child that failed early leaves no navigation
                # log). One aircraft's missing evidence must not cost the
                # rest of the fleet their verdicts -- exactly the boundary
                # the single case draws with its own late-failure salvage.
                errors.append(
                    "verdict construction failed: "
                    f"{type(error).__name__}: {error}"
                )
        truth_block = salvage_truth(
            truths.get(sys_id), directories[sys_id], errors
        )
        verdicts[sys_id] = unscored_result(
            errors,
            scoring_policy=args.scoring_policy,
            truth_stream_acknowledged=plan.truth_stream_accepted.get(sys_id),
            truth_block=truth_block,
            child_result=child_results.get(sys_id),
            scorer=scorers[sys_id],
            case_dir=directories[sys_id],
            sim_cpa=stages[sys_id].error_block(),
        )
        unscored.add(sys_id)
    return verdicts, unscored


def seal_fleet(
    verdicts: dict[int, dict[str, object]],
    stages: dict[int, object],
    directories: dict[int, Path],
    assigned: dict[int, tuple[float, float]],
    unscored: set[int] = frozenset(),
) -> dict[str, dict[str, object]]:
    """Fold in the cross-check and give each aircraft its one atomic write.

    Callable only after the SITL stack is down: `post_teardown` parses BINs
    that are only trustworthy once every aircraft process has exited and
    flushed. Unscored aircraft were already persisted with their cross-check
    error block and are passed through untouched. The fleet row adds the
    wind keys the per-cell report groups by; the persisted per-aircraft file
    keeps the exact single-case schema.
    """
    vehicles: dict[str, dict[str, object]] = {}
    for sys_id, verdict in verdicts.items():
        if sys_id not in unscored:
            verdict["sim_cpa"] = stages[sys_id].post_teardown(
                directories[sys_id], sysid=sys_id,
                truth_block=verdict.get("truth"),
            )
            persist_verdict(directories[sys_id], verdict)
        vehicles[str(sys_id)] = {
            **verdict,
            "wind_speed_mps": assigned[sys_id][0],
            "wind_dir_deg": assigned[sys_id][1],
        }
    return vehicles


__all__ = ["fleet_verdicts", "seal_fleet", "truth_recorders"]
