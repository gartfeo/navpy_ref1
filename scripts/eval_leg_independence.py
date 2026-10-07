"""Vary only the scored leg length, holding the approach geometry constant.

A converging final-approach law should return the same miss from a short leg as from
a long one.  If the miss tracks leg length instead, the law is not nulling the
error -- it is decaying towards it, and the result is whatever the clock
happened to reach.  That is the claim this measures.

Three things have to be controlled for the answer to mean anything:

1. Glide angle.  Shortening the leg with the POI altitude fixed steepens the
   required descent (340 m over 600 m is 29.5 deg; over 1300 m it is 14.7 deg),
   so leg length and glide geometry would move together and the result could
   not be attributed to either.  The POI altitude is therefore scaled with
   the leg to hold ``atan(drop / leg)`` fixed.

2. Gate placement.  The evaluator's ``--gate-offset`` is metres north of home,
   not the scored leg, and the gate must stay north of the loiter, so a leg is
   converted here: ``gate_offset = poi_offset - leg``.  Passing a leg length
   straight through as a gate offset is rejected by check_offsets.

3. Wind.  The lateral law consumes visual bearing and LOS rate, and is not
   demonstrated wind-invariant, so a calm result cannot be transferred to a
   crosswind question.  The wind is a parameter, defaulting to the condition
   under investigation rather than to calm.

Deferral is read from the navigation log with eval_descent_deferral rather than
inferred from the miss: a miss that grows as the leg shortens is consistent
with deferral but does not demonstrate it, and the deferral fraction measures
it directly.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

WORKTREE = Path(__file__).resolve().parent.parent
SCRIPTS = WORKTREE / "scripts"
sys.path[:0] = [str(WORKTREE / "src"), str(WORKTREE), str(SCRIPTS)]

from eval_descent_deferral import deferral  # noqa: E402
from eval_direct_pixel_pn import MIN_SAFE_POI_REL_ALT_M  # noqa: E402
from eval_sim_cpa_block import ab_eligible  # noqa: E402
from upload_north_line_mission import (  # noqa: E402
    DEFAULT_ALT_M,
    DEFAULT_WAYPOINT_OFFSET_M,
)

# Holding the glide angle fixed makes the POI altitude fall as the leg
# grows, so the longest usable leg is set by the mission altitude:
# poi_alt = mission_alt - 0.378 * leg must stay above the evaluator's floor.
# At the default 400 m mission altitude that caps the leg at 900 m, which is
# why these legs are shorter rather than spread either side of it.
DEFAULT_LEGS_M = (500.0, 700.0, 900.0)
# The geometry the other sweeps run, kept as the reference the glide angle is
# held to: 340 m of drop over a 900 m leg.
REFERENCE_LEG_M = 900.0
REFERENCE_DROP_M = DEFAULT_ALT_M - 60.0


def poi_alt_for(leg_m: float, mission_alt_m: float) -> float:
    """POI altitude that keeps the required glide angle constant.

    Same drop-per-metre as the reference case, so a short leg is not silently
    also a steeper dive.
    """
    return mission_alt_m - REFERENCE_DROP_M * (leg_m / REFERENCE_LEG_M)


def _case(
    python: Path,
    leg_m: float,
    args: argparse.Namespace,
) -> list[str]:
    gate_offset = args.poi_offset - leg_m
    poi_alt = poi_alt_for(leg_m, args.mission_alt)
    return [
        str(python),
        str(SCRIPTS / "eval_direct_pixel_pn.py"),
        "--speedups", "1",
        "--repetitions", str(args.repetitions),
        "--wind-speed", str(args.wind_speed),
        "--wind-dir", str(args.wind_dir),
        "--mission-alt", str(args.mission_alt),
        "--gate-offset", str(gate_offset),
        "--poi-offset", str(args.poi_offset),
        "--poi-alt", str(poi_alt),
        "--timeout", str(args.timeout),
    ]


def _exit_code(rows: list[dict[str, object]], launch_failures: int) -> int:
    """Nonzero unless every leg launched and produced a truth-scored run.

    The experiment ranks legs by certified truth misses; a sweep where a
    child evaluator failed, an artifact is missing, or a leg has zero scored
    runs must not report process success to CI or an orchestrator.
    """
    if launch_failures or not rows:
        return 1
    if any(not row["misses_m"] for row in rows):
        return 1
    return 0


def _artifact(stdout: str) -> Path | None:
    for line in reversed(stdout.splitlines()):
        if line.startswith("ARTIFACT "):
            return Path(line.split(" ", 1)[1].strip())
    return None


def _report(root: Path, leg_m: float, poi_alt: float) -> dict[str, object]:
    summary = json.loads((root / "summary.json").read_text(encoding="utf-8"))
    misses: list[float] = []
    unscored = 0
    deferrals: list[float] = []
    for result in summary.get("results", []):
        # Only rows passing the centralized A/B eligibility predicate rank
        # legs: certified simulator truth from a VALID run (the child's
        # snap is the EKF's error projection and ranked runs opposite to
        # truth) AND an accepted,
        # agreeing SIM_CPA cross-check.  A disagreed or module-absent row
        # stays a valid run but never a campaign statistic -- that is what
        # keeps the disagreement flag from being ornamental.  Pre-v3 rows have
        # no module block and are deliberately not grandfathered in.
        truth = result.get("truth") or {}
        distance = truth.get("dist_3d_m")
        if (
            ab_eligible(result)
            and isinstance(distance, (int, float))
            and not isinstance(distance, bool)
        ):
            misses.append(float(distance))
        else:
            unscored += 1
    for log in sorted(root.glob("speed-*/navpy-logs/*_navigation_compact.csv")):
        try:
            measured = deferral(log)
        except (OSError, ValueError):
            continue
        deferrals.append(measured.worst_deferral)
    return {
        "leg_m": leg_m,
        "poi_alt_m": poi_alt,
        "misses_m": misses,
        "unscored_runs": unscored,
        "deferral": deferrals,
        "artifact": str(root),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--python", type=Path, default=Path(sys.executable))
    parser.add_argument(
        "--legs",
        default=",".join(f"{leg:g}" for leg in DEFAULT_LEGS_M),
        help="scored leg lengths in metres, gate-to-POI",
    )
    parser.add_argument("--repetitions", type=int, default=3)
    parser.add_argument("--mission-alt", type=float, default=DEFAULT_ALT_M)
    parser.add_argument(
        "--poi-offset", type=float, default=DEFAULT_WAYPOINT_OFFSET_M
    )
    parser.add_argument("--wind-speed", type=float, default=10.0)
    parser.add_argument(
        "--wind-dir",
        type=float,
        default=270.0,
        help=(
            "degrees the wind blows FROM; defaults to the cross-left case the "
            "leg question was raised about, not to calm"
        ),
    )
    parser.add_argument("--timeout", type=float, default=400.0)
    args = parser.parse_args()

    legs = [float(piece) for piece in args.legs.split(",") if piece.strip()]
    for leg_m in legs:
        poi_alt = poi_alt_for(leg_m, args.mission_alt)
        if poi_alt < MIN_SAFE_POI_REL_ALT_M:
            print(
                f"leg {leg_m:g} m needs POI altitude {poi_alt:.1f} m to "
                f"hold the glide angle, below the {MIN_SAFE_POI_REL_ALT_M:g} m "
                f"floor. Raise --mission-alt to at least "
                f"{MIN_SAFE_POI_REL_ALT_M + REFERENCE_DROP_M * leg_m / REFERENCE_LEG_M:.0f}"
                " m, or use a shorter leg.",
                file=sys.stderr,
            )
            return 2
        if args.poi_offset - leg_m <= 0.0:
            print(
                f"leg {leg_m:g} m exceeds the POI offset "
                f"{args.poi_offset:g} m, so the gate would sit south of home",
                file=sys.stderr,
            )
            return 2
    rows: list[dict[str, object]] = []
    launch_failures = 0
    for leg_m in legs:
        gate_offset = args.poi_offset - leg_m
        poi_alt = poi_alt_for(leg_m, args.mission_alt)
        print(
            f"leg {leg_m:g} m -> gate {gate_offset:g} m, POI alt "
            f"{poi_alt:.1f} m (glide held at "
            f"{REFERENCE_DROP_M / REFERENCE_LEG_M:.3f} m per m)",
            flush=True,
        )
        completed = subprocess.run(
            _case(args.python.resolve(), leg_m, args),
            cwd=str(WORKTREE),
            capture_output=True,
            text=True,
        )
        if completed.returncode != 0:
            print(
                f"  evaluator exited {completed.returncode} for leg {leg_m:g}",
                file=sys.stderr,
            )
            launch_failures += 1
        root = _artifact(completed.stdout)
        if root is None:
            print(f"  no artifact for leg {leg_m:g}", file=sys.stderr)
            launch_failures += 1
            continue
        rows.append(_report(root, leg_m, poi_alt))
        print(
            f"  truth CPA errors {rows[-1]['misses_m']}"
            f" ({rows[-1]['unscored_runs']} unscored)",
            flush=True,
        )

    print()
    print(f"{'leg m':>8}{'POI alt':>12}{'truth CPA error m':>28}"
          f"{'unscored':>10}{'worst deferral':>20}")
    for row in rows:
        misses = ", ".join(f"{value:.3f}" for value in row["misses_m"])
        excess = ", ".join(f"{value:.2f}" for value in row["deferral"])
        print(f"{row['leg_m']:>8.0f}{row['poi_alt_m']:>12.1f}"
              f"{misses:>28}{row['unscored_runs']:>10}{excess:>20}")
    return _exit_code(rows, launch_failures)


if __name__ == "__main__":
    raise SystemExit(main())
