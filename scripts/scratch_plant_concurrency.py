"""SCRATCH: does running N plant-ID instances in parallel change the result?

Parallelism is only worth using if it is free of measurement cost. A
CPU-contended SITL still integrates its physics correctly in SIM time, so a
starved run reports perfectly plausible attitudes and flight-path angles while
its seconds quietly stop meaning seconds. This runs one IDENTICAL case at
several concurrency levels and compares both the physics and the clock.

Every child already self-reports `clock_fidelity`; this harness exists to find
the concurrency at which that gate starts tripping, and to confirm the physics
agrees below it.

Delete with the other plant scratch harnesses once the matrix is done.
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

WORKTREE = Path(__file__).resolve().parent.parent
SCRIPTS = Path(__file__).resolve().parent
sys.path[:0] = [str(WORKTREE / "src"), str(WORKTREE), str(SCRIPTS)]

from gcs.backend import instance_ports as ip  # noqa: E402
from scratch_plant_id_eval import run_case  # noqa: E402

EVAL_CHAT_LO, EVAL_CHAT_HI = ip.eval_band()

# Compared across concurrency levels. Physics first, then the clock that decides
# whether the physics numbers are even denominated in real seconds.
COMPARED_STEADY = ("act_pitch_deg", "gamma_deg", "implied_aoa_deg", "air_speed_mps")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--python", type=Path, default=Path(sys.executable))
    parser.add_argument(
        "--levels",
        # 1, not 1,4,10,20. `_run_level` rejects every level above 1 from a
        # single worktree, so the old default guaranteed a SystemExit on its
        # own second level -- a harness that could not finish its own default
        # invocation.
        default="1",
        help="concurrency levels to test, in order. Levels above 1 are refused "
             "from a single worktree; see the guard in `_run_level`.",
    )
    parser.add_argument("--pitch", type=float, default=-20.0)
    parser.add_argument("--roll", type=float, default=0.0)
    parser.add_argument("--throttle", type=float, default=0.55)
    parser.add_argument("--level-s", type=float, default=3.0)
    parser.add_argument("--hold-s", type=float, default=20.0)
    parser.add_argument("--timeout", type=float, default=600.0)
    parser.add_argument("--home", default=None)
    parser.add_argument("--mission-alt", type=float, default=None)
    parser.add_argument("--gate-offset", type=float, default=None)
    parser.add_argument("--target-offset", type=float, default=None)
    parser.add_argument("--engage-wp", type=int, default=2, dest='scoring_start_wp')
    return parser


def _case_args(args: argparse.Namespace) -> argparse.Namespace:
    """Build the namespace `run_case` expects, filling harness defaults."""
    from scratch_plant_id_eval import _parser as case_parser

    defaults = case_parser().parse_args([])
    for name in (
        "throttle", "level_s", "hold_s", "timeout", "scoring_start_wp",
    ):
        setattr(defaults, name, getattr(args, name))
    for name in ("home", "mission_alt", "gate_offset", "target_offset"):
        value = getattr(args, name)
        if value is not None:
            setattr(defaults, name, value)
    defaults.python = args.python
    return defaults


def run_level(
    python: Path,
    root: Path,
    level: int,
    args: argparse.Namespace,
) -> dict[str, object]:
    """Launch `level` identical cases at once and collect every result."""
    level_root = root / f"n{level:02d}"
    level_root.mkdir(parents=True)
    case_args = _case_args(args)
    # LIMIT OF THE CURRENT IMPLEMENTATION, not a property of the harness.
    # `instance_registry_store.owner_for` keys a slot by ABSOLUTE WORKTREE PATH,
    # and `swarm_run_wsl.resolve_chat` calls `find_for_owner` before claiming --
    # so every concurrent eval launch from this directory resolves to this
    # directory's single eval slot. The losers die with "chat N already has a
    # live SITL supervisor".
    #
    # Passing `--chat` explicitly is not a general workaround, but the failure
    # is narrower than "never recorded". resolve_chat returns the explicit value
    # immediately (swarm_run_wsl.py:24-26) without claiming, yet swarm_run.py:248
    # still calls `begin_sitl_launch`, which (instance_registry_claims.py:238-240)
    # returns False WITHOUT recording when that chat has no registry entry, and
    # updates the entry when it does. So an UNREGISTERED explicit chat is never
    # recorded and the caller waits out its full timeout; a REGISTERED one is
    # recorded, and refused if a live supervisor already holds it.
    #
    # What is scarce is the REGISTRY SLOT, not the aircraft. VEHICLES_PER_CHAT is
    # 3 (instance_ports.py:52) and swarm_run_runner.py:145 launches
    # `args.instances` sysids under ONE supervisor, so up to three cases could
    # share a single slot on distinct sysids. Batching that way, or letting one
    # worktree hold several slots, are both open options: the worktree key exists
    # to identify WHICH code is flying, not to cap how many instances may fly.
    if level > 1:
        raise SystemExit(
            f"concurrency {level} is not supported from a single worktree "
            "today: eval slots are keyed by worktree path, so parallel launches "
            "from here all resolve to the same slot. Run each instance from its "
            "own worktree, run serially, or batch cases onto distinct sysids "
            "within one slot."
        )
    started_s = time.monotonic()
    # Threads, not processes: each worker only supervises subprocesses and
    # blocks on IO, so the GIL is not in the way and a shared failure is easy
    # to surface.
    with ThreadPoolExecutor(max_workers=level) as pool:
        futures = [
            pool.submit(
                run_case,
                python,
                level_root,
                args.pitch,
                args.roll,
                case_args,
                suffix=f"-i{index:02d}",
            )
            for index in range(level)
        ]
        payloads = [future.result() for future in futures]
    return {
        "level": level,
        "wall_s": time.monotonic() - started_s,
        "results": payloads,
    }


def _summarise(level_payload: dict) -> dict[str, object]:
    rows = level_payload["results"]
    passed = [row for row in rows if row.get("passed") is True]
    summary: dict[str, object] = {
        "level": level_payload["level"],
        "wall_s": round(level_payload["wall_s"], 1),
        "runs": len(rows),
        "passed": len(passed),
        "failures": [
            row.get("missing") or row.get("errors") or row.get("aborted")
            for row in rows
            if row.get("passed") is not True
        ],
    }
    for name in COMPARED_STEADY:
        values = [
            (row.get("steady") or {}).get(name)
            for row in passed
        ]
        values = [value for value in values if isinstance(value, (int, float))]
        summary[name] = {
            "mean": statistics.fmean(values) if values else None,
            "spread": (max(values) - min(values)) if len(values) > 1 else 0.0,
            "n": len(values),
        }
    fidelity = [
        (row.get("clock_fidelity") or {}).get("ratio")
        for row in rows
    ]
    fidelity = [value for value in fidelity if isinstance(value, (int, float))]
    summary["clock_fidelity"] = {
        "min": min(fidelity) if fidelity else None,
        "mean": statistics.fmean(fidelity) if fidelity else None,
        "n": len(fidelity),
    }
    taus = [
        (row.get("step_response") or {}).get("tau_s")
        for row in passed
    ]
    taus = [value for value in taus if isinstance(value, (int, float))]
    summary["tau_s"] = {
        "mean": statistics.fmean(taus) if taus else None,
        "spread": (max(taus) - min(taus)) if len(taus) > 1 else 0.0,
    }
    return summary


def main() -> int:
    args = _parser().parse_args()
    levels = [int(piece) for piece in args.levels.split(",") if piece.strip()]
    root = WORKTREE / ".sitl-runs" / f"plant-conc-{time.strftime('%Y%m%d-%H%M%S')}"
    root.mkdir(parents=True)
    summaries = []
    for level in levels:
        print(f"=== concurrency {level} ===", flush=True)
        payload = run_level(args.python.resolve(), root, level, args)
        summary = _summarise(payload)
        summaries.append(summary)
        print(json.dumps(summary, indent=2), flush=True)
        (root / "summary.json").write_text(
            json.dumps({"levels": summaries}, indent=2), encoding="utf-8"
        )
        # A level where the clock already failed makes every higher level a
        # waste of ten minutes and of everyone else's CPU on this machine.
        worst = summary["clock_fidelity"]["min"]
        if worst is None or worst < 0.97:
            print(
                f"STOP: clock fidelity {worst} at concurrency {level}; "
                "higher levels would only be worse.",
                file=sys.stderr,
            )
            break
    print(f"\nARTIFACT {root}")
    print(f"\n{'N':>4}{'wall_s':>9}{'pass':>7}{'pitch':>9}{'gamma':>9}"
          f"{'spread_g':>10}{'fidelity':>10}")
    for row in summaries:
        gamma = row["gamma_deg"]
        print(
            f"{row['level']:>4}{row['wall_s']:>9.0f}"
            f"{row['passed']}/{row['runs']:<5}"
            f"{(row['act_pitch_deg']['mean'] or float('nan')):>9.2f}"
            f"{(gamma['mean'] or float('nan')):>9.2f}"
            f"{gamma['spread']:>10.3f}"
            f"{(row['clock_fidelity']['min'] or float('nan')):>10.4f}"
        )
    return 0 if all(row["passed"] == row["runs"] for row in summaries) else 1


if __name__ == "__main__":
    raise SystemExit(main())
