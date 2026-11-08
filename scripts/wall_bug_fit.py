"""Dev sim step 9: fit a sim profile from wall logs, then search seeds for the wall's failure.

``docs/design/deterministic_dev_sim.md``, section 3 (B). Two commands:

- ``fit RUN_DIR... [--model process_freeze|reader_lag]`` reads wall runs
  (``wall_bug_logs``): each UAV's NavPy stalls in aircraft time and its
  failures. ``process_freeze`` (the default) is fitted to the stalls, pooled over the runs and UAVs: lengths from :data:`STALL_MIN_MS` up to
  the longest stall below the bound the wall's failures depend on
  (:func:`tail_bound_ms`: the command source-age bound for a stale command,
  else the observation-gap gate), and past it as far as gives the stalls at or
  over that bound, which can fail the run alone, at least the wall's share; the
  gap between freezes averages the wall's time between stalls. Uniform lengths
  only approximate the wall's: the share and the mean length come out higher.
  ``reader_lag`` is fitted to the slow lags (``Stall.slow``) at or over the
  tail bound, the ones a reader that kept receiving made (todo 65): their
  peaks, and their rises and drains scaled from the stall threshold to zero as
  the model's linear slopes give (:func:`lag_fit`); one comes every NAV
  window's seconds per slow lag, all of the wall's having come in NAV. The
  model applies that rate over the whole run, a stress assumption. The rest is the wall sim's setup on ``quiet``'s ranges: no line rate,
  no camera latency (the wall sim camera had none), thread order always on.
  Wind, loss and faults stay off, as on the wall. It prints the profile entry
  for ``sim_profiles.json`` and writes ``fit.json`` with the evidence.
- ``search --profile NAME --wall RUN_DIR...`` runs seeds of a profile in
  lockstep, as the seed sweep does (``eval_seed_sweep``: background priority,
  a failing seed rerun for its digest), until one fails, confirmed, with a
  failure kind the wall runs failed with (``wall_bug_logs.failure_kinds``),
  read from the confirmed run. That seed
  reproduces the wall bug. Without one in the budget it is an unreproduced bug
  (section 3).
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from collections.abc import Sequence
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
for directory in (ROOT, ROOT / "src"):
    if str(directory) not in sys.path:
        sys.path.insert(0, str(directory))

from gcs.backend.cpu_share_background import apply_background
from navpy.core.approach.vision_nav.command_freshness import GUIDANCE_COMMAND_MAX_SOURCE_AGE_S
from navpy.sim.lockstep.sim_profile import load_profile, profile_names
from scripts.eval_seed_sweep import Job, Runner, run_eval, settle
from scripts.wall_bug_logs import (
    GAP_GATE_MS,
    STALL_MIN_MS,
    Stall,
    failure_kinds,
    nav_windows,
    span_s,
    stalls,
    uav_arrivals,
    wall_failures,
)

TICK_MS = 20
#: A command's source may be this old (``command_freshness``); a longer stall in NAV fails it stale.
SOURCE_AGE_MS = round(1000 * GUIDANCE_COMMAND_MAX_SOURCE_AGE_S)
#: A UAV in the certified scenario.
UAVS = (1, 2, 3)
MODELS = ("process_freeze", "reader_lag")
FIT_FILE = "fit.json"
SEARCH_FILE = "search.json"
#: Seeds tried by default before a bug is called unreproduced (section 3: "say 200 seeds").
DEFAULT_BUDGET = 200


def tail_bound_ms(kinds: set[str]) -> int:
    """The stall length that alone can cause the wall's failures: a stale command needs a stall past
    its source-age bound; the rest (no detection, a roll step, a gap) start at the gap gate."""
    return SOURCE_AGE_MS if "source_command_stale" in kinds else GAP_GATE_MS


def freeze_fit(found: Sequence[Stall], seconds: float, tail_ms: int) -> dict[str, float | int]:
    """``process_freeze``'s parameters from stalls over ``seconds`` of NavPy time (module docstring).

    Uniform integer lengths from ``lo`` to ``hi`` ticks put a share ``(hi - g + 1) / (hi - lo + 1)`` at
    or over the tail bound's ``g`` ticks; ``hi`` is the smallest that reaches at least the wall's share, at
    least the longest stall below ``g`` and at most the longest stall."""
    if not found or seconds <= 0:
        raise ValueError("no stall to fit")
    lo, gate = STALL_MIN_MS // TICK_MS, math.ceil(tail_ms / TICK_MS)
    share = sum(s.length_ms >= tail_ms for s in found) / len(found)
    longest = max(math.ceil(s.length_ms / TICK_MS) for s in found)
    hi = min(longest, gate - 1)
    while hi < longest and (hi - gate + 1) / (hi - lo + 1) < share:
        hi += 1
    mean_gap = seconds / len(found)
    return {"min_ticks": lo, "max_ticks": hi, "min_gap_s": round(mean_gap / 2, 3),
            "max_gap_s": round(mean_gap * 3 / 2, 3)}


def lag_fit(found: Sequence[Stall], nav_s: float, tail_ms: int) -> dict[str, float | int]:
    """``reader_lag``'s parameters from the slow lags at or over the tail bound in ``nav_s`` seconds of
    NAV (module docstring); one the log ended inside is left out, as its drain was not seen. A stall is measured from the threshold up, so a linear slope seen over
    ``t`` up to a peak ``p`` above it is ``t * p / (p - STALL_MIN_MS)`` long from zero."""
    slow = [s for s in found if s.slow and s.length_ms >= tail_ms and not s.open_at_end]
    if not slow or nav_s <= 0:
        raise ValueError("no slow lag to fit")

    def scaled(s: Stall, seen_ms: int) -> float:
        return seen_ms * s.length_ms / (s.length_ms - STALL_MIN_MS) / 1000

    peaks = [math.ceil(s.length_ms / TICK_MS) for s in slow]
    rises = [scaled(s, s.rise_ms) for s in slow]
    drains = [scaled(s, s.drain_ms) for s in slow]
    mean_gap = nav_s / len(slow)
    return {"min_peak_ticks": min(peaks), "max_peak_ticks": max(peaks),
            "min_rise_s": round(min(rises), 3), "max_rise_s": round(max(rises), 3),
            "min_drain_s": round(min(drains), 3), "max_drain_s": round(max(drains), 3),
            "min_gap_s": round(mean_gap / 2, 3), "max_gap_s": round(mean_gap * 3 / 2, 3)}


def fitted_profile(model: str, params: dict[str, float | int]) -> dict[str, Any]:
    quiet = load_profile("quiet").models
    camera = {name: value if not hasattr(value, "lo") else [value.lo, value.hi]
              for name, value in quiet["camera_frames"].params.items()}
    return {"models": {
        "baud_limit": {"p_on": 0.0},
        "camera_frames": {"p_on": 0.0, "params": camera},
        model: {"p_on": 1.0, "params": params},
        "sensor_noise": {"p_on": 1.0, "params": {"gps_noise_m": [0.0, 1.0]}},
        "thread_order": {"p_on": 1.0},
        "timers_first": {"p_on": quiet["timers_first"].p_on},
    }}


def fit(runs: Sequence[Path], model: str = "process_freeze") -> dict[str, Any]:
    found: list[Stall] = []
    seconds = nav_s = 0.0
    evidence = []
    for run in runs:
        for uav in UAVS:
            pairs = uav_arrivals(run, uav)
            mine = [s for pair in pairs for s in stalls(pair)]
            covered = sum(span_s(pair) for pair in pairs)
            windows = nav_windows(run, uav, pairs)
            found += mine
            seconds += covered
            nav_s += sum(end - start for start, end in windows)
            lengths = sorted(s.length_ms for s in mine)
            evidence.append({
                "run": str(run), "uav": uav, "navpy_processes": len(pairs), "span_s": round(covered, 1),
                "stalls": len(mine), "longest_ms": lengths[-1] if lengths else None,
                "lengths_ms": lengths, "nav_windows_s": [[round(a, 1), round(b, 1)] for a, b in windows],
                "slow_lags": [s.__dict__ for s in mine if s.slow], "failures": [f.__dict__ for f in wall_failures(run, uav, pairs)]})
    kinds = {f["kind"] for e in evidence for f in e["failures"]}
    tail = tail_bound_ms(kinds)
    params = freeze_fit(found, seconds, tail) if model == "process_freeze" else lag_fit(found, nav_s, tail)
    return {"runs": [str(r) for r in runs], "stall_min_ms": STALL_MIN_MS, "tail_bound_ms": tail,
            "stalls": len(found), "at_or_over_tail": sum(s.length_ms >= tail for s in found),
            "navpy_s": round(seconds, 1), "nav_s": round(nav_s, 1), "model": model, model: params,
            "wall_kinds": sorted(kinds), "per_uav": evidence, "profile": fitted_profile(model, params)}


def run_kinds(result: dict[str, Any], base: Path) -> set[str]:
    """A confirmed seed's failure kinds: the errors and NavPys' logged NAV failures of the run its rerun
    confirmed. ``settle`` confirms a failure with its last two attempts, at one digest; earlier attempts
    (a host or setup fault, an end not repeated) are not that failure."""
    confirmed, again = result["attempts"][-2:]
    if confirmed["digest"] is None or confirmed["digest"] != again["digest"]:
        raise ValueError(f"seed {result['seed']}: no confirmed pair of runs")
    texts = list(confirmed["errors"])
    for log in sorted((base / confirmed["run"]).glob("uav_*_navigation.log")):
        texts += log.read_text(encoding="utf-8", errors="replace").splitlines()
    return failure_kinds(texts)


def search(profile: str, wall: set[str], seeds: Sequence[int], base: Path, runner: Runner,
           parallel: int) -> dict[str, Any]:
    """Seeds in batches of ``parallel`` until one reproduces; every settled seed is kept."""
    tried: list[dict[str, Any]] = []
    for at in range(0, len(seeds), parallel):
        batch = [Job(profile, seed) for seed in seeds[at:at + parallel]]
        with ThreadPoolExecutor(max_workers=parallel) as pool:
            results = list(pool.map(lambda job: settle(job, base, runner), batch))
        for result in results:
            kinds = run_kinds(result, base) if result["status"] in ("product", "known") else set()
            result["kinds"] = sorted(kinds)
            result["reproduces"] = bool(kinds & wall)
            tried.append(result)
        found = next((r for r in tried if r["reproduces"]), None)
        if found is not None:
            return {"profile": profile, "wall_kinds": sorted(wall), "reproduced": True,
                    "seed": found["seed"], "tried": tried}
    return {"profile": profile, "wall_kinds": sorted(wall), "reproduced": False, "seed": None, "tried": tried}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    commands = parser.add_subparsers(dest="command", required=True)
    fit_cmd = commands.add_parser("fit", help="fit a profile from wall runs")
    fit_cmd.add_argument("runs", type=Path, nargs="+", help="wall run directories")
    fit_cmd.add_argument("--out", type=Path, help=f"directory for {FIT_FILE}")
    fit_cmd.add_argument("--model", choices=MODELS, default=MODELS[0], help="the stall model to fit")
    search_cmd = commands.add_parser("search", help="search seeds of a profile for the wall's failure")
    search_cmd.add_argument("--profile", required=True, choices=profile_names())
    search_cmd.add_argument("--wall", type=Path, nargs="+", required=True, help="the wall runs to reproduce")
    search_cmd.add_argument("--run-dir", type=Path, required=True, help="new directory for the runs")
    search_cmd.add_argument("--first-seed", type=int, default=1)
    search_cmd.add_argument("--budget", type=int, default=DEFAULT_BUDGET, help="seeds to try at most")
    search_cmd.add_argument("--parallel", type=int, default=4)
    args = parser.parse_args(argv)
    if args.command == "fit":
        result = fit(args.runs, args.model)
        if args.out:
            args.out.mkdir(parents=True, exist_ok=True)
            (args.out / FIT_FILE).write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        print(json.dumps({k: result[k] for k in ("stalls", "at_or_over_tail", "tail_bound_ms", "navpy_s", "nav_s",
                                                 args.model, "wall_kinds")}))
        print(json.dumps(result["profile"]))
        return 0
    wall = {f.kind for run in args.wall for uav in UAVS for f in wall_failures(run, uav)}
    if not wall:
        parser.error("the wall runs logged no failure of a known kind")
    base = args.run_dir.resolve()
    if base.exists() and any(base.iterdir()):
        parser.error(f"run directory is not empty: {base}")
    if args.parallel < 1 or args.budget < 1:
        parser.error("--parallel and --budget must be at least 1")
    base.mkdir(parents=True, exist_ok=True)
    apply_background()  # the runs inherit it (bounded_cpu_share.md)
    seeds = list(range(args.first_seed, args.first_seed + args.budget))
    record = search(args.profile, wall, seeds, base, run_eval, args.parallel)
    (base / SEARCH_FILE).write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")
    for result in record["tried"]:
        print(f"  {result['profile']} {result['seed']}: {result['status']} {result['kinds']}"
              + (" REPRODUCES" if result["reproduces"] else ""))
    print(f"{'REPRODUCED by seed ' + str(record['seed']) if record['reproduced'] else 'NOT REPRODUCED'}"
          f" (wall kinds {record['wall_kinds']}): {base / SEARCH_FILE}")
    return 0 if record["reproduced"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
