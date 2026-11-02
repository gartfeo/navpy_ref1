"""Step 8's development gate: a seed sweep of the certified scenario in lockstep.

``deterministic_dev_sim.md``, sections 3 and 4. Every run is one
``eval_gcs_navigation_demo.py --lockstep`` of a (profile, seed), in its own
subprocess, eval-band slot and WSL root, at background priority. A sweep runs
the regression seeds (``eval_seed_sweep_seeds.json``) and the new seeds:
``--seeds``, or ``--count`` seeds drawn from the git commit, so each change
meets new seeds and a seed plus its commit reproduces a run.

Each run is classed (``eval_seed_sweep_classes``) and each failing seed is
rerun; the first record is kept, a rerun only adds to it:

- a product failure is rerun once: the same digest *confirms* it (``product``,
  or ``known`` when a known todo explains every error). A run an unflushed
  end (``unclean_end``, ``stop_budget``) ended whose rerun replays its journal
  up to that tick unchanged and goes past it, passing, is ``not_repeated``: an
  outside cause is likely (a saturated host, todo 202), though the journal
  cannot rule out a divergence at the fault tick itself, so it is reported
  apart and not passed. If that rerun still fails, it is judged as a first run
  instead, once. Any other different digest is a ``determinism_bug``, which is
  worse;
- a host fault or setup error is rerun once. If the rerun faults the same way
  with the same digest it is ``repeated_fault``: a fault that repeats is not
  shown to be the host's, so stop and look (todo 202); otherwise ``host``,
  reported apart. A rerun that passes or fails is then judged as a first run.

A reader whose attitude lag passed ``READER_LAG_LIMIT_MS`` in any run is a
``reader_lag`` finding (todos 199, 200). The sweep passes (exit 0) when no
seed is ``product``, ``determinism_bug`` or ``repeated_fault``, no reader lags
and every regression seed passes; a seed left ``host``, ``not_repeated`` or ``unconfirmed``
makes it incomplete (exit 2). ``sweep.json`` holds every run's record.
"""

from __future__ import annotations

import argparse
from collections.abc import Callable, Sequence
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import subprocess
import sys
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
for directory in (ROOT, ROOT / "src"):
    if str(directory) not in sys.path:
        sys.path.insert(0, str(directory))

from gcs.backend.cpu_share_background import apply_background
from navpy.sim.lockstep.sim_profile import DEFAULT_PROFILE, profile_names
from scripts.eval_seed_sweep_classes import HOST, PASS, PRODUCT, SETUP, RunVerdict, classify, replays_until

SEEDS_FILE = ROOT / "scripts" / "eval_seed_sweep_seeds.json"
SWEEP_FILE = "sweep.json"
#: New seeds are drawn from 1..SEED_RANGE: small enough to type, wide enough not to repeat soon.
SEED_RANGE = 1_000_000
#: Seed outcomes that fail the sweep, and those that leave it incomplete.
FAILING = ("product", "determinism_bug", "repeated_fault")
INCOMPLETE = ("host", "not_repeated", "unconfirmed")
#: The ends a product failure may have from an outside cause (todo 202); any other fault is the product's.
UNFLUSHED_ENDS = ("unclean_end", "stop_budget")


@dataclass(frozen=True)
class Job:
    profile: str
    seed: int
    regression: bool = False

    @property
    def name(self) -> str:
        return f"{self.profile}-{self.seed}"


#: Runs one attempt of a job in a new directory; returns the eval's exit code.
Runner = Callable[[Job, Path], int]


def new_seeds(commit: str, count: int) -> list[int]:
    """``count`` distinct seeds drawn from the commit: the same commit draws the same seeds."""
    seeds: list[int] = []
    i = 0
    while len(seeds) < count:
        seed = 1 + int(hashlib.sha256(f"{commit}:{i}".encode()).hexdigest(), 16) % SEED_RANGE
        if seed not in seeds:
            seeds.append(seed)
        i += 1
    return seeds


def regression_jobs(path: Path = SEEDS_FILE) -> list[Job]:
    data = json.loads(path.read_text(encoding="utf-8"))
    return [Job(entry["profile"], int(entry["seed"]), True) for entry in data["regression"]]


def unique_jobs(jobs: Sequence[Job]) -> list[Job]:
    """Each (profile, seed) once, as its first job: a new seed that is also a regression seed runs once, as
    a regression seed (both would write the same run directory)."""
    seen: set[tuple[str, int]] = set()
    unique = []
    for job in jobs:
        if (job.profile, job.seed) not in seen:
            seen.add((job.profile, job.seed))
            unique.append(job)
    return unique


def run_eval(job: Job, directory: Path) -> int:
    """The real :data:`Runner`: one CLI run; its console output goes to ``<dir>.out`` beside it."""
    command = [sys.executable, str(ROOT / "scripts" / "eval_gcs_navigation_demo.py"), "--lockstep",
               "--seed", str(job.seed), "--sim-profile", job.profile, "--run-dir", str(directory)]
    with directory.with_name(directory.name + ".out").open("w", encoding="utf-8") as out:
        return subprocess.run(command, stdout=out, stderr=subprocess.STDOUT, check=False).returncode


def _same_fault(first: RunVerdict, again: RunVerdict) -> bool:
    return (first.digest is not None and first.digest == again.digest
            and first.fault is not None and first.fault == again.fault)


def _fault_not_repeated(first: RunVerdict, first_dir: Path, again_dir: Path) -> bool:
    """A run an unflushed end ended, whose rerun replays its journal up to that tick and goes past it (so
    it did not end there): likely an outside cause (a process killed, a write the OS refused; todo 202).
    A real product fault repeats with the digest."""
    fault = first.fault or {}
    unflushed = fault.get("fault") == "ParticipantFault" and fault.get("kind") in UNFLUSHED_ENDS
    tick = fault.get("tick") if unflushed else None
    return isinstance(tick, int) and replays_until(first_dir, again_dir, tick)


def settle(job: Job, base: Path, runner: Runner) -> dict[str, Any]:
    """Run the job, rerun as the module docstring says, and give its outcome with every attempt."""
    attempts: list[tuple[str, RunVerdict]] = []

    def attempt() -> RunVerdict:
        name = job.name if not attempts else f"{job.name}-r{len(attempts)}"
        verdict = classify(base / name, runner(job, base / name))
        attempts.append((name, verdict))
        return verdict

    def confirm(first: RunVerdict, retry: bool) -> tuple[str, RunVerdict]:
        """Rerun a product failure. A rerun past an outside-caused end that still fails is itself judged
        as a first run, once, so a failure the end hid is not lost; a second outside end leaves it
        unconfirmed."""
        again = attempt()
        if again.cls in (HOST, SETUP):
            return "unconfirmed", first
        if again.cls == PRODUCT and again.digest is not None and again.digest == first.digest:
            return ("known" if first.known else "product"), first
        if _fault_not_repeated(first, base / attempts[-2][0], base / attempts[-1][0]):
            if again.cls == PASS:
                return "not_repeated", again
            return confirm(again, False) if retry else ("unconfirmed", again)
        return "determinism_bug", first

    verdict = attempt()
    status = "pass"
    if verdict.cls in (HOST, SETUP):
        again = attempt()
        if again.cls in (HOST, SETUP):
            status = "repeated_fault" if _same_fault(verdict, again) else "host"
        verdict = again
    if verdict.cls == PRODUCT:
        status, verdict = confirm(verdict, True)
    lagging = sorted({f"{name}: {reader}" for name, v in attempts for reader in v.lagging})
    return {"profile": job.profile, "seed": job.seed, "regression": job.regression, "status": status,
            "known": list(verdict.known) if status == "known" else [], "lagging_readers": lagging,
            "attempts": [{"run": name, **v.record()} for name, v in attempts]}


def verdict(seeds: Sequence[dict[str, Any]]) -> tuple[int, list[str]]:
    """The sweep's exit code (0 passed, 1 failed, 2 incomplete) and its reasons."""
    failed = [f"{s['profile']} {s['seed']}: {s['status']}" for s in seeds if s["status"] in FAILING]
    failed += [f"{s['profile']} {s['seed']}: regression seed did not pass ({s['status']})"
               for s in seeds if s["regression"] and s["status"] not in ("pass", *FAILING)]
    failed += [f"{s['profile']} {s['seed']}: reader lag {', '.join(s['lagging_readers'])}"
               for s in seeds if s["lagging_readers"]]
    incomplete = [f"{s['profile']} {s['seed']}: {s['status']}" for s in seeds
                  if s["status"] in INCOMPLETE and not s["regression"]]
    return (1 if failed else 2 if incomplete else 0), failed + incomplete


def sweep(jobs: Sequence[Job], base: Path, runner: Runner, parallel: int) -> list[dict[str, Any]]:
    with ThreadPoolExecutor(max_workers=parallel) as pool:
        return list(pool.map(lambda job: settle(job, base, runner), jobs))


def _commit() -> str:
    head = subprocess.run(["git", "-C", str(ROOT), "rev-parse", "HEAD"], capture_output=True, text=True,
                          check=True).stdout.strip()
    dirty = subprocess.run(["git", "-C", str(ROOT), "status", "--porcelain", "--untracked-files=no"],
                           capture_output=True, text=True, check=True).stdout.strip()
    return head + ("+dirty" if dirty else "")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run-dir", type=Path, required=True, help="new directory; each run in <run-dir>/<profile>-<seed>")
    parser.add_argument("--profile", default=DEFAULT_PROFILE, choices=profile_names(),
                        help=f"the new seeds' sim profile (default: {DEFAULT_PROFILE})")
    parser.add_argument("--count", type=int, default=10, help="new seeds drawn from the commit (default: 10)")
    parser.add_argument("--seeds", type=int, nargs="+", help="these new seeds instead of --count")
    parser.add_argument("--no-regression", action="store_true", help="leave out the regression seeds")
    parser.add_argument("--parallel", type=int, default=4, help="runs at once (default: 4)")
    args = parser.parse_args(argv)
    base = args.run_dir.resolve()
    if base.exists() and any(base.iterdir()):
        parser.error(f"run directory is not empty: {base}")
    if args.parallel < 1 or args.count < 0:
        parser.error("--parallel must be at least 1 and --count not negative")
    commit = _commit()
    seeds = args.seeds if args.seeds else new_seeds(commit, args.count)
    jobs = ([] if args.no_regression else regression_jobs()) + [Job(args.profile, s) for s in seeds]
    jobs = unique_jobs(jobs)
    base.mkdir(parents=True, exist_ok=True)
    apply_background()  # the runs inherit it (bounded_cpu_share.md)
    print(f"sweep at {commit}: {[job.name for job in jobs]}", flush=True)
    results = sweep(jobs, base, run_eval, args.parallel)
    code, reasons = verdict(results)
    record = {"commit": commit, "profile": args.profile, "new_seeds": seeds, "passed": code == 0,
              "exit_code": code, "reasons": reasons, "seeds": results}
    (base / SWEEP_FILE).write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")
    for result in results:
        digests = sorted({a["digest"] for a in result["attempts"] if a["digest"]})
        print(f"  {result['profile']} {result['seed']}: {result['status']}"
              + (f" (todo {', '.join(result['known'])})" if result["known"] else "")
              + f", digests {[d[:8] for d in digests]}", flush=True)
    print(f"sweep {('PASSED', 'FAILED', 'INCOMPLETE')[code]}: {base / SWEEP_FILE}")
    for reason in reasons:
        print(f"  {reason}")
    return code


if __name__ == "__main__":
    raise SystemExit(main())
