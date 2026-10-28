"""Step 11's repeatable gate: a 5-run matrix of the certified scenario in lockstep.

Every cell is a seed-1 ``eval_gcs_navigation_demo.py --lockstep`` run of the
certified 3-UAV scenario, one at a time, each in its own subprocess, eval-band
slot and WSL root: two plain runs, two with seeded host delays (relays,
coordinator, participants and the firmware's helper jitter) and one under CPU
load, all with one sim profile (``--sim-profile``, default ``hw``). The gate passes only when every run exits 0 with ``report.json`` passed,
every journal digest is equal, and every run's artifacts (``*_host*`` and BIN
excluded) are byte-equal to the first run's. ``matrix.json`` records each run's
digest, gate results and wall cost per tick.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import os
from pathlib import Path
import re
import subprocess
import sys
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
for directory in (ROOT, ROOT / "src"):
    if str(directory) not in sys.path:
        sys.path.insert(0, str(directory))

from gcs.backend.cpu_share_background import apply_background
from navpy.sim.lockstep.sim_profile import DEFAULT_PROFILE, profile_names
from scripts.eval_gcs_demo_lockstep import JOURNAL_FILE, RUN_HOST_FILE
from scripts.gcs_demo_artifacts import compare_inventories, without_host_only
from scripts.gcs_demo_comparison import _inventory

DIGEST_FILE = "lockstep_journal_digest.txt"
MATRIX_FILE = "matrix.json"
SEED = 1


@dataclass(frozen=True)
class Cell:
    name: str
    host_delay_seed: int  # 0 is none; nonzero also seeds the firmware helper jitter
    cpu_load: bool  # one busy process per logical CPU for the whole run


CELLS = (
    Cell("plain-a", 0, False),
    Cell("plain-b", 0, False),
    Cell("delay-7", 7, False),
    Cell("delay-13", 13, False),
    Cell("cpuload", 0, True),
)


def cost(run_host: dict[str, Any], tick_us: int) -> dict[str, float]:
    """Wall time per executed tick, and sim time over wall time for the run."""
    ticks = run_host["result"]["parked_tick"]
    wall_s = run_host["wall_s"]
    return {"ticks": ticks, "wall_s": round(wall_s, 3),
            "wall_ms_per_tick": round(wall_s * 1000 / ticks, 2),
            "sim_over_wall": round(ticks * tick_us / 1e6 / wall_s, 2)}


def read_run(run_dir: Path, exit_code: int) -> dict[str, Any]:
    """One run's gate evidence; a missing file is recorded, not raised."""
    record: dict[str, Any] = {"exit_code": exit_code}
    try:
        digest = (run_dir / DIGEST_FILE).read_text(encoding="utf-8").strip()
        if not re.fullmatch(r"[0-9a-f]{64}", digest):
            raise ValueError(f"{DIGEST_FILE} is not a sha256: {digest!r}")
        record["digest"] = digest
        report = json.loads((run_dir / "report.json").read_text(encoding="utf-8"))
        record["report"] = {
            "passed": report["passed"], "errors": report["errors"],
            "approvals": len(report["approvals"]),
            "global_assignments": report["global_assignments"],
            "vehicles": [{"sys_id": v["sys_id"], "role": v["role"], "passed": v["passed"],
                          "errors": v["errors"]} for v in report["vehicles"]],
        }
        grid = json.loads((run_dir / JOURNAL_FILE).open(encoding="utf-8").readline())["grid"]
        run_host = json.loads((run_dir / RUN_HOST_FILE).read_text(encoding="utf-8"))
        record["outcome"] = run_host["result"]
        record["cost"] = cost(run_host, grid["drain_period_us"])
    except (OSError, ValueError, KeyError, TypeError, ZeroDivisionError) as error:
        record["evidence_error"] = f"{type(error).__name__}: {error}"
    return record


def inventory(run_dir: Path) -> dict[str, str]:
    """The run's compared files (``*_host*`` and BIN excluded) and their sha256."""
    return without_host_only(_inventory(run_dir))


def verdict(runs: dict[str, dict[str, Any]],
            inventories: dict[str, dict[str, str]]) -> list[str]:
    """Every reason the matrix fails; empty when it passes."""
    errors = []
    for name, run in runs.items():
        if run["exit_code"] != 0:
            errors.append(f"{name}: exit code {run['exit_code']}")
        if "evidence_error" in run:
            errors.append(f"{name}: {run['evidence_error']}")
        elif not run["report"]["passed"]:
            errors.append(f"{name}: report not passed: {run['report']['errors']}")
    digests = {run.get("digest") for run in runs.values()}
    if len(digests) != 1 or None in digests:
        errors.append(f"journal digests differ or are missing: { {n: r.get('digest') for n, r in runs.items()} }")
    names = list(inventories)
    reference = names[0] if names else None
    for name in names[1:]:
        comparison = compare_inventories(inventories[reference], inventories[name])
        if comparison["status"] != "equal":
            errors.append(f"{name}: artifacts differ from {reference}: "
                          f"{comparison['differing_files'] + comparison['only_in_reference'] + comparison['only_in_repeat']}")
    return errors


def run_cell(cell: Cell, base: Path, sim_profile: str = DEFAULT_PROFILE) -> int:
    """One CLI run; its console output goes to ``<cell>.out`` beside the run."""
    command = [sys.executable, str(ROOT / "scripts" / "eval_gcs_navigation_demo.py"), "--lockstep",
               "--seed", str(SEED), "--host-delay-seed", str(cell.host_delay_seed),
               "--sim-profile", sim_profile, "--run-dir", str(base / cell.name)]
    burners: list[subprocess.Popen] = []
    try:
        # Started one by one inside the try, so a failed start stops those already running.
        for _ in range((os.cpu_count() or 1) if cell.cpu_load else 0):
            burners.append(subprocess.Popen([sys.executable, "-c", "while True: pass"]))
        with (base / f"{cell.name}.out").open("w", encoding="utf-8") as out:
            return subprocess.run(command, stdout=out, stderr=subprocess.STDOUT,
                                  check=False).returncode
    finally:
        for burner in burners:
            burner.kill()
        for burner in burners:
            burner.wait()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run-dir", type=Path, required=True,
                        help="new directory; each cell runs in <run-dir>/<cell>")
    parser.add_argument("--sim-profile", default=DEFAULT_PROFILE, choices=profile_names(),
                        help=f"the sim profile of every cell (default: {DEFAULT_PROFILE})")
    args = parser.parse_args(argv)
    base = args.run_dir.resolve()
    if base.exists() and any(base.iterdir()):
        parser.error(f"run directory is not empty: {base}")
    base.mkdir(parents=True, exist_ok=True)
    apply_background()  # the cells, their CPU load included, inherit it (bounded_cpu_share.md)
    runs, inventories = {}, {}
    for cell in CELLS:
        print(f"matrix cell {cell.name} ...", flush=True)
        runs[cell.name] = read_run(base / cell.name, run_cell(cell, base, args.sim_profile))
        runs[cell.name]["cell"] = {"host_delay_seed": cell.host_delay_seed, "cpu_load": cell.cpu_load}
        if (base / cell.name).is_dir():
            inventories[cell.name] = inventory(base / cell.name)
        print(f"  exit {runs[cell.name]['exit_code']}, digest {runs[cell.name].get('digest')}, "
              f"cost {runs[cell.name].get('cost')}", flush=True)
    errors = verdict(runs, inventories)
    first = next(iter(inventories.values()), {})
    matrix = {"seed": SEED, "sim_profile": args.sim_profile, "passed": not errors, "errors": errors,
              "compared_files": len(first), "runs": runs}
    (base / MATRIX_FILE).write_text(json.dumps(matrix, indent=2) + "\n", encoding="utf-8")
    print(f"matrix {'PASSED' if not errors else 'FAILED'}: {base / MATRIX_FILE}")
    for error in errors:
        print(f"  {error}")
    return 0 if not errors else 1


if __name__ == "__main__":
    raise SystemExit(main())
