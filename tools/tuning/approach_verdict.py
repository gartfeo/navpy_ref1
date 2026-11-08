"""The approach run's verdict (approach.py): exit 0 only when every check in the run's record passed. The series
report (tools/hitl/approach_report.py) reads the same checks from each run's approach_run.json."""
from __future__ import annotations

from pathlib import Path

from tools.tuning import approach_owners as owners, approach_scenario as scenario_run


def problems(run: dict, failed: str | None) -> list[str]:
    """Why the run did not pass (empty: the flight, every stop, the scene, the link health, no orphans, the write
    audit, one owner each and the airspeed source all passed). A missing section counts as failed."""
    found = [f"error: {failed}"] if failed else []
    health = (run.get("board") or {}).get("link_health") or {}
    producers = run.get("producers") or {}
    if not producers or not all(item.get("ended_after_stop") for item in producers.values()):
        found.append("not every process ended after its stop")
    if run.get("orphans") != []:
        found.append(f"orphans {run.get('orphans')}")
    if not scenario_run.reproduced(run.get("scene") or {}, run.get("scenario") or {}):
        found.append("the scene was not reproduced")
    if not health.get("ok"):
        found.append("link health failed")
    if not (run.get("observation") or {}).get("ok"):
        found.append(f"write audit failed: {run.get('observation')}")
    found += (run.get("outcome") or {"problems": ["no outcome"]})["problems"]
    found += (run.get("airspeed") or {"problems": ["no airspeed outcome"]})["problems"]
    owned, owner_problems = owners.verdicts(run)
    if not owned:
        found += owner_problems or ["owner census failed"]
    return found


def verdict(run: dict, run_id: str, run_dir: Path, failed: str | None) -> int:
    """Print the run's summary and return its exit code (0 when ``problems`` finds none)."""
    found = problems(run, failed)
    print(f"[approach] {run_id}: {'PASS' if not found else 'FAIL'}, authority {run.get('authority')}, outcome "
          f"{(run.get('outcome') or {}).get('outcome')}, write audit {run.get('observation')}, timing passed "
          f"{(run.get('timing') or {}).get('passed')}, host load ok {(run.get('host_load') or {}).get('ok')}; "
          f"folder {run_dir}" + "".join(f"; {problem}" for problem in run.get("admission_problems", []) + found),
          flush=True)
    return 0 if not found else 1
