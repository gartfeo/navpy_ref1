"""The approach run's verdict (approach.py): exit 0 only when every check in the run's record passed."""
from __future__ import annotations

from pathlib import Path

from tools.tuning import approach_owners as owners, approach_scenario as scenario_run


def verdict(run: dict, run_id: str, run_dir: Path, failed: str | None) -> int:
    """Print the run's summary and return its exit code (0: the flight, every stop, the scene, the link health, no
    orphans, the observation audit, one owner each and the airspeed source all passed)."""
    health = run["board"].get("link_health") or {}
    ended = all(item.get("ended_after_stop") for item in run["producers"].values())
    scene = scenario_run.reproduced(run["scene"], run["scenario"])
    owned, owner_problems = owners.verdicts(run)
    airspeed_problems = run["airspeed"]["problems"]
    print(f"[approach] {run_id}: error {failed}, orphans {run['orphans']}, all ended after stop {ended}, "
          f"scene reproduced {scene}, link health {health.get('ok')}, outcome {run['outcome']['outcome']}, "
          f"observation {run['observation']}, one owner each {owned}; folder {run_dir}"
          + "".join(f"; {problem}" for problem in run["admission_problems"] + owner_problems + airspeed_problems),
          flush=True)
    clean = run["observation"]["ok"] and not run["outcome"]["problems"] and not airspeed_problems and owned
    return 0 if failed is None and not run["orphans"] and ended and scene and health.get("ok") and clean else 1
