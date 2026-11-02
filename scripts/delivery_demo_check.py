"""What happened in one hand demo run: the facts of its folder, in one command.

Default: the newest run under this checkout's ``.logs/delivery-demo``. This is
not the delivery gate, which judges an eval run and reports false errors on a
hand run: nothing here is a pass or a fail, and a dock outcome is a plain fact.
A missing or partial file says so and the rest still prints. To add a line, add
a section to ``RUN_SECTIONS`` (or ``UAV_SECTIONS`` in ``delivery_demo_check_uav``).
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from collections import Counter
from collections.abc import Callable
from pathlib import Path
from typing import Any

_REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
for _import_root in (_REPOSITORY_ROOT, _REPOSITORY_ROOT / "src"):
    if str(_import_root) not in sys.path:
        sys.path.insert(0, str(_import_root))

from scripts.delivery_demo import RUNS_DIR  # noqa: E402
from scripts.delivery_demo_check_uav import (  # noqa: E402
    approach_outcomes,
    debug_csv,
    guarded,
    uav_ids,
    uav_lines,
)
from scripts.eval_gcs_demo_inputs import INPUT_MANIFEST  # noqa: E402

BACKEND_LOG = Path("gcs") / "backend.log"
#: ``diagnostics`` writes each session's timeline here.
TIMELINE_GLOB = "gcs/sessions/*/timeline.jsonl"
_EMPTY_SHA256 = hashlib.sha256(b"").hexdigest()
_BACKEND_FACT_RE = re.compile(r"cpu_share_holder\]|Stack scheduling:|/launch called|BLOCKED")
_LEVEL_RE = re.compile(r" (?:\[[^\]]+\] )?(WARNING|ERROR|CRITICAL)\b")
_BAD_TIMELINE_EVENT_RE = re.compile(r"reject|timeout|fail|error", re.IGNORECASE)


def _short(digest: object) -> str:
    return digest[:12] if isinstance(digest, str) else "not recorded"


def summary_lines(run_dir: Path) -> list[str]:
    uavs = uav_ids(run_dir)
    if not uavs:
        return ["UAVs: none, no uav_N_* file in this folder"]
    results: Counter[str] = Counter()
    without: list[int] = []
    unread: list[int] = []
    for uav in uavs:
        try:
            outcomes = approach_outcomes(debug_csv(run_dir, uav), uav)
        except Exception:  # noqa: BLE001 - the UAV's own section says why
            unread.append(uav)
            continue
        results.update(outcome.result for outcome in outcomes)
        if not outcomes:
            without.append(uav)
    lines = [f"UAVs (from file names): {_ids(uavs)}"]
    told = ", ".join(f"{count} {result}" for result, count in sorted(results.items()))
    lines.append(f"Approaches: {sum(results.values())}" + (f": {told}" if told else ""))
    if without:
        lines.append(f"No approach ended in the debug log: UAV {_ids(without)}")
    if unread:
        lines.append(f"Approaches not read: UAV {_ids(unread)}, see below")
    return lines


def _ids(uavs: list[int]) -> str:
    return ", ".join(map(str, uavs))


def input_lines(run_dir: Path) -> list[str]:
    path = run_dir / INPUT_MANIFEST
    if not path.is_file():
        return [
            f"Commit and inputs: unknown, no {INPUT_MANIFEST} (a run from before it was "
            "written, or its write failed and the launch warned)"
        ]
    manifest: dict[str, Any] = json.loads(path.read_text(encoding="utf-8"))
    source = manifest.get("source", {})
    options = manifest.get("run_options", {})
    sitl = manifest.get("sitl", {})
    navpy = manifest.get("navpy", {})
    environment = manifest.get("environment", {})
    files = environment.get("files", {})
    variables = environment.get("variables", {})
    told = "; ".join(f"{key}={value}" for key, value in sorted(variables.items()))
    dialect = navpy.get("mavlink_dialect", {})
    return [
        f"Commit: {source.get('git_head', 'not recorded')}, "
        f"uncommitted changes: {_changed(source.get('uncommitted_diff_sha256'))}, "
        f"untracked files: {_changed(source.get('untracked_sha256'))}",
        f"Inputs (sha256 start): settings {_short(files.get('GCS_SETTINGS_PATH'))}, "
        f"fixture {_short(manifest.get('scenario_sha256'))}, "
        f"plan file offered {_short(options.get('offered_plan_sha256'))} "
        "(the plan actually loaded is not recorded), "
        f"vision profiles {_short(navpy.get('vision_profiles_sha256'))}",
        f"Run options: CPU share asked {options.get('cpu_share_min', 'not recorded')} min, "
        f"SITL speedup {sitl.get('speedup', 'not recorded')}, "
        f"pymavlink {dialect.get('pymavlink', 'not recorded')}",
        f"Environment (allow-listed): {told or 'none'}",
    ]


def _changed(digest: object) -> str:
    if not isinstance(digest, str):
        return "not recorded"
    return "none" if digest == _EMPTY_SHA256 else f"yes (sha256 start {digest[:12]})"


def backend_lines(run_dir: Path) -> list[str]:
    path = run_dir / BACKEND_LOG
    if not path.is_file():
        return [f"Backend: {BACKEND_LOG.as_posix()} is missing"]
    rotated = _rotated_backend_logs(path)
    lines = [
        line for part in (*rotated, path)
        for line in part.read_text(encoding="utf-8", errors="replace").splitlines()
    ]
    levels = Counter(match.group(1) for match in map(_LEVEL_RE.search, lines) if match)
    tracebacks = sum(line.startswith("Traceback") for line in lines)
    facts = [f"  {line}" for line in lines if _BACKEND_FACT_RE.search(line)]
    launched = any("/launch called" in line for line in lines)
    told = ", ".join(f"{count} {level}" for level, count in sorted(levels.items())) or "no warnings"
    # Rotation drops the oldest file, so with rotated files "never" cannot be known.
    no_launch = ", no /launch call in the text kept" if rotated else ", /launch never called"
    kept = f" (with {len(rotated)} rotated files)" if rotated else ""
    return [
        f"Backend: {len(lines)} log lines{kept}, {told}, {tracebacks} tracebacks"
        + ("" if launched else no_launch),
        *facts,
    ]


def _rotated_backend_logs(path: Path) -> list[Path]:
    """Oldest first; the highest number is the oldest."""
    numbered = (part for part in path.parent.glob(f"{path.name}.*") if part.suffix[1:].isdigit())
    return sorted(numbered, key=lambda part: int(part.suffix[1:]), reverse=True)


def timeline_lines(run_dir: Path) -> list[str]:
    paths = sorted(run_dir.glob(TIMELINE_GLOB))
    if not paths:
        return [f"Timeline: no {TIMELINE_GLOB}"]
    events: Counter[str] = Counter()
    failed_http: Counter[str] = Counter()
    unreadable = 0
    for path in paths:
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
            try:
                row = json.loads(line)
                events[row["event"]] += 1
                if row["event"] == "http_request_finished" and row.get("outcome") != "success":
                    failed_http[str(row.get("route"))] += 1
            except (ValueError, KeyError, TypeError):
                unreadable += 1
    told = [
        f"{count} {event}" for event, count in sorted(events.items())
        if _BAD_TIMELINE_EVENT_RE.search(event)
    ]
    told += [f"{count} failed {route}" for route, count in sorted(failed_http.items())]
    tail = f", {unreadable} unreadable lines" if unreadable else ""
    return [f"Timeline: {sum(events.values())} events{tail}" + (f": {', '.join(told)}" if told else "")]


#: Printed in order, each isolated from the others' failures.
RUN_SECTIONS: tuple[tuple[str, Callable[[Path], list[str]]], ...] = (
    ("Summary", summary_lines),
    ("Commit and inputs", input_lines),
    ("Backend", backend_lines),
    ("Timeline", timeline_lines),
    ("UAVs", uav_lines),
)


def report(run_dir: Path) -> str:
    lines = [
        f"Run folder: {run_dir}",
        "Facts from the logs, not a gate. DOCKED is the simulator's contact sensor; "
        "simulation never proves physical docking or cargo receipt.",
    ]
    for title, section in RUN_SECTIONS:
        lines.extend(guarded(title, section, run_dir))
    return "\n".join(lines)


def newest_run(runs_dir: Path) -> Path | None:
    """Run folders are named by their start time, so the largest name is the newest."""
    if not runs_dir.is_dir():
        return None
    return max((path for path in runs_dir.iterdir() if path.is_dir()), key=lambda path: path.name, default=None)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "run_dir", nargs="?", type=Path,
        help=f"a hand demo run folder (default: the newest under {RUNS_DIR})",
    )
    args = parser.parse_args(argv)
    run_dir = args.run_dir or newest_run(RUNS_DIR)
    if run_dir is None:
        print(f"No hand demo run folder under {RUNS_DIR}; name one.", file=sys.stderr)
        return 2
    if not run_dir.is_dir():
        print(f"Not a folder: {run_dir}", file=sys.stderr)
        return 2
    # Log text can hold anything; a Windows console's code page must not stop the report.
    print(report(run_dir).encode("ascii", "backslashreplace").decode("ascii"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
