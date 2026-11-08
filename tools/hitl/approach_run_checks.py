"""One approach run's score checks for the series report (hitl_final_approach.md section 9, step 27).

Each check reads the run folder after every process stopped and returns its problems; a run counts only with none.
Missing or over-budget evidence is a problem, never a skip. Truth (the board centre, the plant's path) is read only by
the truth cross-check (approach_truth.py), as scoring.
"""
from __future__ import annotations

import json
from collections.abc import Callable
from pathlib import Path
from typing import Any, Optional

from tools.hitl import run_manifest
from tools.hitl.approach_truth import cross_check
from tools.hitl.approach_video import video_check, video_numbers
from tools.hitl.contact_scene import read_dock
from tools.hitl.json_checks import RecordError
from tools.tuning import approach_chain as chain, approach_contact as contact, approach_finish as finish
from tools.tuning import approach_scenario as scenario_run
from tools.tuning.approach_quiesce import ENDED_BY_TERMINAL
from tools.tuning.approach_verdict import problems as run_problems

DOCKED = "docked"                       # FinalApproachTerminal.DOCKED's value in navpy's terminal record
# navpy's own acquisition from the encoded stream: the real ChArUco detector on the HITL profile, with integration
# timing and the contact link. Any other detector or profile (sim, ideal_360, raw frames) cannot substitute.
NAVPY_REQUIRED = (("--detector-type", "real"), ("--detector-backend", "charuco"),
                  ("--vision-profile", chain.VISION_PROFILE))
NAVPY_FLAGS = ("--integration-timing", "--hitl-contact")
SUBSTITUTE_WORDS = ("ideal", "raw-frame", "raw_frame", "truth", "sim-detector")


def _json(path: Path) -> Any:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _argv(run: dict, role: str) -> list[str]:
    return [str(word) for word in ((run.get("producers") or {}).get(role) or {}).get("argv") or []]


def _value(argv: list[str], flag: str) -> Optional[str]:
    return argv[argv.index(flag) + 1] if flag in argv and argv.index(flag) + 1 < len(argv) else None


def _values(argv: list[str], flag: str) -> list[str]:
    return [argv[i + 1] for i, word in enumerate(argv[:-1]) if word == flag]


def verdict_check(run: dict, run_dir: Path) -> list[str]:
    return run_problems(run, run.get("error"))


def command_check(run: dict, run_dir: Path) -> list[str]:
    """navpy commanded the aircraft: the runner asked for it, the router ran navpy as a client and the audit agreed."""
    found = [] if run.get("authority") == "command" else [f"authority {run.get('authority')}, not command"]
    router, navpy = _argv(run, "router"), f"navpy={chain.NAVPY_PORT}"
    if navpy not in _values(router, "--client") or navpy in _values(router, "--reader"):
        found.append("the router did not run navpy as a client")
    if (run.get("observation") or {}).get("authority") != "command":
        found.append("the write audit was not a command audit")
    return found


def docked_check(run: dict, run_dir: Path) -> list[str]:
    outcome = run.get("outcome") or {}
    if outcome.get("outcome") != DOCKED or outcome.get("ended_by") != ENDED_BY_TERMINAL:
        return [f"outcome {outcome.get('outcome')} ended by {outcome.get('ended_by')}, not docked by navpy's record"]
    return []


def contact_events(run_dir: Path) -> list[dict]:
    path = run_dir / contact.EVIDENCE_FILE
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def contact_check(run: dict, run_dir: Path) -> list[str]:
    """The service sent exactly one contact, for an arm it saw, and that arm is the episode navpy's DOCKED record
    names (task, dock uid and episode: navpy's own identity, echoed by the service)."""
    try:
        events = contact_events(run_dir)
    except (OSError, ValueError) as exc:
        return [f"contact evidence unreadable: {exc}"]
    contacts = [e for e in events if e.get("event") == "contact"]
    arms = {e.get("arm_id"): e for e in events if e.get("event") == "arm"}
    found = [] if len(contacts) == 1 else [f"{len(contacts)} contacts sent, not 1"]
    arm = arms.get(contacts[0].get("arm_id")) if contacts else None
    if contacts and arm is None:
        found.append("the contact names no arm the service saw")
    record = (run.get("outcome") or {}).get("navpy_record") or {}
    terminal = (record.get("task"), record.get("dock"), record.get("episode"))
    if terminal[2] is None:
        found.append("navpy's terminal record names no episode")
    elif arm is not None and (arm.get("task_id"), arm.get("dock_uid"), arm.get("episode")) != terminal:
        found.append(f"the contact's arm {(arm.get('task_id'), arm.get('dock_uid'), arm.get('episode'))} is not "
                     f"navpy's terminal episode {terminal}")
    return found


def truth_check(run: dict, run_dir: Path) -> list[str]:
    try:
        contacts = [e for e in contact_events(run_dir) if e.get("event") == "contact"]
        dock = read_dock(run_dir / scenario_run.SCENARIO_DIR)
    except (OSError, ValueError, RecordError) as exc:
        return [f"truth inputs unreadable: {exc}"]
    when = contacts[0]["board_time_us"] * 1e-6 if len(contacts) == 1 else None
    result = cross_check(run_dir / "board" / "link.csv", dock, float(chain.UNREAL_ALT_OFFSET_M), when)
    run.setdefault("_report", {})["truth"] = result
    return result["problems"]


def timing_check(run: dict, run_dir: Path) -> list[str]:
    timing = run.get("timing") or {"failures": ["no timing check"]}
    return [] if timing.get("passed") is True else [f"timing: {f}" for f in timing.get("failures") or ["failed"]]


def evidence_check(run: dict, run_dir: Path) -> list[str]:
    """The manifest names every evidence file, each still as digested, and its producers all ended after the stop."""
    try:
        manifest = _json(run_dir / "run_manifest.json")
    except (OSError, ValueError) as exc:
        return [f"manifest unreadable: {exc}"]
    found = []
    roles = {entry.get("role") for entry in manifest.get("files", [])}
    for role in sorted(set(run_manifest.FILE_PRODUCERS) - run_manifest.OPTIONAL_ROLES - roles):
        found.append(f"manifest has no {role}")
    named = {str(entry.get("name")).replace("\\", "/"): entry for entry in manifest.get("other_files", [])}
    for name, entry in named.items():                 # every digested file still as it was
        path = run_dir / name
        if not path.is_file() or run_manifest.digest(path)["sha256"] != entry.get("sha256"):
            found.append(f"{name} is missing or changed since the manifest")
    if not any(name.startswith(finish.BOARD_LOGS) and name.endswith(".BIN") for name in named):
        found.append("the manifest names no board DataFlash log")
    for entry in manifest.get("files", []):
        path = next((p for p in run_dir.rglob(entry.get("name", "?"))), None)
        if path is None or run_manifest.digest(path)["sha256"] != entry.get("sha256"):
            found.append(f"{entry.get('role')} {entry.get('name')} is missing or changed")
    for name in finish.OTHER_EVIDENCE:
        if name not in named:
            found.append(f"{name} is missing from the manifest")
    return found


def source_check(run: dict, run_dir: Path) -> list[str]:
    """Encoded video and navpy's own acquisition throughout: the run's navpy is the HITL build on the encoded stream,
    with no commissioning fault and no ideal, raw-frame or truth input; the scene's sources were single."""
    argv = _argv(run, "navpy")
    found = [f"navpy {flag} {_value(argv, flag)}, not {want}" for flag, want in NAVPY_REQUIRED
             if _value(argv, flag) != want]
    found += [f"navpy ran without {flag}" for flag in NAVPY_FLAGS if flag not in argv]
    found += [f"navpy option {word!r} names a substitute" for word in argv      # options only: paths may say anything
              if word.startswith("-") and any(bad in word.lower() for bad in SUBSTITUTE_WORDS)]
    if run.get("admission_problems"):
        found += [f"not admitted: {problem}" for problem in run["admission_problems"]]
    try:
        record = _json(Path(run.get("record") or run_dir / "approach_record.json"))
    except (OSError, ValueError) as exc:
        return found + [f"run record unreadable: {exc}"]
    if record.get("integration_timing", {}).get("source_id") != chain_source_id():
        found.append("the record's video source is not the HITL encoded stream")
    if not (run.get("sources") or {}).get("ok"):
        found.append("the scene's pose or gimbal sources were not single")
    return found


def chain_source_id() -> str:
    from tools.hitl.orbit_plan import load_profiles
    profiles, _, _ = load_profiles()
    return profiles[chain.VISION_PROFILE]["detector"]["camera_index"]


def geometry_check(run: dict, run_dir: Path) -> list[str]:
    return [] if (run.get("orbit_check") or {}).get("feasible") is True else ["the orbit check did not pass"]


def host_check(run: dict, run_dir: Path) -> list[str]:
    load = run.get("host_load") or {}
    return [] if load.get("ok") is True else [f"host load: {load}"]


CHECKS: dict[str, Callable[[dict, Path], list[str]]] = {
    "verdict": verdict_check, "command": command_check, "docked": docked_check, "contact": contact_check,
    "truth": truth_check, "timing": timing_check, "evidence": evidence_check, "source": source_check,
    "geometry": geometry_check, "video": video_check, "host": host_check}


def score_run(run_dir: Path) -> dict:
    """Every check of one run folder; ``passed`` only when each found nothing."""
    try:
        run = _json(run_dir / "approach_run.json")
    except (OSError, ValueError) as exc:
        return {"run_dir": str(run_dir), "passed": False, "checks": {"run": [f"approach_run.json: {exc}"]}}
    checks = {}
    for name, check in CHECKS.items():
        try:
            checks[name] = check(run, run_dir)
        except (KeyError, TypeError, ValueError, OSError) as exc:      # malformed evidence fails, never skips
            checks[name] = [f"{type(exc).__name__}: {exc}"]
    return {"run_dir": str(run_dir), "run_id": run.get("run_id"), "seed_index": (run.get("scenario") or {})
            .get("seed_index"), "passed": not any(checks.values()), "checks": checks,
            "truth": (run.get("_report") or {}).get("truth"), "timing": _skew(run.get("timing") or {}),
            "video": video_numbers(run_dir), "record": run.get("record")}


# The held-out 10-minute observation stream the simple checks ask for (same frozen configuration as the series): it
# must pass these, as an observation run whose timing check applied the coverage rules.
OBSERVATION_CHECKS = ("verdict", "timing", "evidence", "source", "geometry", "video", "host")


def score_observation(run_dir: Path) -> dict:
    """The observation run's checks; ``passed`` only for a passing 10-minute observation (step 12's coverage)."""
    try:
        run = _json(run_dir / "approach_run.json")
    except (OSError, ValueError) as exc:
        return {"run_dir": str(run_dir), "passed": False, "checks": {"run": [f"approach_run.json: {exc}"]}}
    checks = {}
    for name in OBSERVATION_CHECKS:
        try:
            checks[name] = CHECKS[name](run, run_dir)
        except (KeyError, TypeError, ValueError, OSError) as exc:
            checks[name] = [f"{type(exc).__name__}: {exc}"]
    covered = run.get("authority") == "observation" and (run.get("timing") or {}).get("observation") is True
    checks["observation"] = [] if covered else ["not an observation run checked under the 10-minute coverage rules"]
    return {"run_dir": str(run_dir), "run_id": run.get("run_id"), "passed": not any(checks.values()),
            "checks": checks, "timing": _skew(run.get("timing") or {}), "video": video_numbers(run_dir)}


def _skew(timing: dict) -> dict:
    return {name: {key: (timing.get(name) or {}).get(key) for key in ("median_s", "p95_abs_s", "max_abs_s",
                                                                        "uncertainty_s")}
            for name in ("attitude", "gimbal")}


__all__ = ["CHECKS", "DOCKED", "OBSERVATION_CHECKS", "score_observation", "score_run"]
