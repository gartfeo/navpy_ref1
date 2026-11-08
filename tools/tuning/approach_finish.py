"""The approach runner's evidence after every stop (hitl_guidance.md section 9, steps 13 and 27).

The board's DataFlash logs go where the link-health gate reads them; the manifest names every file of the run with its
digest and producers; the offline timing-validity check (tools/hitl/check_integration_timing.py ``check``) runs on the
run's own files against the run's frozen constants. None of these reads the scenario's private files or the contact
service's evidence content: the series report (tools/hitl/approach_report.py) scores those.
"""
from __future__ import annotations

import json
import shutil
import sys
from pathlib import Path

from tools.hitl import check_integration_timing, run_manifest
from tools.hitl.timing_check import OBSERVATION_S
from tools.tuning import approach_chain as chain, approach_contact as contact, approach_scenario as scenario_run
from tools.tuning.approach_procs import STOP_SIGINT, ProcessGroup, ProcSpec
from tools.tuning.hitl_procs import LINK_CSV

HOST_LOAD_ROLE = "host_load"
HOST_LOAD_CSV, HOST_LOAD_JSON = "host_load.csv", "host_load.json"
TIMING_FILE = "timing_check.json"
NO_CONSTANTS = "no timing constants: a calibration-only run proves no timing validity"


def host_load(paths: chain.ChainPaths) -> ProcSpec:
    """The host-load sampler (tools/hitl/host_load.py) for the whole run, on the receiving host (WSL)."""
    from tools.hitl.host_load import READY_LINE
    return ProcSpec(HOST_LOAD_ROLE, chain.tool(paths, "tools.hitl.host_load", "--out-csv",
                                               str(paths.run_dir / HOST_LOAD_CSV), "--out-json",
                                               str(paths.run_dir / HOST_LOAD_JSON)),
                    "^" + READY_LINE, 10.0, STOP_SIGINT, stop_wait_s=10.0, cwd=str(paths.repo))


def copy_logs(run_dir: Path, board_workdir: Path) -> None:
    """The backing's DataFlash logs into the board folder, where the link-health gate reads them."""
    target = board_workdir / "logs"
    target.mkdir(parents=True, exist_ok=True)
    for log in sorted((run_dir / "backing" / "logs").glob("*.BIN")):
        shutil.copy2(log, target / log.name)


# Evidence beside the timing roles (run_manifest.FILE_PRODUCERS): the manifest digests them under other_files, and the
# series report requires each of them there.
OTHER_EVIDENCE = ("router_writes.csv", "router_stats.json", "navpy.out", f"board/{LINK_CSV}", contact.EVIDENCE_FILE,
                  HOST_LOAD_JSON, HOST_LOAD_CSV, "publisher_profile.json")
BOARD_LOGS = "board/logs/"              # the DataFlash logs the link-health gate read: at least one


def timing_files(run_dir: Path, record_path: Path) -> list[tuple[str, Path]]:
    """The timing check's inputs by manifest role."""
    publisher = run_dir / "publisher"
    return [("run_record", record_path), ("navpy_integration_log", run_dir / "navpy_integration.jsonl"),
            ("publisher_status", publisher / "publisher.json"),
            *(("publisher_frames", path) for path in sorted(publisher.glob("frames-*.csv"))),
            ("unreal_log", run_dir / "unreal.log"), ("emulator_log", run_dir / "emulator.jsonl"),
            ("media_fault_log", run_dir / chain.FAULT_LOG)]


def manifest(run_dir: Path, run_id: str, record_path: Path, group: ProcessGroup) -> Path:
    files = timing_files(run_dir, record_path)
    producers = {**group.producers(), "runner": {"pid": None, "argv": sys.argv, "ended_after_stop": True}}
    built = run_manifest.build(run_id, producers, [(role, path) for role, path in files if path.exists()])
    # The scenario's private files hold the dock's truth: the runner leaves them unread (scoring checks them).
    named = {path.resolve() for _, path in files}
    private = scenario_run.private_folder(run_dir).resolve()
    built["other_files"] = [{"name": str(path.relative_to(run_dir)), **run_manifest.digest(path)}
                            for path in sorted(run_dir.rglob("*")) if path.is_file() and path.resolve() not in named
                            and private not in path.resolve().parents]
    out = run_dir / "run_manifest.json"
    out.write_text(json.dumps(built, indent=2) + "\n", encoding="utf-8")
    return out


def timing(run_dir: Path, record_path: Path, manifest_path: Path, constants: str | None, observation: bool) -> dict:
    """The offline timing-validity check of this run (its result file in the run folder). A calibration-only run
    (no constants) and an unreadable input fail; ``observation`` applies the 10-minute coverage rules."""
    if not constants:
        return {"passed": False, "failures": [NO_CONSTANTS]}
    out = run_dir / TIMING_FILE
    argv = ["check", "--run-record", str(record_path), "--navpy", str(run_dir / "navpy_integration.jsonl"),
            "--publisher", *(str(p) for p in sorted((run_dir / "publisher").glob("frames-*.csv"))),
            "--unreal", str(run_dir / "unreal.log"), "--emulator", str(run_dir / "emulator.jsonl"),
            "--constants", constants, "--manifest", str(manifest_path), "--out", str(out),
            *(("--observation",) if observation else ())]
    try:
        code = check_integration_timing.main(argv)
    except SystemExit as exc:          # argparse: no publisher frames at all
        code = exc.code
    if not out.exists():
        return {"passed": False, "failures": [f"the timing check could not read its inputs (exit {code})"]}
    return json.loads(out.read_text(encoding="utf-8"))


def observation_run(scenario: dict, command: bool) -> bool:
    """An observation run long enough for the 10-minute coverage rules (the simple checks' observation stream)."""
    return not command and float(scenario["limits"]["observe_s"]) >= OBSERVATION_S


def param_digests(files: list) -> list[dict]:
    """The board's parameter files as flown (path and content digest), for the series' frozen configuration."""
    return [{"path": str(f.path), "sha256": run_manifest.digest(Path(f.path))["sha256"] if Path(f.path).is_file()
             else None} for f in files]


def host_load_summary(run_dir: Path) -> dict:
    path = run_dir / HOST_LOAD_JSON
    if not path.exists():
        return {"ok": False, "problems": ["no host load summary"]}
    return json.loads(path.read_text(encoding="utf-8"))


__all__ = ["BOARD_LOGS", "HOST_LOAD_ROLE", "NO_CONSTANTS", "OTHER_EVIDENCE", "param_digests", "TIMING_FILE", "copy_logs", "host_load",
           "host_load_summary", "manifest", "observation_run", "timing", "timing_files"]
