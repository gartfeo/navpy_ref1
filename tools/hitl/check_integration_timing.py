"""Offline timing validity of explicit integration timing (hitl_final_approach.md section 9, stage 1 step 12).

    python -m tools.hitl.check_integration_timing fit   --run-record R.json --navpy N.jsonl --publisher F.csv ...
        --unreal U.log --emulator E.jsonl --out constants.json
    python -m tools.hitl.check_integration_timing check --run-record R.json --navpy N.jsonl --publisher F.csv ...
        --unreal U.log --emulator E.jsonl --constants constants.json [--observation] --out report.json
    either with --manifest M.json (the runner's run_manifest.py; --observation needs it): every input must be a
        file of that run, from its one set of producers, ending cleanly

One receiver, one stream, one run per call. Logs: timing_inputs.py; joins: timing_joins.py and timing_run.py;
``fit`` (calibration-only run, each source's receiver-local correction C = Ds - Dc): timing_fit.py; ``check``
(held-out run, constants frozen in its run record; median/p95/max skew against the 8 ms and 10x bounds):
timing_check.py. Missing, ambiguous or over-budget joins fail.

Exit: 0 pass, 1 fail, 2 unreadable input.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

from navpy.core.perception.camera.integration_timing import IntegrationTimingConfig, IntegrationTimingLimits
from tools.hitl import approach_records, run_manifest
from tools.hitl.timing_check import OBSERVATION_S, SKEW_BUDGET_S, check
from tools.hitl.timing_fit import FIT_SCHEMA, MIN_PAIRS, fit
from tools.hitl.timing_inputs import (
    TimingInputError,
    json_record,
    read_emulator_log,
    read_navpy_log,
    read_publisher_frames,
    read_unreal_slots,
)
from tools.hitl.timing_joins import JoinError
from tools.hitl.timing_run import Joined, join_run


def _config(record: dict[str, Any]) -> IntegrationTimingConfig:
    section = record["integration_timing"]
    return IntegrationTimingConfig(
        run_id=record["run"]["run_id"], mode=record["run"]["mode"], source_id=section["source_id"],
        link_id=section["link_id"], limits=IntegrationTimingLimits(**section["limits"]),
        attitude_correction_s=section["attitude_correction_s"], gimbal_correction_s=section["gimbal_correction_s"])


def _read_lines(path: Path) -> list[str]:
    return path.read_text(encoding="utf-8", errors="strict").splitlines()


def load(args: argparse.Namespace) -> Joined:
    config = _config(approach_records.read_record(Path(args.run_record)))
    log = read_navpy_log(_read_lines(Path(args.navpy)), args.navpy)
    publisher = read_publisher_frames(Path(path) for path in args.publisher)
    slots = read_unreal_slots(_read_lines(Path(args.unreal)), args.unreal)
    emulator = read_emulator_log(_read_lines(Path(args.emulator)), args.emulator)
    return join_run(config, log, publisher, slots, emulator)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="python -m tools.hitl.check_integration_timing",
                                     description=__doc__.split("\n")[0])
    commands = parser.add_subparsers(dest="command", required=True)
    for name in ("fit", "check"):
        command = commands.add_parser(name)
        command.add_argument("--run-record", required=True, help="the run's record (tools/hitl/approach_records.py)")
        command.add_argument("--navpy", required=True, help="navpy's integration log (JSONL), one receiver")
        command.add_argument("--publisher", required=True, nargs="+", help="the publisher's frames-NNN.csv, in order")
        command.add_argument("--unreal", required=True, help="Unreal's log with NavPyCaptureSlot lines")
        command.add_argument("--emulator", required=True, help="the ZR10 emulator's JSONL log")
        command.add_argument("--out", required=True, help="where the JSON result goes")
        if name == "check":
            command.add_argument("--constants", required=True, help="the fit's JSON")
            command.add_argument("--observation", action="store_true",
                                 help="the held-out 10-minute observation's coverage rules (needs --manifest)")
        command.add_argument("--manifest", help="the run's manifest (tools/hitl/run_manifest.py)")
    return parser


def manifest_failures(args: argparse.Namespace, joined: Joined) -> list[str]:
    """Why the inputs are not one whole run of the manifest (empty without --manifest)."""
    if not args.manifest:
        return []
    manifest = json_record(Path(args.manifest).read_text(encoding="utf-8"), args.manifest)
    if not isinstance(manifest, dict):
        raise TimingInputError(f"{args.manifest}: not a JSON object")
    inputs = [("run_record", Path(args.run_record)), ("navpy_integration_log", Path(args.navpy)),
              ("publisher_status", Path(args.publisher[0]).parent / "publisher.json"),
              *(("publisher_frames", Path(path)) for path in args.publisher),
              ("unreal_log", Path(args.unreal)), ("emulator_log", Path(args.emulator))]
    return run_manifest.verify(manifest, joined.config.run_id, inputs, joined.log.run_ids)


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if getattr(args, "observation", False) and not args.manifest:
        parser.error("--observation needs --manifest")
    try:
        if args.command == "check":
            constants = json_record(Path(args.constants).read_text(encoding="utf-8"), args.constants)
            if not isinstance(constants, dict) or constants.get("schema") != FIT_SCHEMA or not constants.get("passed"):
                raise TimingInputError(f"{args.constants}: not a passing {FIT_SCHEMA}")
        joined = load(args)
        unproven = manifest_failures(args, joined)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        if isinstance(exc, JoinError):
            result = {"passed": False, "failures": [str(exc)]}
            Path(args.out).write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
            print(f"FAIL {exc}", flush=True)
            return 1
        print(f"input error: {exc}", file=sys.stderr, flush=True)
        return 2
    try:
        result = fit(joined) if args.command == "fit" else check(joined, constants, args.observation)
    except JoinError as exc:
        result = {"passed": False, "failures": [str(exc)]}
    if unproven:
        result = {**result, "passed": False, "failures": [*result["failures"], *unproven]}
    Path(args.out).write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(("PASS" if result["passed"] else "FAIL") + f" {args.command}: " + "; ".join(result["failures"]), flush=True)
    return 0 if result["passed"] else 1


__all__ = ["MIN_PAIRS", "OBSERVATION_S", "SKEW_BUDGET_S", "check", "fit", "join_run", "main"]


if __name__ == "__main__":
    sys.exit(main())
