"""Replay a lockstep run from its action file, the script off (``deterministic_dev_sim.md`` step 12).

    python scripts/eval_gcs_demo_lockstep_replay.py SOURCE_RUN_DIR --run-dir NEW_DIR [--stop-at N] [--pace]

A bug report is the commit, the run's ``input_manifest.json`` and its
``lockstep_actions.jsonl``. The replay builds the same run from the
manifest's options (seed, sim profile, operator budgets; ``--scenario`` must
be the scenario the run flew) and feeds the action file through the
coordinator (``navpy.sim.lockstep.replay``): no relay, no browser, no app.
It runs as fast as the PC allows; ``--pace`` paces it to 1x, which changes
no digest.

``--stop-at N`` ends the replay right after tick N's reply: its journal ends
with the stop record (``coordinator_stop``), and ``lockstep_snapshot.json``
holds that state with what only the replay saw: the vehicles as the app
would show them, and the requests still open.

``replay_report.json`` says whether the replay matched: the replay digests
(the journal without the script's ``operator`` notes) of the run and of the
replay, up to tick N when stopped, the first record that differs, whether
the admitted actions equal the file, and whether the inputs (the manifest
without its ``source`` part) equal the run's. Exit 0 only when all match.
No gate scores a replay: with the script off it has no approvals.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Sequence

_ROOT = Path(__file__).resolve().parents[1]
for _directory in (_ROOT, _ROOT / "src"):
    if str(_directory) not in sys.path:
        sys.path.insert(0, str(_directory))

from gcs.backend.lockstep.app_refusals import refusal
from navpy.sim.lockstep.coordinator_admission import Admission
from navpy.sim.lockstep.coordinator_operator import OperatorSide
from navpy.sim.lockstep.coordinator_pacing import RealTimePacer
from navpy.sim.lockstep.part_wire_codec import dumps
from navpy.sim.lockstep.participant_hooks import RequestIds
from navpy.sim.lockstep.replay import (
    ActionFile,
    ActionReplay,
    ReplayOperator,
    ReplayRelay,
    first_difference,
    load_actions,
    read_journal,
    replay_digest,
    same_actions,
)
from scripts.eval_delivery_fixture import load_eval_scenario, scenario_path
from scripts.eval_gcs_demo_inputs import require_recorded_scenario
from scripts.eval_gcs_demo_lockstep import JOURNAL_FILE, LockstepOptions, execute_lockstep_run
from scripts.eval_gcs_demo_lockstep_app import ACTIONS_FILE
from scripts.eval_gcs_demo_lockstep_wiring import fleet_layout
from scripts.eval_gcs_demo_models import RegressionError
from scripts.eval_gcs_demo_scenario import CERTIFIED_SCENARIO_PATH
from scripts.lockstep_app_wire import TelemetryState

MANIFEST_FILE = "input_manifest.json"
REPORT_FILE = "replay_report.json"
SNAPSHOT_FILE = "lockstep_snapshot.json"


class ReplayDriver:
    """The replay's operator side: what ``execute_lockstep_run`` takes instead of the script."""

    def __init__(self, file: ActionFile, backend: int, *, pace: bool, stop_at: int | None) -> None:
        self.file = file
        self.ids = RequestIds(0)
        self.feed = ActionReplay(file)
        self.operator = ReplayOperator(self.ids, self.feed)
        self.telemetry = TelemetryState()
        self.relay = ReplayRelay(backend, self.feed, lambda data: self.telemetry.observe(json.loads(data)))
        admission = Admission(self.ids, self.relay, refusal)
        self.side = OperatorSide(admission, RealTimePacer() if pace else OperatorSide().pace, stop_at)

    def open_requests(self) -> dict[str, list[list[int]]]:
        return {"script": [list(request_id) for request_id in self.operator.open],
                "relay": [list(request_id) for request_id in self.relay.open()]}


def source_options(source: Path, pace: bool) -> LockstepOptions:
    run = json.loads((source / MANIFEST_FILE).read_text(encoding="utf-8"))["run_options"]
    if run.get("lockstep") is not True:
        raise RegressionError(f"{source} is not a lockstep run")
    return LockstepOptions(run["seed"], run["timeout_s"], run["confirmation_delay_s"],
                           sim_profile=run["sim_profile"]["profile"], pace=pace)


def prefix(records: Sequence[dict[str, Any]], stop_at: int | None) -> list[dict[str, Any]]:
    """The run's records up to and including tick ``stop_at``'s (all of them for None)."""
    if stop_at is None:
        return list(records)
    last = max((index for index, record in enumerate(records) if record.get("k") == stop_at
                and "phase" in record), default=None)
    if last is None:
        raise RegressionError(f"the run has no tick {stop_at}")
    return list(records[:last + 1])


def _inputs(path: Path) -> dict[str, Any]:
    manifest = json.loads(path.read_text(encoding="utf-8"))
    return {key: value for key, value in manifest.items() if key != "source"}


def verdict(source: Path, run_dir: Path, driver: ReplayDriver, stop_at: int | None) -> dict[str, Any]:
    run = prefix(read_journal(source / JOURNAL_FILE), stop_at)
    records = read_journal(run_dir / JOURNAL_FILE)
    replay = records[:-1] if stop_at is not None and records and "stopped_at" in records[-1] else records
    difference = first_difference(run, replay)
    fed = driver.file if stop_at is None else ActionFile(
        tuple(a for a in driver.file.actions if a["k"] <= stop_at), None)
    left = driver.feed.left()
    report = {
        "stop_at": stop_at,
        "stopped": len(replay) < len(records),
        "run_digest": replay_digest(run),
        "replay_digest": replay_digest(replay),
        "first_difference": None if difference is None else {
            "index": difference, "k": (run[difference] if difference < len(run) else
                                       replay[difference]).get("k")},
        "actions_equal": same_actions(fed, driver.side.admission.actions),
        "actions_left": left if stop_at is None else None,
        "inputs_equal": _inputs(source / MANIFEST_FILE) == _inputs(run_dir / MANIFEST_FILE),
    }
    report["matched"] = (report["run_digest"] == report["replay_digest"] and report["actions_equal"]
                         and report["inputs_equal"] and (stop_at is None or report["stopped"])
                         and (stop_at is not None or not any(left.values())))
    return report


def write_snapshot(run_dir: Path, driver: ReplayDriver) -> None:
    records = read_journal(run_dir / JOURNAL_FILE)
    stop, tick = records[-1], records[-2]
    telemetry = driver.telemetry.snapshot()
    (run_dir / SNAPSHOT_FILE).write_bytes(dumps({
        "tick": stop["stopped_at"], "phase": stop["phase"], "t_us": tick["t_us"],
        "replay_digest": replay_digest(records[:-1]), **stop["snapshot"],
        "telemetry": None if telemetry is None else json.loads(telemetry),
        "open_requests": driver.open_requests(),
    }) + b"\n")


def parse_args(argv: Sequence[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Replay a lockstep run from its action file")
    parser.add_argument("source", type=Path, help="the run directory to replay")
    parser.add_argument("--run-dir", type=Path, required=True, help="the replay's (new) directory")
    parser.add_argument("--stop-at", type=int, default=None, metavar="N",
                        help="end after tick N, with lockstep_snapshot.json")
    parser.add_argument("--pace", action="store_true", help="pace the replay to 1x wall time")
    parser.add_argument("--scenario", type=scenario_path, default=CERTIFIED_SCENARIO_PATH,
                        help="the scenario the run flew (default: the certified three-UAV scenario)")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    if args.stop_at is not None and args.stop_at < 1:
        raise RegressionError("--stop-at must be a tick, 1 or more")
    if args.run_dir.exists() and any(args.run_dir.iterdir()):  # so all compared is this replay's
        raise RegressionError(f"replay directory is not empty: {args.run_dir}")
    scenario = load_eval_scenario(args.scenario)
    require_recorded_scenario(args.source, scenario)
    options = source_options(args.source, args.pace)
    driver = ReplayDriver(load_actions(args.source / ACTIONS_FILE), fleet_layout(scenario).backend,
                          pace=args.pace, stop_at=args.stop_at)
    print(f"lockstep replay of {args.source.resolve()}: {args.run_dir.resolve()}", flush=True)
    try:
        run = execute_lockstep_run(args.run_dir, scenario, options, driver)
        print(f"replay outcome {run.outcome!r}, journal digest {run.digest}", flush=True)
    except RegressionError as error:  # a run that faulted replays its fault: still compared
        print(f"replay ended in a fault: {error}", flush=True)
    if args.stop_at is not None and (args.run_dir / JOURNAL_FILE).exists():
        records = read_journal(args.run_dir / JOURNAL_FILE)
        if records and "stopped_at" in records[-1]:
            write_snapshot(args.run_dir, driver)
    report = verdict(args.source, args.run_dir, driver, args.stop_at)
    (args.run_dir / REPORT_FILE).write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2), flush=True)
    return 0 if report["matched"] else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except RegressionError as error:
        print(f"REPLAY FAILED: {error}", file=sys.stderr)
        raise SystemExit(2) from error
