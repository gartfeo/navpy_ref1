"""The failure classes of one seed-sweep run (``deterministic_dev_sim.md``, section 4, step 8).

A run is classed from what it left, never from its console text alone:

- *pass:* exit 0 and ``report.json`` passed;
- *product failure:* ``report.json`` scored and not passed, a ``ParticipantFault``
  included (the eval scores it, todo 202), or a journaled ``ParticipantFault``
  with no report;
- *host fault:* no report, and the run's host record or journal names a
  ``HostFault``, a ``CoordinatorFault`` or another exception during the run;
- *setup error:* no report and no run record: the eval failed before or
  around the run (todo 191's traceback is one).

A product failure whose every error a known todo explains is *known*; the
explanations are narrow on purpose, so a new failure is never hidden:

- todo 121 (a): ``forwarding: NAVLINK <packet> router_filtered`` where the
  journal's radio records the packet as a ``false`` emission (a frame byte
  loss made);
- todo 77: with the ``wind`` model on, a UAV that misses its dock (SNAP
  without ``DOCKED``, no reset marker, ``no_detection``), and a roll-command
  step on a UAV that also missed.
"""

from __future__ import annotations

import ast
import json
import re
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from navpy.sim.lockstep.radio_link import radio_events
from scripts.eval_gcs_demo_lockstep import JOURNAL_FILE, RUN_HOST_FILE
from scripts.eval_seed_sweep_lag import lagging_readers, reader_lag

PASS, PRODUCT, HOST, SETUP = "pass", "product_failure", "host_fault", "setup_error"
DIGEST_FILE = "lockstep_journal_digest.txt"
REPORT_FILE = "report.json"
MANIFEST_FILE = "input_manifest.json"
_FILTERED = re.compile(r"forwarding: NAVLINK (\[.*\]) router_filtered at tick \d+")
#: A missed dock as the gates word it (todo 77's mode).
_DOCK_MISS = (
    re.compile(r"guidance episode reached SNAP without DOCKED"),
    re.compile(r"missing coordinate pass reset marker"),
    re.compile(r"(NAV_FAIL|DOCK_FAILED): no_detection"),
    re.compile(r"workflow ticks: no reset marker"),
)
#: A saturated roll command, part of todo 77's mode only on a UAV that also missed (it fails without
#: wind too, todo 229).
_ROLL_STEP = re.compile(r"max roll-command step [\d.]+deg > [\d.]+deg")
_VEHICLE = re.compile(r"vehicle (\d+)")


@dataclass(frozen=True)
class RunVerdict:
    cls: str
    digest: str | None
    #: The journal's fault record (class, kind, tick, ordinal), if the run ended on one.
    fault: Mapping[str, Any] | None
    errors: Sequence[str]
    #: The todos that explain every error of a product failure; empty when any is unexplained.
    known: Sequence[str] = ()
    lag: Mapping[str, Mapping[str, float]] = field(default_factory=dict)
    lagging: Sequence[str] = ()

    def record(self) -> dict[str, Any]:
        return {"class": self.cls, "digest": self.digest, "fault": self.fault, "errors": list(self.errors),
                "known": list(self.known), "reader_lag_ms": self.lag, "lagging_readers": list(self.lagging)}


def _json(path: Path) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


def _journal(run_dir: Path) -> list[dict[str, Any]]:
    try:
        with (run_dir / JOURNAL_FILE).open(encoding="utf-8") as lines:
            return [json.loads(line) for line in lines]
    except (OSError, ValueError):
        return []


def _errors(report: Mapping[str, Any]) -> list[str]:
    return list(report.get("errors", [])) + [f"vehicle {v['sys_id']}: {e}" for v in report.get("vehicles", [])
                                             for e in v.get("errors", [])]


def _models_on(run_dir: Path) -> set[str]:
    manifest = _json(run_dir / MANIFEST_FILE) or {}
    models = manifest.get("run_options", {}).get("sim_profile", {}).get("models", {})
    return {name for name, model in models.items() if model.get("on")}


def _radio_false(journal: Sequence[Mapping[str, Any]]) -> set[tuple]:
    return {tuple(event[2]) for event in radio_events(journal) if event[0] == "false"}


def _vehicle(error: str) -> str | None:
    match = _VEHICLE.search(error)
    return match.group(1) if match else None


def _explainers(run_dir: Path, journal: Sequence[Mapping[str, Any]],
                errors: Sequence[str]) -> list[Callable[[str], str | None]]:
    false = _radio_false(journal)
    windy = "wind" in _models_on(run_dir)
    missed = {_vehicle(e) for e in errors if any(p.search(e) for p in _DOCK_MISS)} - {None}

    def radio_frame(error: str) -> str | None:
        match = _FILTERED.fullmatch(error)
        if match is None:
            return None
        try:
            packet = tuple(ast.literal_eval(match.group(1)))
        except (ValueError, SyntaxError):
            return None
        return "121(a)" if packet in false else None

    def windy_miss(error: str) -> str | None:
        if not windy:
            return None
        if any(p.search(error) for p in _DOCK_MISS):
            return "77"
        return "77" if _ROLL_STEP.search(error) and _vehicle(error) in missed else None

    return [radio_frame, windy_miss]


def known_todos(run_dir: Path, journal: Sequence[Mapping[str, Any]], errors: Sequence[str]) -> list[str]:
    """The todos that explain every error, or empty if one is unexplained (or there are none)."""
    explainers = _explainers(run_dir, journal, errors)
    todos = []
    for error in errors:
        todo = next((t for t in (explain(error) for explain in explainers) if t), None)
        if todo is None:
            return []
        todos.append(todo)
    return sorted(set(todos))


def _lines_before(run_dir: Path, tick: int) -> list[str]:
    lines = []
    try:
        with (run_dir / JOURNAL_FILE).open(encoding="utf-8") as journal:
            for line in journal:
                record = json.loads(line)
                if ("fault" in record and "kind" in record) or record.get("k", -1) >= tick:
                    break
                lines.append(line)
    except (OSError, ValueError):
        return []
    return lines


def replays_until(first: Path, again: Path, tick: int) -> bool:
    """The rerun's journal holds every record the first run journaled before ``tick``, unchanged, and a
    record of a later tick: the run is deterministic up to its fault, so a fault that did not repeat had
    an outside cause (a process killed, a write the OS refused). The fault tick itself is not compared:
    the outside cause may already have changed the first run's records there."""
    before = _lines_before(first, tick)
    if not before:
        return False
    try:
        with (again / JOURNAL_FILE).open(encoding="utf-8") as journal:
            if [journal.readline() for _ in before] != before:
                return False
            return any(json.loads(line).get("k", -1) > tick for line in journal)
    except (OSError, ValueError):
        return False


def classify(run_dir: Path, exit_code: int) -> RunVerdict:
    """One run's class from its report, host record and journal (module docstring)."""
    journal = _journal(run_dir)
    fault = next((r for r in reversed(journal) if "fault" in r and "kind" in r), None)
    digest_text = (run_dir / DIGEST_FILE).read_text(encoding="utf-8").strip() if (run_dir / DIGEST_FILE).is_file() else ""
    digest = digest_text if re.fullmatch(r"[0-9a-f]{64}", digest_text) else None
    lag = reader_lag(run_dir) if run_dir.is_dir() else {}
    report = _json(run_dir / REPORT_FILE)
    if isinstance(report, dict):
        errors = _errors(report)
        if exit_code == 0 and report.get("passed"):
            return RunVerdict(PASS, digest, fault, errors, (), lag, lagging_readers(lag))
        if exit_code == 0:  # a passed exit with a failed report is the eval's own contradiction
            errors.append("exit 0 with a report that did not pass")
        return RunVerdict(PRODUCT, digest, fault, errors, known_todos(run_dir, journal, errors), lag,
                          lagging_readers(lag))
    host = _json(run_dir / RUN_HOST_FILE)
    if fault is not None and fault.get("fault") == "ParticipantFault":  # unscored, but the product's
        return RunVerdict(PRODUCT, digest, fault, [f"ParticipantFault {fault.get('kind')} with no report"], (),
                          lag, lagging_readers(lag))
    if isinstance(host, dict) and host.get("fault"):
        return RunVerdict(HOST, digest, fault, [str(host["fault"])], (), lag, lagging_readers(lag))
    return RunVerdict(SETUP, digest, fault, [f"exit {exit_code} with no report and no run record"], (), lag,
                      lagging_readers(lag))


__all__ = ["HOST", "PASS", "PRODUCT", "RunVerdict", "SETUP", "classify", "known_todos", "replays_until"]
