"""Revalidate and compare complete synchronized-flight evidence, not endpoints."""

from __future__ import annotations

import argparse
import csv
from collections import Counter
from dataclasses import asdict
import hashlib
import itertools
import json
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(ROOT), str(ROOT / "src")]

from simtime_navigation_protocol import (
    ATTITUDE, COMMAND, HEADER, GUIDED, HOST_DELAYS_S, TAKEOFF_ARM, WINDOW_SIZE,
    SnapshotSequence, StepCommand,
)
from simtime_navigation_identity import source_hashes
from scripts.simtime_navigation_config import validate_evidence, validate_command


def validate_source_identity(identity_record: dict, case: Path) -> None:
    root = Path(__file__).resolve().parent.parent
    expected = dict(source_hashes(root))
    recorded = dict(identity_record["source_sha256"])
    # The offline validator is never executed by a flight. Record its current
    # hash separately so stricter validation can re-examine unchanged flights.
    validator = Path(__file__).relative_to(root).as_posix()
    expected.pop(validator)
    recorded.pop(validator, None)
    if recorded != expected:
        raise ValueError("run source identity differs from current implementation")
    if identity_record["defaults_sha256"] != hashlib.sha256((case / "navigation-defaults.parm").read_bytes()).hexdigest():
        raise ValueError("defaults identity mismatch")


def validate(case: Path, *, lifecycle_count: int = 1,
             lifecycle_dir: Path | None = None) -> tuple[dict, list[dict]]:
    if lifecycle_count not in (1, 3):
        raise ValueError("invalid lifecycle size")
    lifecycle_dir = case if lifecycle_dir is None else lifecycle_dir
    peer = json.loads((case / "peer.json").read_text())
    identity_record = json.loads((case / "identity.json").read_text())
    validate_source_identity(identity_record, case)
    parameters = validate_evidence(case, lifecycle_dir, peer, identity_record)
    with (case / "navpy-navigation.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    records = peer["records"]
    if len(rows) != WINDOW_SIZE or len(records) != WINDOW_SIZE:
        raise ValueError("incomplete synchronized window")
    sequence = SnapshotSequence()
    normalized = []
    commands = 0
    for index, (record, row) in enumerate(zip(records, rows), 1):
        snapshot = sequence.accept(bytes.fromhex(record["snapshot"]))
        reply = bytes.fromhex(record["reply"])
        computed, apply, kind, mask, *values = COMMAND.unpack_from(reply, HEADER.size)
        command = StepCommand(kind, mask, tuple(values[:4]), values[4])
        validate_command(snapshot.limits, kind, mask, values[4], parameters,
                         peer["throttle_policy"]["configured_throttle_percent"])
        if sequence.reply(command) != reply or (computed, apply) != (index, index + 1):
            raise ValueError("noncanonical command identity")
        if not 0 <= record["elapsed_s"] <= 1.0 / peer["speedup"]:
            raise ValueError("invalid wall liveness")
        if record["elapsed_s"] < record["delay_s"]:
            raise ValueError("scheduled delay not observed")
        expected_delay = HOST_DELAYS_S[(index - 1) % len(HOST_DELAYS_S)] if peer["delayed"] else 0.
        if record["delay_s"] != expected_delay:
            raise ValueError("wrong delay schedule")
        identity = snapshot.identity
        if (int(row["step"]), int(row["tick"]), int(row["source_us"])) != (
                index, identity.tick, identity.source_us):
            raise ValueError("AP/peer identity mismatch")
        if row["source_us"] != row["after_us"] or int(row["complete_us"]) < identity.source_us:
            raise ValueError("AP clock advanced while waiting")
        if index > 1:
            previous = rows[index - 2]
            if snapshot.previous_targets != (int(previous["target_roll"]), int(previous["target_pitch"])):
                raise ValueError("AP target receipt mismatch")
            if snapshot.previous_servos != tuple(int(previous[f"servo{i}"]) for i in range(16)):
                raise ValueError("AP servo receipt mismatch")
        commands += kind == ATTITUDE
        if index == 1 and kind != TAKEOFF_ARM:
            raise ValueError("missing takeoff fixture")
        if index == peer["guided_step"] and kind != GUIDED:
            raise ValueError("missing phase entry")
        state = asdict(snapshot)
        del state["identity"]["boot"]
        simulated = {key: value for key, value in row.items() if key not in ("wait_us", "wall_us")}
        normalized.append({"snapshot": state, "command": reply[HEADER.size:].hex(),
                           "navigation": record["evidence"], "control": simulated})
    if sequence.expected.kind or normalized[-1]["snapshot"]["applied_kind"] != 0:
        raise ValueError("undrained final command")
    if commands != peer["command_count"] or not commands:
        raise ValueError("navigation command count mismatch")
    log = (lifecycle_dir / "supervisor.log").read_text(encoding="utf-8", errors="replace")
    attempts = re.findall(r"instances streaming telemetry.*?\(attempt (\d+)\)", log)
    if (log.count("Starting sketch 'ArduPlane'") != lifecycle_count or attempts != ["1"]
            or "NAVPY_NAVIGATION_INVALID" in log
            or log.count("NAVPY_NAVIGATION_COMPLETE count=3000") != lifecycle_count):
        raise ValueError("invalid simulator lifecycle")
    first = normalized[0]["snapshot"]["truth"]
    phase = normalized[peer["seed_step"] - 1]["snapshot"]
    summary = {"case": str(case), "rows": len(rows), "commands": commands,
               "vehicle": sequence.previous.identity.vehicle,
               "boot": sequence.previous.identity.boot,
               "seed_step": peer["seed_step"], "camera_hz": peer["camera_hz"],
               "requested_speed": peer["speedup"], "delayed": peer["delayed"], "identity": identity_record,
               "phase_height_above_start_m": phase["truth"]["altitude"] - first["altitude"],
               "phase_airspeed_mps": phase["observation"]["airspeed_mps"],
               "first_pass_step": next((i + 1 for i, r in enumerate(records) if r["evidence"].get("passed")), None),
               "measured_speed": (int(rows[-1]["source_us"]) - int(rows[0]["source_us"])) /
                    (int(rows[-1]["wall_us"]) - int(rows[0]["wall_us"])),
               "max_receipt_s": max(r["elapsed_s"] for r in records),
               "sha256": {name: hashlib.sha256(((lifecycle_dir if name.endswith('.log') else case) / name).read_bytes()).hexdigest()
                          for name in ("peer.json", "navpy-navigation.csv", "supervisor.log", "teardown.log")}}
    return summary, normalized


def compare(cases: list[Path], *, speeds: tuple[float, ...] = (1., 10.),
            rates: tuple[int, ...] = (40,), repetitions: int = 2) -> dict:
    if len({case.resolve() for case in cases}) != len(cases):
        raise ValueError("matrix requires distinct case directories")
    checked = [validate(case) for case in cases]
    actual = Counter((summary["camera_hz"], summary["requested_speed"], summary["delayed"])
                     for summary, _ in checked)
    expected = Counter({(rate, speed, delayed): repetitions
                        for rate, speed, delayed in itertools.product(rates, speeds, (False, True))})
    if len(checked) < 2 or actual != expected:
        raise ValueError(f"incomplete/extra matrix cells: expected {expected}, observed {actual}")
    if len({tuple(summary["boot"]) for summary, _ in checked}) != len(checked):
        raise ValueError("matrix requires distinct simulator boot identities")
    if any(summary["identity"] != checked[0][0]["identity"] for summary, _ in checked):
        raise ValueError("matrix mixes implementation identity")
    pairs = []
    for (left, a), (right, b) in itertools.combinations(checked, 2):
        if left["camera_hz"] != right["camera_hz"]:
            continue
        divergence = next((i + 1 for i, (x, y) in enumerate(zip(a, b)) if x != y), None)
        fields = [] if divergence is None else [key for key in a[divergence-1] if a[divergence-1][key] != b[divergence-1][key]]
        pairs.append({"left": left["case"], "right": right["case"],
                      "equal": divergence is None, "first_divergence": divergence, "fields": fields})
    if not pairs:
        raise ValueError("no comparisons")
    identity_record = checked[0][0]["identity"]
    return {"identity": identity_record,
            "validator_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "expected_matrix": {"speeds": speeds, "rates": rates, "repetitions": repetitions,
                                "delayed": [False, True]},
            "runs": [{k: v for k, v in summary.items() if k != "identity"} for summary, _ in checked],
            "comparisons": pairs}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("cases", type=Path, nargs="+")
    parser.add_argument("--speeds", type=float, nargs="+", default=[1., 10.])
    parser.add_argument("--rates", type=int, nargs="+", default=[40])
    parser.add_argument("--repetitions", type=int, default=2)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = compare(args.cases, speeds=tuple(args.speeds), rates=tuple(args.rates), repetitions=args.repetitions)
    if args.output:
        args.output.write_text(json.dumps(report, indent=2))
    print(json.dumps({"runs": len(report["runs"]), "comparisons": len(report["comparisons"]),
                      "equal": sum(pair["equal"] for pair in report["comparisons"]),
                      "failures": [p for p in report["comparisons"] if not p["equal"]]}, indent=2))
    if any(not pair["equal"] for pair in report["comparisons"]):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
