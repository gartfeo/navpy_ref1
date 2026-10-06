"""Validate complete investigation boots before interpreting profile contrasts."""

import argparse
from collections import Counter
import itertools
import json
from pathlib import Path
import os
import subprocess

from compare_lockstep_fleet import read_run, launch_path as wsl_path
from eval_lockstep_fleet import ROOT, digest
from eval_preboot_params import parse_parm
from noise_isolation_profiles import PROFILES, cells
from noise_log_artifacts import new_log


def verify_ledger(ledger: dict) -> list[dict]:
    attempts = ledger["attempts"]
    expected = cells()
    if ledger["version"] != 1 or len(attempts) != len(expected):
        raise ValueError("incomplete noise matrix")
    if any(attempt["status"] != "captured" for attempt in attempts):
        raise ValueError("noise matrix contains rejected or uncompleted attempts")
    if any({key: attempt[key] for key in cell} != cell for attempt, cell in zip(attempts, expected)):
        raise ValueError("noise matrix differs from declared interleaved order")
    if len({str(Path(a["directory"]).resolve()) for a in attempts}) != len(attempts):
        raise ValueError("duplicate attempt directory")
    return attempts


def verify_profile(case: Path, manifest: dict, history: list[dict]) -> dict:
    expected = PROFILES[manifest["noise_profile"]]
    base = parse_parm((case / "firmware-template.parm").read_text())
    actual = parse_parm((case / "navigation-defaults.parm").read_text())
    delta = dict(expected, LOG_DISARMED=1, LOG_FILE_DSRMROT=0)
    if parse_parm((case / "noise-profile.parm").read_text()) != delta or actual != dict(base, **delta):
        raise ValueError("defaults differ beyond declared profile")
    before, after = [json.loads((case / f"logs-{when}.json").read_text()) for when in ("before", "after")]
    directory = manifest["firmware_root"] + "/" + case.name
    if before["directory"] != directory or after["directory"] != directory:
        raise ValueError("BIN belongs to a different instance")
    name = new_log(before, after)
    binding = json.loads((case / "bin-binding.json").read_text())
    if (binding["source"] != directory + "/logs/" + name or
            binding["sha256"] != digest(case / "flight.BIN") or
            (case / "flight.BIN").stat().st_size != after["entries"][name]["size"]):
        raise ValueError("BIN binding mismatch")
    times = [row["snapshot"]["identity"]["source_us"] for row in history]
    intervals = [b - a for a, b in zip(times, times[1:])]
    quantum = 1_000_000 // expected["SIM_RATE_HZ"]
    # A 50 Hz control boundary lands on adjacent integer simulator samples.
    # At 1200 Hz the integer 833 us sample yields 19992/20825 us; at 1000 Hz
    # 20000 us is exactly representable. Do not call a 24 ms loop nominal 50 Hz.
    nominal = 20_000
    allowed = {nominal // quantum * quantum, -(-nominal // quantum) * quantum}
    if not intervals or not set(intervals) <= allowed:
        raise ValueError("boot-bound tick cadence differs from simulator rate")
    identity = json.loads((case / "identity.json").read_text())
    command = json.loads((case / "peer-command.json").read_text())
    wanted = ["wsl.exe", "--exec", identity["peer_python"],
              wsl_path(ROOT / "scripts/simtime_navigation_peer.py"),
              "--defaults", wsl_path(case / "navigation-defaults.parm"), "--directory", directory,
              "--result", wsl_path(case / "peer.json"), "--pid-file", wsl_path(case / "peer.pid"),
              "--speedup", str(manifest["speedup"]), "--camera-hz", "40", "--fault", "none"]
    if manifest.get("approach_throttle_percent") is not None:
        wanted.extend(["--throttle-percent", str(manifest["approach_throttle_percent"])])
    if manifest["delayed"]:
        wanted.append("--delayed")
    if command != wanted:
        raise ValueError("unexpected peer invocation")
    prefix = ["wsl.exe", "--exec"] if os.name == "nt" else []
    parsed = subprocess.run([*prefix, identity["peer_python"],
        wsl_path(ROOT / "scripts/read_noise_parameters.py"), wsl_path(case), json.dumps(expected)],
        check=True, capture_output=True, text=True, timeout=60)
    parameters = json.loads(parsed.stdout)
    if parameters["binary_sha256"] != binding["sha256"]:
        raise ValueError("parameter reader used another BIN")
    return dict(parameters=parameters, control_intervals_us=dict(Counter(intervals)))


def compare(path: Path) -> dict:
    attempts = verify_ledger(json.loads(path.read_text()))
    entries = []
    for attempt in attempts:
        directory = Path(attempt["directory"])
        manifest, vehicles = read_run(directory, noise_profile=attempt["noise_profile"])
        if any(attempt[key] != manifest[key] for key in (
                "instances", "speedup", "delayed", "noise_profile", "run_id")):
            raise ValueError("attempt differs from captured manifest")
        vehicle = manifest["vehicles"][0]
        summary, history = vehicles[vehicle]
        case = directory / str(vehicle)
        profile = verify_profile(case, manifest, history)
        peer = json.loads((case / "peer.json").read_text())
        pinned = dict(summary["identity"])
        pinned.pop("defaults_sha256")
        pinned.update({key: manifest[key] for key in (
            "chat", "home", "camera_hz", "launcher_sha256", "wrapper_sha256", "firmware_root", "driver_sha256")})
        pinned.update(dock=peer["dock"], template_sha256=digest(case / "firmware-template.parm"))
        if entries and pinned != entries[0]["pinned"]:
            raise ValueError("cross-profile implementation or environment differs")
        entries.append(dict(directory=str(directory), profile=attempt["noise_profile"],
            run_id=manifest["run_id"], manifest_sha256=digest(directory / "fleet.json"),
            pinned=pinned, summary=summary, history=history, evidence=profile,
            guided_step=peer["guided_step"]))
    if (len({entry["run_id"] for entry in entries}) != len(entries) or
            len({tuple(entry["summary"]["boot"]) for entry in entries}) != len(entries)):
        raise ValueError("duplicate simulator boot")
    comparisons = []
    for name in PROFILES:
        group = [entry for entry in entries if entry["profile"] == name]
        for left, right in itertools.combinations(group, 2):
            step = next((i + 1 for i, (a, b) in enumerate(zip(left["history"], right["history"])) if a != b), None)
            comparisons.append(dict(profile=name, left=left["directory"], right=right["directory"],
                                    equal=step is None, first_divergence=step))
    representatives = {entry["profile"]: entry["history"] for entry in entries}
    if representatives["stock"] == representatives["noise-off"]:
        raise ValueError("noise mask contrast had no effect")
    return dict(ledger=str(path.resolve()), ledger_sha256=digest(path), validator_sha256=digest(Path(__file__)),
                control_steps=sum(entry["summary"]["rows"] for entry in entries),
                runs=[dict({key: value for key, value in entry.items() if key not in ("history", "pinned", "summary")},
                           summary={key: value for key, value in entry["summary"].items() if key != "identity"})
                      for entry in entries], comparisons=comparisons,
                identity=entries[0]["pinned"],
                limits="One fixed realization per profile. BIN binding is owned lifecycle provenance, not a boot nonce. Fresh EEPROM is enforced by prepare. Whole-boot contrasts include estimator and trajectory changes.")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ledger", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = compare(args.ledger)
    args.output.write_text(json.dumps(report, indent=2))
    failures = [row for row in report["comparisons"] if not row["equal"]]
    print(json.dumps(dict(control_steps=report["control_steps"], comparisons=len(report["comparisons"]), failures=failures)))
    if failures:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
