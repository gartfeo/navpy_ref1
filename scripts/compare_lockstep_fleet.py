"""Require independent fleet boots and compare each vehicle's complete history."""

from __future__ import annotations

import argparse
from collections import Counter
import itertools
import json
from pathlib import Path

from compare_simtime_navigation import validate
from eval_lockstep_fleet import HOME, digest, driver_identity, resolve_home
from eval_simtime_step import wsl_path
from gcs.backend.instance_ports import sysids_for_chat
from noise_isolation_profiles import PROFILES


def launch_path(path: Path) -> str:
    """Recorded launch arguments use Linux paths, even when checked in WSL."""
    if not path.is_absolute():
        raise ValueError("launch evidence requires an absolute path")
    return wsl_path(path) if path.drive else path.as_posix()


def read_run(directory: Path, *, noise_profile: str | None = None,
             pose_capture: bool | None = None, pose_source: str | None = None,
             home: str | None = None) -> tuple[dict, dict]:
    expected_home = resolve_home(home)
    manifest = json.loads((directory / "fleet.json").read_text())
    if manifest.get("noise_profile") != noise_profile:
        raise ValueError("investigation profile cannot enter the normal matrix")
    if manifest.get("pose_capture") is not pose_capture:
        raise ValueError("pose experiment cannot enter an undeclared matrix")
    if manifest.get("pose_source") != pose_source:
        raise ValueError("render pose source cannot enter an undeclared matrix")
    if pose_source is not None and (pose_capture is not True or pose_source not in ("rounded", "precast")):
        raise ValueError("invalid render pose source")
    if pose_capture is not None and (type(pose_capture) is not bool or noise_profile != "noise-off-1000"):
        raise ValueError("invalid pose experiment profile")
    count = manifest["instances"]
    if (manifest['version'] != 1 or manifest['camera_hz'] != 40 or manifest['supervisor_alive']
            or manifest["status"] != "captured" or manifest["errors"] or count not in (1, 3)
            or manifest["home"] != expected_home or manifest["template_directories"]):
        raise ValueError("unaccepted fleet lifecycle/profile")
    if home is not None and manifest.get("dock") is None:
        raise ValueError("explicit home requires an explicit dock in captured evidence")
    if manifest["driver_sha256"] != driver_identity():
        raise ValueError("fleet driver identity changed")
    vehicles = manifest["vehicles"]
    if vehicles != sysids_for_chat(manifest["chat"])[:count]:
        raise ValueError("fleet vehicle identity differs from reserved slot")
    shared = directory / str(manifest["shared_profile_vehicle"])
    if manifest["shared_profile_vehicle"] != vehicles[0]:
        raise ValueError("wrong shared profile owner")
    command = json.loads((directory / "command.json").read_text())
    expected_flags = {'--instances', '--home', '--speedup', '--dist', '--defaults',
                      '--sitl-binary', '--sitl-root', '--single-boot', '--eval'}
    if {arg for arg in command if arg.startswith('--')} != expected_flags:
        raise ValueError('unexpected launch options')
    for flag, expected in (("--instances", str(count)), ("--home", expected_home),
                           ("--speedup", str(manifest["speedup"])), ("--dist", "0"),
                           ("--sitl-root", manifest['firmware_root']),
                           ("--defaults", launch_path(shared / "navigation-defaults.parm")),
                           ("--sitl-binary", launch_path(shared / "launch-probe.sh"))):
        if command.count(flag) != 1 or command[command.index(flag) + 1] != expected:
            raise ValueError(f"launched profile differs: {flag}")
    if "--single-boot" not in command or "--eval" not in command:
        raise ValueError("wrong launch mode")
    for name, expected in manifest["evidence_sha256"].items():
        path = directory / name
        if not path.resolve().is_relative_to(directory.resolve()) or digest(path) != expected:
            raise ValueError("fleet evidence hash mismatch")
    checked = {}
    required = {"command.json", "supervisor.log", "teardown.log", "launch-unlock.json"}
    for vehicle in vehicles:
        case = directory / str(vehicle)
        required |= {f"{vehicle}/{name}" for name in (
            "peer.json", "identity.json", "navpy-navigation.csv", "peer-command.json",
            "launch-probe.sh", "navigation-defaults.parm", "flight.BIN",
            "logs-before.json", "logs-after.json", "bin-binding.json")}
        if noise_profile is not None:
            required |= {f"{vehicle}/{name}" for name in (
                "firmware-template.parm", "noise-profile.parm", "flight.BIN",
                "logs-before.json", "logs-after.json", "bin-binding.json",
                "navigation_debug.csv", "navigation_compact.csv")}
        if pose_capture:
            required.add(f"{vehicle}/navpy-pose.csv")
        elif (case / "navpy-pose.csv").exists():
            raise ValueError("unexpected pose capture")
        for name, key in (("launch-probe.sh", "wrapper_sha256"),
                          ("navigation-defaults.parm", "defaults_sha256")):
            if digest(case / name) != manifest[key]:
                raise ValueError("per-vehicle profile differs from actual shared file")
        peer_command = json.loads((case / "peer-command.json").read_text())
        if not isinstance(peer_command, list):
            raise ValueError("peer dock command differs from fleet manifest")
        dock = manifest.get("dock")
        if peer_command.count("--dock") != (1 if dock is not None else 0):
            raise ValueError("peer dock command differs from fleet manifest")
        if dock is not None:
            index = peer_command.index("--dock")
            try:
                launched_dock = [float(value) for value in peer_command[index + 1:index + 4]]
            except (TypeError, ValueError) as error:
                raise ValueError("peer dock command differs from fleet manifest") from error
            if launched_dock != dock:
                raise ValueError("peer dock command differs from fleet manifest")
        peer = json.loads((case / "peer.json").read_text())
        if dock is not None and peer.get("dock") != dock:
            raise ValueError("peer recorded dock differs from fleet manifest")
        peer_policy = peer.get("throttle_policy", {})
        if peer_policy.get("configured_throttle_percent") != manifest.get("approach_throttle_percent"):
            raise ValueError("peer throttle policy differs from requested fleet policy")
        summary, history = validate(case, lifecycle_count=count, lifecycle_dir=directory)
        if (summary["vehicle"], summary["camera_hz"], summary["requested_speed"], summary["delayed"]) != (
                vehicle, manifest["camera_hz"], manifest["speedup"], manifest["delayed"]):
            raise ValueError("trace identity/profile differs from manifest")
        checked[vehicle] = (summary, history)
    if not required <= manifest["evidence_sha256"].keys():
        raise ValueError("incomplete evidence binding")
    return manifest, checked


def compare(directories: list[Path], *, repetitions: int = 3) -> dict:
    if repetitions != 3 or len({p.resolve() for p in directories}) != len(directories):
        raise ValueError("matrix needs independent repeated runs")
    runs = [(directory, *read_run(directory)) for directory in directories]
    expected = Counter({(3, speed, delayed): repetitions
                        for speed, delayed in itertools.product((1., 10.), (False, True))})
    expected[(1, 1., False)] = 1
    actual = Counter((m["instances"], m["speedup"], m["delayed"]) for _, m, _ in runs)
    if actual != expected:
        raise ValueError(f"incomplete fleet/solo matrix: expected {expected}, observed {actual}")
    if len({m["run_id"] for _, m, _ in runs}) != len(runs):
        raise ValueError("duplicate fleet boot")
    entries = [(directory, manifest, vehicle, summary, history)
               for directory, manifest, checked in runs
               for vehicle, (summary, history) in checked.items()]
    if len({tuple(s["boot"]) for _, _, _, s, _ in entries}) != len(entries):
        raise ValueError("duplicate vehicle boot")
    reference = entries[0][3]["identity"]
    if any(s["identity"] != reference for _, _, _, s, _ in entries):
        raise ValueError("mixed flight implementation")
    if len({m["chat"] for _, m, _ in runs}) != 1:
        raise ValueError("vehicle identities changed between runs")
    for key in ('launcher_sha256', 'wrapper_sha256', 'defaults_sha256', 'firmware_root'):
        if len({m[key] for _, m, _ in runs}) != 1:
            raise ValueError(f"{key} changed between runs")
    pairs = []
    for left, right in itertools.combinations(entries, 2):
        if left[2] != right[2]:
            continue
        a, b = left[4], right[4]
        step = next((i + 1 for i, (x, y) in enumerate(zip(a, b)) if x != y), None)
        pairs.append(dict(vehicle=left[2], left=str(left[0]), right=str(right[0]),
                          equal=step is None, first_divergence=step,
                          fields=[] if step is None else [k for k in a[step-1] if a[step-1][k] != b[step-1][k]]))
    return dict(expected_repetitions=repetitions, fleet_runs=sum(m["instances"] == 3 for _, m, _ in runs),
                solo_runs=1, vehicle_runs=len(entries), control_steps=sum(s["rows"] for _, _, _, s, _ in entries),
                identity=reference, validator_sha256=digest(Path(__file__)),
                driver_sha256=runs[0][1]['driver_sha256'], launcher_sha256=runs[0][1]['launcher_sha256'],
                runs=[dict(directory=str(d), manifest_sha256=digest(d / "fleet.json"),
                           run_id=m["run_id"], instances=m["instances"], speedup=m["speedup"], delayed=m["delayed"],
                           vehicles=[{k: v for k, v in s.items() if k != "identity"} for s, _ in c.values()])
                      for d, m, c in runs], comparisons=pairs)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ledger", type=Path, nargs="?")
    parser.add_argument("--run", type=Path, help="Validate one captured run")
    parser.add_argument("--home", type=resolve_home, help="Expected explicit launch point for --run")
    parser.add_argument("--noise-profile", choices=tuple(PROFILES), help="Expected noise profile for --run")
    pose = parser.add_mutually_exclusive_group()
    pose.add_argument("--pose-capture", dest="pose_capture", action="store_true", default=None)
    pose.add_argument("--pose-control", dest="pose_capture", action="store_false")
    parser.add_argument("--pose-source", choices=("rounded", "precast"))
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.run:
        if args.ledger or args.output:
            parser.error("--run cannot be combined with ledger or --output")
        manifest, checked = read_run(args.run.resolve(), home=args.home, noise_profile=args.noise_profile,
                                     pose_capture=args.pose_capture, pose_source=args.pose_source)
        print(json.dumps(dict(run_id=manifest["run_id"], status=manifest["status"],
                              home=manifest["home"], vehicles=list(checked),
                              manifest_sha256=digest(args.run.resolve() / "fleet.json"),
                              validator_sha256=digest(Path(__file__)))))
        return
    if args.ledger is None or any(value is not None for value in
                                  (args.home, args.noise_profile, args.pose_capture, args.pose_source)):
        parser.error("a ledger is required; profile options require --run")
    report = compare_ledger(args.ledger)
    if args.output:
        args.output.write_text(json.dumps(report, indent=2))
    failures = [p for p in report["comparisons"] if not p["equal"]]
    print(json.dumps(dict(vehicle_runs=report["vehicle_runs"], control_steps=report["control_steps"],
                          comparisons=len(report["comparisons"]), failures=failures), indent=2))
    if failures:
        raise SystemExit(1)


def compare_ledger(path: Path) -> dict:
    ledger = json.loads(path.read_text())
    attempts = ledger['attempts']
    if any(a['status'] != 'captured' for a in attempts):
        raise ValueError('matrix contains rejected or uncompleted attempts')
    for attempt in attempts:
        manifest = json.loads((Path(attempt['directory']) / 'fleet.json').read_text())
        if any(attempt[k] != manifest[k] for k in ('instances', 'speedup', 'delayed', 'run_id')):
            raise ValueError('attempt ledger differs from captured run')
    report = compare([Path(a['directory']) for a in attempts], repetitions=ledger['repetitions'])
    report.update(attempts=len(attempts), rejected_attempts=0,
                  ledger=str(path), ledger_sha256=digest(path))
    return report


if __name__ == "__main__":
    main()
