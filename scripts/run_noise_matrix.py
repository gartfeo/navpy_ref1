"""Twelve predeclared, interleaved boots; stop on the first rejected attempt."""

import argparse
from datetime import datetime
import json
from pathlib import Path

from eval_lockstep_fleet import ROOT, run
from noise_isolation_profiles import cells
from simtime_step_launch import require_base_interpreter
from compare_lockstep_fleet import read_run
from compare_noise_matrix import verify_profile


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--firmware-root", required=True)
    parser.add_argument("--peer-python", default="/home/gart/navpy-simtime-env/bin/python")
    args = parser.parse_args()
    require_base_interpreter()
    root = ROOT / ".sitl-runs" / f"noise-matrix-{datetime.now():%Y%m%d-%H%M%S}"
    root.mkdir()
    ledger = dict(version=1, policy="one attempt per declared cell; abort on first rejection",
                  attempts=[dict(cell, directory=str(root / f"run-{i:02}"), status="pending")
                            for i, cell in enumerate(cells())])
    path = root / "attempts.json"
    def save() -> None:
        path.write_text(json.dumps(ledger, indent=2))
    save()
    print(f"LEDGER {path}", flush=True)
    references = {}
    for attempt in ledger["attempts"]:
        attempt["status"] = "running"
        save()
        options = argparse.Namespace(firmware_root=args.firmware_root, peer_python=args.peer_python,
            **{key: attempt[key] for key in ("instances", "speedup", "delayed", "noise_profile")})
        try:
            result = run(options, Path(attempt["directory"]))
            directory = Path(attempt["directory"])
            manifest, vehicles = read_run(directory, noise_profile=attempt["noise_profile"])
            vehicle = manifest["vehicles"][0]
            summary, history = vehicles[vehicle]
            verify_profile(directory / str(vehicle), manifest, history)
            profile = attempt["noise_profile"]
            if profile in references and history != references[profile]:
                raise ValueError("within-profile history diverged")
            references[profile] = history
            if "stock" in references and "noise-off" in references and references["stock"] == references["noise-off"]:
                raise ValueError("noise mask contrast had no effect")
            attempt.update(status="captured", run_id=result["run_id"])
        except BaseException as error:
            attempt.update(status="rejected", error=str(error))
            save()
            raise
        save()
        print(json.dumps(attempt), flush=True)


if __name__ == "__main__":
    main()
