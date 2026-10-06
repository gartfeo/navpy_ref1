"""One-command exact three-UAV ideal-vision acceptance test."""

from __future__ import annotations

import argparse
import hashlib
import json
import secrets
import subprocess
import sys
import tempfile
from dataclasses import asdict
from pathlib import Path
from typing import Sequence


ROOT = Path(__file__).resolve().parents[1]
for import_root in (ROOT, ROOT / "src"):
    path = str(import_root)
    if path not in sys.path:
        sys.path.insert(0, path)

from scripts.eval_gcs_demo_models import RegressionError, finite_number
from scripts.eval_gcs_demo_process import exclusive_evaluator_lock
from scripts.eval_ideal_three_uav_analysis import analyze_ideal_run
from scripts.eval_ideal_three_uav_runtime import execute_ideal_live_run


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Exact normal-workflow three-UAV ideal_360 final-approach regression"
        )
    )
    parser.add_argument("--analyze-only", type=Path, metavar="RUN_DIR")
    parser.add_argument("--run-dir", type=Path)
    parser.add_argument("--timeout-s", type=float, default=600.0)
    parser.add_argument("--max-snap-distance-m", type=float, default=1.0)
    return parser.parse_args(argv)


def _candidate_identity() -> dict[str, object]:
    def git(*arguments: str) -> str:
        result = subprocess.run(
            ["git", *arguments],
            cwd=ROOT,
            text=True,
            capture_output=True,
            check=False,
        )
        if result.returncode != 0:
            raise RegressionError(
                f"git {' '.join(arguments)} failed: {result.stderr.strip()}"
            )
        return result.stdout

    commit = git("rev-parse", "HEAD").strip()
    diff = git("diff", "--binary", "HEAD")
    untracked_names = sorted(
        name
        for name in git(
            "ls-files",
            "--others",
            "--exclude-standard",
            "-z",
        ).split("\0")
        if name
    )
    untracked: list[tuple[str, bytes]] = []
    root = ROOT.resolve()
    for name in untracked_names:
        path = (ROOT / name).resolve()
        if path != root and root not in path.parents:
            raise RegressionError(f"untracked candidate path escaped root: {name}")
        try:
            content = path.read_bytes()
        except OSError as error:
            raise RegressionError(
                f"could not hash untracked candidate file {name}: {error}"
            ) from error
        untracked.append((name, content))
    return {
        "commit": commit,
        "working_tree": git("status", "--short").splitlines(),
        "diff_sha256": hashlib.sha256(diff.encode("utf-8")).hexdigest(),
        "untracked_files": untracked_names,
        "candidate_sha256": _candidate_content_sha256(
            commit,
            diff.encode("utf-8"),
            untracked,
        ),
    }


def _candidate_content_sha256(
    commit: str,
    tracked_diff: bytes,
    untracked: Sequence[tuple[str, bytes]],
) -> str:
    digest = hashlib.sha256()
    for label, content in (
        ("commit", commit.encode("utf-8")),
        ("tracked-diff", tracked_diff),
    ):
        digest.update(label.encode("ascii") + b"\0")
        digest.update(len(content).to_bytes(8, "big"))
        digest.update(content)
    for name, content in sorted(untracked):
        encoded_name = name.encode("utf-8")
        digest.update(b"untracked\0")
        digest.update(len(encoded_name).to_bytes(8, "big"))
        digest.update(encoded_name)
        digest.update(len(content).to_bytes(8, "big"))
        digest.update(content)
    return digest.hexdigest()


def _default_run_dir() -> Path:
    return Path(tempfile.gettempdir()) / (
        f"navpy-ideal-three-uav-{secrets.token_hex(6)}"
    )


def _write_final(run_dir: Path, payload: dict[str, object]) -> None:
    with (run_dir / "final.json").open("x", encoding="utf-8") as stream:
        stream.write(json.dumps(payload, indent=2))


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    timeout = finite_number("--timeout-s", args.timeout_s)
    max_snap = finite_number(
        "--max-snap-distance-m",
        args.max_snap_distance_m,
    )
    if timeout <= 0.0 or max_snap <= 0.0:
        raise RegressionError("timeout and max SNAP distance must be positive")
    if args.analyze_only is not None:
        report = analyze_ideal_run(
            args.analyze_only,
            max_snap_distance_m=max_snap,
        )
        payload = {
            "status": "passed" if report.passed else "failed",
            "report": asdict(report),
        }
        print(json.dumps(payload, indent=2))
        return 0 if report.passed else 1
    run_dir = args.run_dir or _default_run_dir()
    print(f"Ideal three-UAV artifacts: {run_dir.resolve()}", flush=True)
    identity = _candidate_identity()
    try:
        run_dir.mkdir(parents=True, exist_ok=False)
    except OSError as error:
        print(json.dumps({
            "status": "failed",
            "candidate": identity,
            "setup_error": f"{type(error).__name__}: {error}",
        }, indent=2))
        return 2
    try:
        with exclusive_evaluator_lock():
            mission = execute_ideal_live_run(run_dir, timeout)
        report = analyze_ideal_run(
            run_dir,
            sys_ids=mission.sys_ids,
            approvals=mission.approvals,
            max_snap_distance_m=max_snap,
        )
        payload = {
            "status": "passed" if report.passed else "failed",
            "candidate": identity,
            "report": asdict(report),
        }
        exit_code = 0 if report.passed else 1
    except BaseException as error:
        payload = {
            "status": "failed",
            "candidate": identity,
            "setup_error": f"{type(error).__name__}: {error}",
        }
        exit_code = 2
    print(json.dumps(payload, indent=2))
    try:
        _write_final(run_dir, payload)
    except OSError as error:
        print(f"Could not write final verdict: {error}", file=sys.stderr)
        return 2
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["main", "parse_args"]
