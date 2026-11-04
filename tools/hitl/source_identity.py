"""The navpy checkout a HITL run record names (approach_records.py ``sources``): commit, diff and untracked files."""
from __future__ import annotations

import hashlib
import subprocess
from pathlib import Path

from tools.hitl import json_checks as jc
from tools.hitl.json_checks import RecordError

REPO_ROOT = Path(__file__).resolve().parents[2]


def _git(repo: Path, *args: str) -> bytes:
    result = subprocess.run(["git", "-C", str(repo), *args], capture_output=True, timeout=60, check=False)
    if result.returncode != 0:
        raise RecordError([f"git {' '.join(args)} failed in {repo}: {result.stderr.decode(errors='replace')}"])
    return result.stdout


def source_identity(repo: Path = REPO_ROOT) -> dict[str, str]:
    """The checkout's commit, uncommitted diff and untracked files, in the demo eval's input-manifest fields
    (scripts/eval_gcs_demo_inputs.py)."""
    untracked = hashlib.sha256()
    listing = _git(repo, "ls-files", "--others", "--exclude-standard", "-z").decode("utf-8")
    for name in sorted(n for n in listing.split("\0") if n):
        path = Path(repo) / name
        untracked.update(f"{name}\0{jc.sha256_file(path) if path.is_file() else 'absent'}\n".encode("utf-8"))
    return {"git_head": _git(repo, "rev-parse", "HEAD").decode("ascii").strip(),
            "uncommitted_diff_sha256": hashlib.sha256(_git(repo, "diff", "HEAD", "--binary")).hexdigest(),
            "untracked_sha256": untracked.hexdigest()}


__all__ = ["source_identity"]
