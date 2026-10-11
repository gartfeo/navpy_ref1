"""Run Python code in a fresh interpreter that imports this checkout's src/."""

from __future__ import annotations

import os
import subprocess
import sys

from tests.guards.step1_sources import REPO_ROOT

SRC = REPO_ROOT / "src"


def run_fresh(code: str, *args: str, timeout_s: float = 300) -> subprocess.CompletedProcess[str]:
    """`python -c code args...` with src/ first on the path (and as the working
    directory), so an installed navpy or the repo root cannot stand in for it."""
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join(p for p in (str(SRC), env.get("PYTHONPATH")) if p)
    return subprocess.run(
        [sys.executable, "-c", code, *args],
        cwd=SRC, env=env, capture_output=True, text=True, timeout=timeout_s, check=False,
    )


def error_tail(result: subprocess.CompletedProcess[str], lines: int = 3) -> str:
    return "\n    ".join(result.stderr.strip().splitlines()[-lines:])
