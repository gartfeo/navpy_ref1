"""Safe subprocess primitives used by certificate identity probes."""

from __future__ import annotations

import posixpath
import shutil
import subprocess
from pathlib import Path
from typing import Sequence

from scripts.eval_certificate_profile import IDENTITY_PROBE_TIMEOUT_S
from scripts.eval_certificate_values import require_finite


def run_text(
    command: Sequence[str],
    *,
    cwd: Path | None = None,
    timeout_s: float = IDENTITY_PROBE_TIMEOUT_S,
) -> tuple[str | None, str]:
    """Run an argv-only command and return ``(stdout, reason)``."""
    timeout = require_finite(
        timeout_s,
        name="timeout_s",
        minimum=0.0,
        minimum_inclusive=False,
    )
    if not command or not all(isinstance(part, str) and part for part in command):
        raise ValueError("command must contain non-empty argv strings")
    try:
        completed = subprocess.run(
            list(command),
            cwd=str(cwd) if cwd is not None else None,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
    except FileNotFoundError:
        return None, f"{command[0]} is not available on this machine"
    except subprocess.TimeoutExpired:
        return None, f"{command[0]} did not answer within {timeout:g}s"
    except OSError as exc:
        return None, f"{command[0]} could not be run ({exc})"
    if completed.returncode != 0:
        detail = (completed.stderr or "").strip().splitlines()
        return None, (
            f"{command[0]} exited {completed.returncode}"
            + (f": {detail[-1]}" if detail else "")
        )
    return completed.stdout.strip(), ""


def wsl_executable() -> str | None:
    if shutil.which("wsl.exe") is not None:
        return "wsl.exe"
    if shutil.which("wsl") is not None:
        return "wsl"
    return None


def normalize_wsl_path(
    raw_path: str,
    *,
    executable: str,
    timeout_s: float,
) -> tuple[str | None, str]:
    """Lexically normalize a WSL path, expanding ``~/`` without a shell."""
    if not isinstance(raw_path, str):
        raise TypeError("WSL path must be a string")
    if not raw_path.strip() or "\0" in raw_path or "\r" in raw_path or "\n" in raw_path:
        raise ValueError("WSL path must be non-empty and contain no control characters")
    if raw_path.startswith("~") and raw_path != "~" and not raw_path.startswith("~/"):
        raise ValueError("named-user WSL home paths are not supported")
    if raw_path == "~" or raw_path.startswith("~/"):
        home, error = run_text(
            [executable, "--exec", "printenv", "HOME"],
            timeout_s=timeout_s,
        )
        if home is None:
            return None, error or "WSL HOME could not be determined"
        home_line = home.splitlines()[0].strip() if home else ""
        if not home_line.startswith("/") or any(char in home_line for char in "\0\r\n"):
            return None, f"unparsable WSL HOME: {home!r}"
        suffix = raw_path[2:] if raw_path.startswith("~/") else ""
        return posixpath.normpath(posixpath.join(home_line, suffix)), ""
    return posixpath.normpath(raw_path), ""
