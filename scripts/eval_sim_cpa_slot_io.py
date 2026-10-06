"""WSL slot IO primitives for the SIM_CPA cross-check.

One concern: talking to the ArduPilot slot on the WSL side of this host
-- reading LASTLOG.TXT, statting and copying BIN files, confirming the
slot's SITL process is gone, and hashing the deployed binary.  What the
numbers MEAN (the successor binding policy) lives in
eval_sim_cpa_artifact.py; nothing here decides anything.
"""

from __future__ import annotations

import subprocess
import sys
import time
from pathlib import Path

_SCRIPTS = str(Path(__file__).resolve().parent)
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from scripts.eval_certificate_process import (  # noqa: E402
    normalize_wsl_path, run_text, wsl_executable,
)
from swarm_run_wsl import sysid_process_pattern  # noqa: E402

ARDUPILOT_ROOT = "~/ardupilot"
ARDUPLANE_BINARY = "~/ardupilot/build/sitl/bin/arduplane"
WSL_TIMEOUT_S = 30.0
SITL_EXIT_TIMEOUT_S = 30.0


def _wsl() -> str | None:
    return wsl_executable()


def resolve_logs_dir(sysid: int) -> tuple[str | None, str]:
    executable = _wsl()
    if executable is None:
        return None, "wsl.exe is not on PATH"
    root, error = normalize_wsl_path(
        ARDUPILOT_ROOT, executable=executable, timeout_s=WSL_TIMEOUT_S
    )
    if root is None:
        return None, error
    return f"{root}/{sysid}/logs", ""


def read_lastlog(logs_dir: str) -> tuple[int | None, str]:
    """The LASTLOG.TXT number, or (None, reason) when absent/unreadable."""
    executable = _wsl()
    if executable is None:
        return None, "wsl.exe is not on PATH"
    text, error = run_text(
        [executable, "--exec", "cat", "--", f"{logs_dir}/LASTLOG.TXT"],
        timeout_s=WSL_TIMEOUT_S,
    )
    if text is None:
        return None, error or "LASTLOG.TXT unreadable"
    try:
        return int(text.strip()), ""
    except ValueError:
        return None, f"unparsable LASTLOG.TXT content: {text!r}"


def count_bins(logs_dir: str) -> int | None:
    executable = _wsl()
    if executable is None:
        return None
    text, _ = run_text(
        [
            executable, "--exec", "find", logs_dir, "-maxdepth", "1",
            "-name", "*.BIN",
        ],
        timeout_s=WSL_TIMEOUT_S,
    )
    if text is None:
        return None
    return sum(1 for line in text.splitlines() if line.strip())


def bin_path_for(logs_dir: str, number: int) -> str:
    return f"{logs_dir}/{number:08d}.BIN"


def wait_for_sitl_exit(
    sysid: int, timeout_s: float = SITL_EXIT_TIMEOUT_S
) -> tuple[bool, str]:
    """True once no process matches the slot's SITL pattern.

    pgrep exits 1 for "no matches", which is the success condition here;
    run_text cannot express that, so this uses its own runner.  Any other
    failure counts as "could not confirm" -- fail closed.
    """
    executable = _wsl()
    if executable is None:
        return False, "wsl.exe is not on PATH"
    pattern = sysid_process_pattern(sysid)
    deadline = time.monotonic() + timeout_s
    last_detail = ""
    while time.monotonic() < deadline:
        try:
            completed = subprocess.run(
                [executable, "--exec", "pgrep", "-f", "--", pattern],
                capture_output=True,
                text=True,
                timeout=WSL_TIMEOUT_S,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            return False, f"pgrep probe failed: {exc}"
        if completed.returncode == 1:
            return True, ""
        if completed.returncode == 0:
            last_detail = (
                f"still running: {completed.stdout.strip().splitlines()[:3]}"
            )
        else:
            return False, (
                f"pgrep exited {completed.returncode}: "
                f"{(completed.stderr or '').strip()}"
            )
        time.sleep(0.5)
    return False, f"SITL did not exit within {timeout_s:g}s ({last_detail})"


def stat_size_mtime(path: str) -> tuple[tuple[int, int] | None, str]:
    executable = _wsl()
    if executable is None:
        return None, "wsl.exe is not on PATH"
    text, error = run_text(
        [executable, "--exec", "stat", "-c", "%s %Y", "--", path],
        timeout_s=WSL_TIMEOUT_S,
    )
    if text is None:
        return None, error
    parts = text.split()
    try:
        return (int(parts[0]), int(parts[1])), ""
    except (IndexError, ValueError):
        return None, f"unparsable stat output: {text!r}"


def copy_slot_file(source: str, destination: Path) -> tuple[bool, str]:
    """Copy a WSL-side file to a Windows-side destination path."""
    executable = _wsl()
    if executable is None:
        return False, "wsl.exe is not on PATH"
    destination_wsl, path_error = run_text(
        [executable, "--exec", "wslpath", "-a", str(destination)],
        timeout_s=WSL_TIMEOUT_S,
    )
    if destination_wsl is None:
        return False, path_error
    _, copy_error = run_text(
        [executable, "--exec", "cp", "--", source, destination_wsl],
        timeout_s=max(WSL_TIMEOUT_S, 120.0),
    )
    if copy_error:
        return False, copy_error
    return True, ""


def arduplane_sha256() -> tuple[str | None, str]:
    """SHA-256 of the deployed SITL binary that produced SCPC/SCPA."""
    executable = _wsl()
    if executable is None:
        return None, "wsl.exe is not on PATH"
    binary, error = normalize_wsl_path(
        ARDUPLANE_BINARY, executable=executable, timeout_s=WSL_TIMEOUT_S
    )
    if binary is None:
        return None, error
    text, sha_error = run_text(
        [executable, "--exec", "sha256sum", "--", binary],
        timeout_s=120.0,
    )
    if text is None:
        return None, sha_error
    digest = text.split()[0] if text.split() else ""
    if len(digest) != 64:
        return None, f"unparsable sha256sum output: {text!r}"
    return digest.lower(), ""


__all__ = [
    "ARDUPLANE_BINARY",
    "ARDUPILOT_ROOT",
    "SITL_EXIT_TIMEOUT_S",
    "WSL_TIMEOUT_S",
    "arduplane_sha256",
    "bin_path_for",
    "copy_slot_file",
    "count_bins",
    "read_lastlog",
    "resolve_logs_dir",
    "stat_size_mtime",
    "wait_for_sitl_exit",
]
