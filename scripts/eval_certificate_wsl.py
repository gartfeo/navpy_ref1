"""Shell-free WSL source and EEPROM identity probes."""

from __future__ import annotations

import posixpath
import re
from typing import Any

from scripts.eval_certificate_process import (
    normalize_wsl_path,
    run_text,
    wsl_executable,
)
from scripts.eval_certificate_profile import IDENTITY_PROBE_TIMEOUT_S
from scripts.eval_certificate_values import failed_reasons, require_finite


_SHA256_RE = re.compile(r"^[0-9a-fA-F]{64}$")


def wsl_ardupilot_identity(
    *,
    repo: str = "~/ardupilot",
    timeout_s: float = IDENTITY_PROBE_TIMEOUT_S,
) -> dict[str, Any]:
    """Best-effort identity of a WSL checkout, using argv-only execution."""
    timeout = _timeout(timeout_s)
    executable = wsl_executable()
    if executable is None:
        return {"available": False, "reason": "wsl.exe is not on PATH"}
    resolved, resolve_error = normalize_wsl_path(
        repo,
        executable=executable,
        timeout_s=timeout,
    )
    if resolved is None:
        return {"available": False, "repo": repo, "reason": resolve_error}
    commit, commit_error = run_text(
        [executable, "--exec", "git", "-C", resolved, "rev-parse", "HEAD"],
        timeout_s=timeout,
    )
    if commit is None:
        return {
            "available": False,
            "repo": repo,
            "reason": commit_error or "no output",
        }
    dirty, dirty_error = run_text(
        [executable, "--exec", "git", "-C", resolved, "status", "--porcelain"],
        timeout_s=timeout,
    )
    return {
        "available": True,
        "repo": repo,
        "commit": commit.splitlines()[0].strip() if commit else None,
        "dirty": None if dirty is None else bool(dirty.strip()),
        "unavailable": failed_reasons(dirty=dirty_error),
    }


def eeprom_identity(
    sysid: int,
    *,
    root: str = "~/ardupilot",
    timeout_s: float = IDENTITY_PROBE_TIMEOUT_S,
) -> dict[str, Any]:
    """Best-effort identity of a SITL EEPROM, using direct WSL argv calls."""
    vehicle_id = _sysid(sysid)
    timeout = _timeout(timeout_s)
    executable = wsl_executable()
    if executable is None:
        return {"available": False, "reason": "wsl.exe is not on PATH"}
    resolved_root, resolve_error = normalize_wsl_path(
        root,
        executable=executable,
        timeout_s=timeout,
    )
    display_path = posixpath.join(root, str(vehicle_id), "eeprom.bin")
    if resolved_root is None:
        return {
            "available": False,
            "path": display_path,
            "reason": resolve_error,
        }
    resolved_path = posixpath.join(resolved_root, str(vehicle_id), "eeprom.bin")
    stat_text, stat_error = run_text(
        [executable, "--exec", "stat", "-c", "%s %Y", "--", resolved_path],
        timeout_s=timeout,
    )
    if stat_text is None:
        return {
            "available": False,
            "path": display_path,
            "reason": stat_error or "no stat output",
        }
    checksum_text, checksum_error = run_text(
        [executable, "--exec", "sha256sum", "--", resolved_path],
        timeout_s=timeout,
    )
    if checksum_text is None:
        return {
            "available": False,
            "path": display_path,
            "reason": checksum_error or "no checksum output",
        }
    parsed = _parse_eeprom_output(stat_text, checksum_text)
    if isinstance(parsed, str):
        return {"available": False, "path": display_path, "reason": parsed}
    size, mtime, checksum = parsed
    return {
        "available": True,
        "path": display_path,
        "size_bytes": size,
        "mtime_epoch_s": mtime,
        "sha256": checksum,
    }


def _parse_eeprom_output(
    stat_text: str,
    checksum_text: str,
) -> tuple[int, int, str] | str:
    stat_parts = stat_text.strip().split()
    checksum_parts = checksum_text.strip().split()
    if len(stat_parts) != 2:
        return f"unparsable stat output: {stat_text!r}"
    try:
        size, mtime = (int(value) for value in stat_parts)
    except ValueError:
        return f"unparsable stat output: {stat_text!r}"
    if size < 0:
        return f"unparsable stat output: {stat_text!r}"
    checksum = checksum_parts[0] if checksum_parts else ""
    if _SHA256_RE.fullmatch(checksum) is None:
        return f"unparsable sha256sum output: {checksum_text!r}"
    return size, mtime, checksum.lower()


def _sysid(value: Any) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError(f"sysid must be an integer, not {value!r}")
    if not 1 <= value <= 255:
        raise ValueError(f"sysid must be in 1..255, not {value}")
    return value


def _timeout(value: Any) -> float:
    return require_finite(
        value,
        name="timeout_s",
        minimum=0.0,
        minimum_inclusive=False,
    )


__all__ = ["eeprom_identity", "wsl_ardupilot_identity"]
