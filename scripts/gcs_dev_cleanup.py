"""Stop stale local GCS dev processes for THIS session (registry-scoped).

Windows-only. By default stops only the chat this session owns (its backend,
frontend, and NavPy sims, resolved from ``gcs.backend.instance_registry``) - safe
when many sessions run at once. ``--all`` does the legacy broad sweep across every
session (hardcoded ports 8000/3000 + all NavPy sims); use it only deliberately.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))
from gcs.backend import instance_registry as reg  # noqa: E402


BACKEND_PORT = 8000
FRONTEND_PORT = 3000


@dataclass(frozen=True)
class ProcessInfo:
    pid: int
    name: str
    command_line: str
    executable_path: str


class CleanupError(RuntimeError):
    """Raised when cleanup cannot safely continue."""


def main() -> int:
    parser = argparse.ArgumentParser(description="Stop stale GCS dev processes.")
    parser.add_argument("--dry-run", action="store_true", help="Print matches without stopping them.")
    parser.add_argument("--all", action="store_true",
                        help="Stop EVERY session's GCS + NavPy (WARNING: broad - affects all sessions, "
                             "hardcoded ports 8000/3000). Default stops only THIS session's chat.")
    args = parser.parse_args()

    if os.name != "nt":
        print("gcs_dev_cleanup.py is intended for the Windows dev environment.", file=sys.stderr)
        return 1

    try:
        if args.all:
            print("WARNING: --all stops EVERY session's GCS + NavPy, not just yours.")
            stop_stale_processes(dry_run=args.dry_run)
        else:
            stop_this_session(dry_run=args.dry_run)
    except CleanupError as exc:
        print(f"Cleanup failed: {exc}", file=sys.stderr)
        return 1
    return 0


def stop_this_session(*, dry_run: bool = False) -> None:
    """Stop only the chat owned by THIS session (resolved from the shared registry)."""
    entry = reg.find_for_owner(reg.owner_for(str(ROOT)))
    if entry is None:
        print("No GCS instance registered for this session - nothing to clean.")
        print("(Use --all to stop ALL sessions; that affects everyone.)")
        return
    n = entry["chat_index"]
    print(f"Stopping this session's stack (chat {n})...")
    stop_port_listener(entry["backend"], "GCS Backend", is_gcs_backend, dry_run=dry_run)
    stop_port_listener(entry["frontend"], "GCS Frontend", is_gcs_frontend, dry_run=dry_run)
    stop_navpy_sim(sysids=set(entry["sysids"]), dry_run=dry_run)
    if not dry_run:
        reg.release(n)
        print(f"Released chat {n}.")


def stop_stale_processes(*, dry_run: bool = False) -> None:
    """Legacy broad sweep (all sessions) - only via --all."""
    print("Checking for stale GCS dev processes (ALL sessions)...")
    stop_port_listener(BACKEND_PORT, "GCS Backend", is_gcs_backend, dry_run=dry_run)
    stop_port_listener(FRONTEND_PORT, "GCS Frontend", is_gcs_frontend, dry_run=dry_run)
    stop_navpy_sim(sysids=None, dry_run=dry_run)


def stop_port_listener(port: int, label: str, matcher, *, dry_run: bool) -> None:
    for pid in listening_pids(port):
        if pid == os.getpid():
            continue
        info = process_info(pid)
        if info is None:
            continue
        if not matcher(info):
            raise CleanupError(
                f"port {port} is occupied by unmatched process pid={pid} "
                f"name={info.name!r} command={info.command_line!r}"
            )
        terminate_pid_tree(pid, f"stale {label}", dry_run=dry_run)


def stop_navpy_sim(*, sysids: set[int] | None, dry_run: bool) -> None:
    """Stop NavPy sim processes. If *sysids* is given, only those (this session's);
    otherwise all (legacy broad sweep)."""
    for info in find_processes("navpy.main"):
        if info.pid == os.getpid():
            continue
        if not is_navpy_sim(info):
            continue
        if sysids is not None and _navpy_sysid(info) not in sysids:
            continue
        terminate_pid_tree(info.pid, "NavPy sim", dry_run=dry_run)


def _navpy_sysid(info: ProcessInfo) -> int | None:
    m = re.search(r"(?:^|\s)-ss(?:=|\s+)(\d+)", info.command_line)
    return int(m.group(1)) if m else None


def is_gcs_backend(info: ProcessInfo) -> bool:
    cmd = norm(info.command_line)
    return "uvicorn" in cmd and "gcs.backend.main:app" in cmd


def is_gcs_frontend(info: ProcessInfo) -> bool:
    cmd = norm(info.command_line)
    exe = norm(info.executable_path)
    has_node = "node" in norm(info.name) or "node" in exe or "npm" in norm(info.name)
    has_vite = "vite" in cmd or "vite" in exe
    has_gcs_frontend_path = "/src/gcs/frontend/" in cmd or "/src/gcs/frontend/" in exe
    return has_node and has_vite and has_gcs_frontend_path


def is_navpy_sim(info: ProcessInfo) -> bool:
    cmd = norm(info.command_line)
    has_module = re.search(r"(^|\s)-m\s+navpy\.main(\s|$)", cmd) is not None
    has_sim_detector = re.search(
        r"(^|\s)--detector-type(?:=|\s+)['\"]?sim['\"]?(?=\s|$)",
        cmd,
    ) is not None
    return has_module and has_sim_detector


def terminate_pid_tree(pid: int, label: str, *, dry_run: bool) -> None:
    action = "Would stop" if dry_run else "Stopping"
    print(f"{action} {label} pid={pid}...")
    if dry_run:
        return
    result = subprocess.run(
        ["taskkill", "/PID", str(pid), "/T", "/F"],
        capture_output=True,
        text=True,
    )
    if result.returncode not in (0, 128):
        detail = (result.stderr or result.stdout or "").strip()
        raise CleanupError(f"taskkill failed for pid={pid}: {detail}")


def listening_pids(port: int) -> list[int]:
    command = (
        "$ErrorActionPreference='SilentlyContinue'; "
        f"@(Get-NetTCPConnection -LocalPort {port} -State Listen | "
        "Select-Object -ExpandProperty OwningProcess | Sort-Object -Unique) | "
        "ConvertTo-Json -Compress"
    )
    data = powershell_json(command)
    if data is None:
        return []
    values = data if isinstance(data, list) else [data]
    return sorted({int(value) for value in values if value is not None})


def process_info(pid: int) -> ProcessInfo | None:
    command = (
        "$ErrorActionPreference='SilentlyContinue'; "
        f"Get-CimInstance Win32_Process -Filter \"ProcessId = {pid}\" | "
        "Select-Object ProcessId,Name,CommandLine,ExecutablePath | "
        "ConvertTo-Json -Compress"
    )
    data = powershell_json(command)
    if not data:
        return None
    return process_info_from_dict(data)


def find_processes(command_fragment: str) -> list[ProcessInfo]:
    safe_fragment = command_fragment.replace("'", "''")
    command = (
        "$ErrorActionPreference='SilentlyContinue'; "
        f"@(Get-CimInstance Win32_Process | Where-Object {{ $_.CommandLine -like '*{safe_fragment}*' }} | "
        "Select-Object ProcessId,Name,CommandLine,ExecutablePath) | ConvertTo-Json -Compress"
    )
    data = powershell_json(command)
    if data is None:
        return []
    rows = data if isinstance(data, list) else [data]
    return [process_info_from_dict(row) for row in rows if row]


def powershell_json(command: str):
    result = subprocess.run(
        ["powershell.exe", "-NoProfile", "-ExecutionPolicy", "Bypass", "-Command", command],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        detail = (result.stderr or result.stdout or "").strip()
        raise CleanupError(f"PowerShell query failed: {detail}")
    output = result.stdout.strip()
    if not output:
        return None
    return json.loads(output)


def process_info_from_dict(row: dict) -> ProcessInfo:
    return ProcessInfo(
        pid=int(row.get("ProcessId") or 0),
        name=str(row.get("Name") or ""),
        command_line=str(row.get("CommandLine") or ""),
        executable_path=str(row.get("ExecutablePath") or ""),
    )


def norm(value: str) -> str:
    return value.replace("\\", "/").lower()


if __name__ == "__main__":
    raise SystemExit(main())
