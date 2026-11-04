"""The process and socket tables the approach runner's owner census reads (approach_owners.py), in WSL and on Windows.

WSL: /proc (each process's parent, group and command line; each socket's port, state and, through the fd links, its
process). Windows: Win32_Process through PowerShell and ``netstat -ano``. Read-only; a socket whose process cannot be
read keeps no pid.
"""
from __future__ import annotations

import json
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional

WSL, WINDOWS = "wsl", "windows"


@dataclass(frozen=True)
class Proc:
    host: str
    pid: int
    ppid: int
    argv: tuple[str, ...]
    pgid: Optional[int] = None          # WSL only


@dataclass(frozen=True)
class Sock:
    host: str
    proto: str                          # "tcp" | "udp"
    local_port: int
    state: str                          # "listen" | "established" | "bound"
    pid: Optional[int]                  # None: no process found for it
    remote_port: Optional[int] = None


@dataclass(frozen=True)
class Tables:
    procs: tuple[Proc, ...]
    socks: tuple[Sock, ...]


TCP_STATES = {"0A": "listen", "01": "established", "02": "established"}     # /proc/net/tcp st: LISTEN, ESTAB, SYN_SENT


def parse_proc_net(text: str, proto: str, inode_pid: Mapping[int, int]) -> list[Sock]:
    """/proc/net/{tcp,tcp6,udp,udp6} rows; TCP rows in other states (TIME_WAIT...) hold no port and are skipped."""
    socks = []
    for line in text.splitlines()[1:]:
        fields = line.split()
        if len(fields) < 10:
            continue
        local_port = int(fields[1].rsplit(":", 1)[1], 16)
        remote_port = int(fields[2].rsplit(":", 1)[1], 16)
        state = TCP_STATES.get(fields[3]) if proto == "tcp" else "bound"
        if state is None:
            continue
        inode = int(fields[9])
        socks.append(Sock(WSL, proto, local_port, state, inode_pid.get(inode), remote_port or None))
    return socks


def wsl_tables(proc: Path = Path("/proc")) -> Tables:
    procs, inode_pid = [], {}
    for entry in proc.iterdir():
        if not entry.name.isdigit():
            continue
        try:
            stat = (entry / "stat").read_text()
            argv = tuple(arg for arg in (entry / "cmdline").read_bytes().decode("utf-8", "replace").split("\0") if arg)
            fields = stat[stat.rindex(")") + 2:].split()
            procs.append(Proc(WSL, int(entry.name), int(fields[1]), argv, int(fields[2])))
            for fd in (entry / "fd").iterdir():
                target = str(fd.readlink())
                if target.startswith("socket:["):
                    inode_pid[int(target[8:-1])] = int(entry.name)
        except (OSError, ValueError):       # gone meanwhile, or not ours to read: its sockets stay unmapped
            continue
    socks = [sock for name, proto in (("tcp", "tcp"), ("tcp6", "tcp"), ("udp", "udp"), ("udp6", "udp"))
             for sock in parse_proc_net((proc / "net" / name).read_text(), proto, inode_pid)]
    return Tables(tuple(procs), tuple(socks))


NETSTAT_STATES = {"LISTENING": "listen", "ESTABLISHED": "established", "SYN_SENT": "established"}


def parse_netstat(text: str) -> list[Sock]:
    """``netstat -ano`` rows: TCP proto, local, remote, state, pid; UDP proto, local, remote, pid."""
    socks = []
    for line in text.splitlines():
        fields = line.split()
        if len(fields) == 5 and fields[0] == "TCP" and fields[4].isdigit() and fields[3] in NETSTAT_STATES:
            remote = fields[2].rsplit(":", 1)[1]
            socks.append(Sock(WINDOWS, "tcp", int(fields[1].rsplit(":", 1)[1]), NETSTAT_STATES[fields[3]],
                              int(fields[4]), int(remote) if remote.isdigit() and remote != "0" else None))
        elif len(fields) == 4 and fields[0] == "UDP" and fields[3].isdigit():
            socks.append(Sock(WINDOWS, "udp", int(fields[1].rsplit(":", 1)[1]), "bound", int(fields[3])))
    return socks


def parse_windows_procs(text: str) -> list[Proc]:
    """PowerShell's ConvertTo-Json of Win32_Process ProcessId, ParentProcessId, CommandLine (an object or a list)."""
    rows = json.loads(text) if text.strip() else []
    rows = [rows] if isinstance(rows, dict) else rows
    return [Proc(WINDOWS, int(row["ProcessId"]), int(row["ParentProcessId"]),
                 tuple(_split_windows(row.get("CommandLine") or ""))) for row in rows]


def _split_windows(line: str) -> list[str]:
    return [part.strip('"') for part in re.findall(r'"[^"]*"|\S+', line)]


WINDOWS_PROCS = ("Get-CimInstance Win32_Process | Select-Object ProcessId,ParentProcessId,CommandLine | "
                 "ConvertTo-Json -Compress")


def windows_tables(run: Callable[..., object]) -> Tables:
    """*run* is subprocess.run; a query that fails raises (unknown is not free)."""
    procs = run(["powershell.exe", "-NoProfile", "-NonInteractive", "-Command", WINDOWS_PROCS], capture_output=True,
                text=True, timeout=60, check=True).stdout
    socks = run(["netstat.exe", "-ano"], capture_output=True, text=True, timeout=60, check=True).stdout
    return Tables(tuple(parse_windows_procs(procs)), tuple(parse_netstat(socks)))


__all__ = ["Proc", "Sock", "Tables", "WINDOWS", "WSL", "parse_netstat", "parse_proc_net", "parse_windows_procs",
           "windows_tables", "wsl_tables"]
