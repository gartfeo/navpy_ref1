"""Exactly one owner of each approach role (hitl_guidance.md section 9, stage 1 step 17).

A role's owner is whoever holds its footprint on this PC, in WSL or on Windows: its ports, and the programs that
play it (navpy's and aas-hitl's). The runner's own role whose process tree holds a claim owns it; any other holder is
foreign, and so is a socket whose process cannot be found (unknown is not ours). Before the run nothing may hold a
role; once the chain is up, and again before the run ends, each role must have exactly one owner, the runner's own.

| role      | owner     | footprint                                                                               |
|-----------|-----------|-----------------------------------------------------------------------------------------|
| autopilot | backing   | TCP listeners on SITL serial0 and HIL; clients of serial0 other than the router          |
| bridge    | bridge    | clients of the HIL port; ``-m tools.tuning.hitl_bridge``, ``-m aas_hitl bridge|run``      |
| emulator  | emulator  | UDP SIYI and vehicle-state ports; ``-m tools.hitl.zr10_emulator``, ``-m aas_hitl zr10``   |
| publisher | publisher | TCP listeners on the native-frame and RTSP ports; ``-m aas_hitl.video.gstreamer_encoder``, |
|           |           | ``-m aas_hitl video``                                                                   |
| contact   | contact   | UDP service and pose ports on the loopback; ``-m tools.hitl.contact_service``            |

approach_sources.py checks the same at the data level after the run.
"""
from __future__ import annotations

import os
import re
import subprocess
from dataclasses import dataclass
from typing import Callable, Iterable, Mapping, Optional

from tools.tuning import approach_chain as chain, approach_contact as contact
from tools.tuning.approach_procs import ProcessGroup, RunnerError
from tools.tuning.approach_tables import WINDOWS, WSL, Proc, Sock, Tables, windows_tables, wsl_tables

ROLE_OWNER = {"autopilot": "backing", "bridge": "bridge", "emulator": "emulator", "publisher": "publisher",
              "contact": contact.ROLE}
FOREIGN = "foreign"


@dataclass(frozen=True)
class PortRule:
    role: str
    proto: str
    port: int
    state: str                          # "listen", "bound" (UDP) or "client" (a TCP connection to it)
    allowed: str                        # the owned role that may hold it without being the role's owner
    unit: str                           # the one process that holds it; rules with the same unit share it


PORT_RULES = (
    PortRule("autopilot", "tcp", chain.SITL_PORT, "listen", "backing", "sitl"),
    PortRule("autopilot", "tcp", chain.HIL_PORT, "listen", "backing", "sitl"),
    PortRule("autopilot", "tcp", chain.SITL_PORT, "client", "router", "serial0 client"),  # a second serial owner
    PortRule("bridge", "tcp", chain.HIL_PORT, "client", "bridge", "bridge"),
    PortRule("emulator", "udp", chain.SIYI_PORT, "bound", "emulator", "emulator"),
    PortRule("emulator", "udp", chain.VEHICLE_STATE_PORT, "bound", "emulator", "emulator"),
    PortRule("publisher", "tcp", chain.NATIVE_PORT, "listen", "publisher", "native server"),
    PortRule("publisher", "tcp", chain.RTSP_PORT, "listen", "publisher", "rtsp server"),
    PortRule("contact", "udp", contact.SERVICE_PORT, "bound", contact.ROLE, "contact"),
    PortRule("contact", "udp", contact.POSE_PORT, "bound", contact.ROLE, "contact"),
)
# Programs by the module they run (``-m MODULE``): navpy's and aas-hitl's implementations of each role. aas-hitl's
# CLI by its command (cli.py): ``run`` is its flight launcher with the truth bridge, ``sitl start|mission`` its SITL.
PROGRAMS = {"tools.tuning.hitl_bridge": ("bridge",), "tools.hitl.zr10_emulator": ("emulator",),
            "aas_hitl.zr10": ("emulator",), "aas_hitl.video.gstreamer_encoder": ("publisher",),
            "tools.hitl.contact_service": ("contact",)}
AAS_HITL_COMMANDS = {("run",): ("autopilot", "bridge"), ("sitl", "start"): ("autopilot",),
                     ("sitl", "mission"): ("autopilot",), ("bridge",): ("bridge",), ("zr10", "emulate"): ("emulator",),
                     ("video", "publish"): ("publisher",)}


INTERPRETER = re.compile(r"(python[\d.]*w?|pyw?)(\.exe)?", re.IGNORECASE)     # pythonw/pyw: windowed
NICE = re.compile(r"nice(\.exe)?", re.IGNORECASE)
ARG_OPTIONS = ("-W", "-X", "-Q")       # interpreter options that take the next word


def _base(word: str) -> str:
    return word.replace("\\", "/").rsplit("/", 1)[-1]


def program_roles(argv: tuple[str, ...]) -> tuple[str, ...]:
    """The roles a command line plays by the module it runs (``[nice -n N] python [options] -m MODULE``); () for any
    other program, including a shell whose command text only names one."""
    start = 0
    if argv and NICE.fullmatch(_base(argv[0])):
        start = 1
        while start < len(argv) and argv[start].startswith("-"):
            start += 2 if argv[start] == "-n" else 1
    if start >= len(argv) or not INTERPRETER.fullmatch(_base(argv[start])):
        return ()
    i = start + 1
    while i < len(argv) - 1 and argv[i] != "-m":
        if not argv[i].startswith("-") or argv[i] == "-c":
            return ()                   # a script or a command string: not a module run
        i += 2 if argv[i] in ARG_OPTIONS else 1
    if i < len(argv) - 1:
        module = argv[i + 1]
        if module in PROGRAMS:
            return PROGRAMS[module]
        if module == "aas_hitl":
            words = tuple(t for t in argv[i + 2:] if not t.startswith("-"))[:2]
            return AAS_HITL_COMMANDS.get(words[:1], AAS_HITL_COMMANDS.get(words, ()))
        return ()
    return ()


# ---- who owns what ------------------------------------------------------------------------------------------------
WSL_RELAY = "wslrelay.exe"     # holds a Windows 127.0.0.1 mirror of every WSL TCP listener (NAT localhost forwarding)


class Owners:
    """Which runner role each process belongs to, by process tree: in WSL the role's pid, its process group or an
    ancestor; on Windows the role's pid (Unreal, the emulator's own Python) or an ancestor, and the venv launcher
    whose child it is (a Windows venv's python.exe starts the real interpreter as a child with the same arguments)."""

    def __init__(self, wsl_roots: Mapping[int, str], windows_pids: Mapping[int, str], procs: Iterable[Proc],
                 own_pid: int) -> None:
        self.roots = {WSL: dict(wsl_roots), WINDOWS: dict(windows_pids)}
        self.own_pid = own_pid
        self.procs = {(p.host, p.pid): p for p in procs}
        self.launchers: set[int] = set()     # Windows venv launchers: not a second instance of their child's program
        for proc in list(self.procs.values()):
            parent = self.procs.get((WINDOWS, proc.ppid)) if proc.host == WINDOWS else None
            role = self.roots[WINDOWS].get(proc.pid)
            if parent is not None and role is not None and parent.argv[1:] == proc.argv[1:] and proc.argv[1:]:
                self.roots[WINDOWS].setdefault(parent.pid, role)
                self.launchers.add(parent.pid)

    def of(self, host: str, pid: Optional[int]) -> str:
        roots, seen, current = self.roots[host], set(), pid
        while current and current not in seen:
            if current in roots:
                return roots[current]
            seen.add(current)
            proc = self.procs.get((host, current))
            if proc is None:
                break
            if proc.pgid in roots:
                return roots[proc.pgid]
            current = proc.ppid
        return FOREIGN

    def is_self(self, proc: Proc) -> bool:
        return proc.host == WSL and proc.pid == self.own_pid

    def is_relay(self, sock: Sock) -> bool:
        """A Windows listener that is only WSL's localhost mirror: the WSL table shows the real holder."""
        proc = self.procs.get((WINDOWS, sock.pid)) if sock.host == WINDOWS and sock.pid is not None else None
        if proc is None:
            return False
        image = proc.name or (proc.argv[0].replace("/", "\\").rsplit("\\", 1)[-1] if proc.argv else "")
        return image.lower() == WSL_RELAY      # its command line holds only its arguments


def group_owners(group: ProcessGroup) -> tuple[dict[int, str], dict[int, str]]:
    """The runner's roles by WSL process-tree root and by Windows pid (Unreal; the emulator's own Windows Python)."""
    wsl, windows = {}, {}
    for owned in group.owned:
        if owned.spec.kind == "windows":
            windows[owned.handle.pid] = owned.spec.role
            continue
        wsl[owned.handle.pid] = owned.spec.role
        windows_pid = getattr(owned.handle, "windows_pid", None)
        if windows_pid is not None:
            windows[windows_pid] = owned.spec.role
    return wsl, windows


def claims(tables: Iterable[Tables], owners: Owners) -> dict[str, list[dict]]:
    """Every role's claims: who holds which part of its footprint (the runner's own process excluded)."""
    found: dict[str, list[dict]] = {role: [] for role in ROLE_OWNER}
    for table in tables:
        for sock in table.socks:
            for rule in PORT_RULES:
                port = sock.remote_port if rule.state == "client" else sock.local_port
                state = "established" if rule.state == "client" else rule.state
                if sock.proto != rule.proto or port != rule.port or sock.state != state or owners.is_relay(sock):
                    continue
                owner = owners.of(sock.host, sock.pid)
                if rule.state == "client" and owner == rule.allowed and rule.allowed != ROLE_OWNER[rule.role]:
                    continue            # the router: the one expected client of serial0
                found[rule.role].append({"owner": owner, "host": sock.host, "pid": sock.pid, "unit": rule.unit,
                                         "what": f"{rule.proto} {rule.state} {rule.port}"})
        for proc in table.procs:
            if owners.is_self(proc):
                continue
            for role in program_roles(proc.argv):
                found[role].append({"owner": owners.of(proc.host, proc.pid), "host": proc.host, "pid": proc.pid,
                                    "what": " ".join(proc.argv)[:200],
                                    "instance": not (proc.host == WINDOWS and proc.pid in owners.launchers)})
    return found


def verdict(found: Mapping[str, list[dict]], running: bool) -> dict:
    """Before the run (``running`` False) no role may have an owner; during it each has exactly its runner role,
    running its program at most once per host (two copies inside the role's own tree are still two owners)."""
    roles, problems = {}, []
    for role, items in found.items():
        holders = sorted({item["owner"] for item in items})
        hosts = [item["host"] for item in items if item.get("instance")]
        copies = sorted({host for host in hosts if hosts.count(host) > 1})
        units = {(i["host"], i["unit"]) for i in items if "unit" in i}   # one process per unit: a hidden copy shows
        pids_of = {key: {i["pid"] for i in items if (i["host"], i.get("unit")) == key} for key in units}
        copies += sorted(f"{host} {unit} sockets" for (host, unit), pids in pids_of.items() if len(pids) > 1)
        ok = (holders == [ROLE_OWNER[role]] and not copies) if running else not holders
        roles[role] = {"owners": holders, "claims": items, "ok": ok}
        if not ok:
            want = f"exactly 1 owner ({ROLE_OWNER[role]})" if running else "no owner before the run"
            foreign = [f"{item['host']} pid {item['pid']}: {item['what']}" for item in items
                       if item["owner"] != ROLE_OWNER[role] or not running]
            problems.append(f"{role}: {want}, found {holders or 'none'}" + (f" ({'; '.join(foreign)})" if foreign
                                                                             else "")
                            + "".join(f"; 2 or more copies on {where}" for where in copies))
    return {"ok": not problems, "roles": roles, "problems": problems}


def census(group: ProcessGroup, tables: Callable[[], list[Tables]], own_pid: int, running: bool) -> dict:
    """One census; a table that cannot be read fails it (unknown is not free)."""
    try:
        taken = tables()
    except Exception as exc:       # noqa: BLE001 - the verdict says why
        return {"ok": False, "roles": {}, "problems": [f"owner tables unreadable: {type(exc).__name__}: {exc}"]}
    wsl, windows = group_owners(group)
    owners = Owners(wsl, windows, [proc for table in taken for proc in table.procs], own_pid)
    return verdict(claims(taken, owners), running)


PREFLIGHT, READY, END = "preflight", "ready", "end"


def host_tables() -> list[Tables]:
    return [wsl_tables(), windows_tables(subprocess.run)]


def check(run: dict, group: ProcessGroup, phase: str, tables: Optional[Callable[[], list[Tables]]] = None) -> None:
    """The runner's census at *phase*, recorded in ``run["owners"]``. Before the run nothing may own a role and
    once the chain is up each has exactly its own owner: a failure there raises. At the end it is only recorded (the
    run's stop must still run), and only once the chain came up."""
    census_run = run.setdefault("owners", {})
    if phase == END and READY not in census_run:
        return
    result = census_run[phase] = census(group, tables or host_tables, os.getpid(), running=phase != PREFLIGHT)
    if not result["ok"] and phase != END:
        raise RunnerError(f"owner census ({phase}): " + "; ".join(result["problems"]))


def verdicts(run: dict) -> tuple[bool, list[str]]:
    """Every census passed and the sources were single; the problems found."""
    census_run, sources_run = run.get("owners", {}), run.get("sources") or {"ok": False, "problems": ["no sources"]}
    problems = [f"owners {phase}: {problem}" for phase, result in census_run.items() for problem in result["problems"]]
    problems += [f"sources: {problem}" for problem in sources_run["problems"]]
    passed = all(census_run.get(phase, {}).get("ok") for phase in (PREFLIGHT, READY, END)) and sources_run["ok"]
    return passed, problems


__all__ = ["END", "Owners", "PORT_RULES", "PREFLIGHT", "READY", "ROLE_OWNER", "census", "check", "claims",
           "group_owners", "host_tables", "program_roles", "verdicts"]
