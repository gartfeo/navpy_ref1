"""WSL-side reap of one wall launch's SITLs and router (python3, stdlib only).

A process is the launch's only when both hold:

- its command line is a SITL with one of the given sysids (``--sysid N`` or
  ``--sysid=N``) or a ``mavlink-routerd`` on the given monitor port, and
- its environment holds ``SWARM_OFFSET=<offset>``. ``launch_command`` sets it
  for every wall launch and ``run_swarm.sh`` passes it to the SITLs and the
  router it starts; the offset is ``3 * chat``, so it names one slot.

A sysid alone names no owner: lockstep SITLs take ``--sysid 1..3`` on any slot
(``lockstep_wsl.sitl_command``) and start under ``env -i``, so they carry no
``SWARM_OFFSET`` and are never matched (todo 51).

Each process is signalled through a pidfd opened before its ownership is
checked, so neither TERM nor KILL can reach a process that reused the pid.

    reap OFFSET ROUTER_PORT SYSID...   -> one "<pid> stopped|killed|survived-kill" line each

Any failure exits non-zero; ``swarm_run_wsl.cleanup`` then raises.
"""

from __future__ import annotations

import os
import select
import signal
import sys
import time

#: Wall seconds the launch's processes get to exit after TERM, before KILL.
TERM_GRACE_S = 2.0
ROUTER = "mavlink-routerd"


def sitl_sysid(argv: list[str]) -> int | None:
    """The ``--sysid`` an ArduPilot command line names, in either spelling."""
    for index, arg in enumerate(argv):
        value = None
        if arg == "--sysid" and index + 1 < len(argv):
            value = argv[index + 1]
        elif arg.startswith("--sysid="):
            value = arg.split("=", 1)[1]
        if value is not None and value.isdigit():
            return int(value)
    return None


def is_router(argv: list[str], router_port: int) -> bool:
    return (bool(argv) and os.path.basename(argv[0]) == ROUTER
            and any(arg.endswith(f":{router_port}") for arg in argv[1:]))


def owned(argv: list[str], environ: list[str], offset: int,
          router_port: int, sysids: set[int]) -> bool:
    """Whether a process with this command line and environment is the launch's."""
    if f"SWARM_OFFSET={offset}" not in environ:
        return False
    return sitl_sysid(argv) in sysids or is_router(argv, router_port)


def read_nul_list(path: str) -> list[str]:
    with open(path, "rb") as stream:
        return [item.decode("utf-8", "replace") for item in stream.read().split(b"\0") if item]


def owned_now(proc: str, pid: int, offset: int, router_port: int, sysids: set[int]) -> bool:
    try:
        return owned(read_nul_list(f"{proc}/{pid}/cmdline"),
                     read_nul_list(f"{proc}/{pid}/environ"), offset, router_port, sysids)
    except (FileNotFoundError, ProcessLookupError, PermissionError):
        return False


def owned_pids(proc: str, offset: int, router_port: int, sysids: set[int]) -> list[int]:
    pids = sorted(int(name) for name in os.listdir(proc) if name.isdigit())
    return [pid for pid in pids if owned_now(proc, pid, offset, router_port, sysids)]


def reap(offset: int, router_port: int, sysids: set[int], *, proc: str = "/proc") -> list[str]:
    handles: dict[int, int] = {}
    for pid in owned_pids(proc, offset, router_port, sysids):
        try:
            handle = os.pidfd_open(pid)
        except ProcessLookupError:
            continue
        # Checked again behind the pidfd: the pid now names this process for good.
        if not owned_now(proc, pid, offset, router_port, sysids):
            os.close(handle)
            continue
        try:
            signal.pidfd_send_signal(handle, signal.SIGTERM)
        except ProcessLookupError:
            pass
        handles[pid] = handle
    reports = []
    deadline = time.monotonic() + TERM_GRACE_S
    while handles:
        ready, _, _ = select.select(list(handles.values()), [], [],
                                    max(0.0, deadline - time.monotonic()))
        if not ready:
            break
        for pid in [pid for pid, handle in handles.items() if handle in ready]:
            os.close(handles.pop(pid))
            reports.append(f"{pid} stopped")
    for pid, handle in handles.items():
        try:
            signal.pidfd_send_signal(handle, signal.SIGKILL)
        except ProcessLookupError:
            pass
        exited = bool(select.select([handle], [], [], TERM_GRACE_S)[0])
        os.close(handle)
        reports.append(f"{pid} {'killed' if exited else 'survived-kill'}")
    return reports


def main(argv: list[str]) -> int:
    if len(argv) >= 3 and argv[0] == "reap" and all(a.isdigit() for a in argv[1:]):
        if not (hasattr(os, "pidfd_open") and hasattr(signal, "pidfd_send_signal")):
            print("wall_reap_wsl: this python3 has no pidfd support", file=sys.stderr)
            return 3
        reports = reap(int(argv[1]), int(argv[2]), {int(a) for a in argv[3:]})
        for line in reports:
            print(line)
        return 1 if any(line.endswith("survived-kill") for line in reports) else 0
    print(__doc__, file=sys.stderr)
    return 2


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
