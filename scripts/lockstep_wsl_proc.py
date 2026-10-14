"""WSL-side process identity and ownership-checked stop (python3, stdlib only).

identity PID          -> "<start_ticks> <cwd>" (start ticks: /proc stat field 22)
stop PID TICKS CWD    -> stopped | killed | gone | not-ours
alive PID TICKS CWD   -> alive | gone (gone: no process, or another one has the pid)

The stop holds a pidfd from before the identity check to the end, so neither
the check, the TERM, the wait nor a KILL can reach a process that reused the
pid.
"""

from __future__ import annotations

import os
import select
import signal
import sys

#: Wall seconds a stopped process gets to exit after TERM, before KILL.
TERM_GRACE_S = 5.0


def start_ticks(pid: int) -> str:
    with open(f"/proc/{pid}/stat", encoding="ascii") as stream:
        stat = stream.read()
    # Field 2 (comm) may hold spaces: fields 3.. follow the last ')'.
    return stat.rsplit(")", 1)[1].split()[19]


def identity(pid: int) -> str:
    return f"{start_ticks(pid)} {os.readlink(f'/proc/{pid}/cwd')}"


def stop(pid: int, ticks: str, cwd: str) -> str:
    try:
        handle = os.pidfd_open(pid)
    except ProcessLookupError:
        return "gone"
    try:
        try:
            if start_ticks(pid) != ticks or os.readlink(f"/proc/{pid}/cwd") != cwd:
                return "not-ours"
        except (FileNotFoundError, ProcessLookupError):
            return "gone"
        signal.pidfd_send_signal(handle, signal.SIGTERM)
        if select.select([handle], [], [], TERM_GRACE_S)[0]:
            return "stopped"
        signal.pidfd_send_signal(handle, signal.SIGKILL)
        select.select([handle], [], [], TERM_GRACE_S)
        return "killed"
    except ProcessLookupError:
        return "stopped"
    finally:
        os.close(handle)


def alive(pid: int, ticks: str, cwd: str) -> str:
    try:
        ours = start_ticks(pid) == ticks and os.readlink(f"/proc/{pid}/cwd") == cwd
    except (FileNotFoundError, ProcessLookupError):
        return "gone"
    return "alive" if ours else "gone"


def main(argv: list[str]) -> int:
    if argv[:1] == ["identity"] and len(argv) == 2:
        print(identity(int(argv[1])))
        return 0
    if argv[:1] == ["alive"] and len(argv) == 4:
        print(alive(int(argv[1]), argv[2], argv[3]))
        return 0
    if argv[:1] == ["stop"] and len(argv) == 4:
        print(stop(int(argv[1]), argv[2], argv[3]))
        return 0
    print(__doc__, file=sys.stderr)
    return 2


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
