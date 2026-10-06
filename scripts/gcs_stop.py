"""Stop a GCS chat instance — only its own processes, never another session's.

This replaces broad teardowns (``taskkill //F //IM python.exe``,
``pkill -f arduplane``) that kill every session's stack. It reads the shared
registry (``gcs.backend.instance_registry``) to find the owning PIDs and stops
exactly that chat's backend (and its NavPy children), frontend, and SITL swarm
(its sys_ids + router), then releases the slot.

Usage::

    python scripts/gcs_stop.py --chat 1      # stop chat 1's stack
    python scripts/gcs_stop.py --chat 1 --keep-sitl
    python scripts/gcs_stop.py --list        # show running instances
    python scripts/gcs_stop.py --all         # stop ALL instances (explicit, opt-in)
"""
from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SRC_DIR = ROOT / "src"
sys.path.insert(0, str(SRC_DIR))
sys.path.insert(0, str(Path(__file__).resolve().parent))  # sibling scripts (swarm_run)

from gcs.backend import instance_ports as ip  # noqa: E402
from gcs.backend import instance_registry as reg  # noqa: E402
import swarm_run  # noqa: E402


def _kill_tree(pid) -> None:
    if not pid:
        return
    if os.name == "nt":
        subprocess.run(["taskkill", "/PID", str(pid), "/T", "/F"], capture_output=True)
    else:
        try:
            os.kill(int(pid), signal.SIGTERM)
        except (ProcessLookupError, ValueError):
            pass


def _verified_pid(entry: dict, pid_key: str, start_key: str):
    """The entry's pid if it is still the recorded process, else None.

    A recycled pid (same number, different process — creation stamp mismatch)
    or an already-dead one must not be tree-killed: taskkill /T on a recycled
    pid would take down an unrelated session's process tree.
    """
    pid = entry.get(pid_key)
    if pid and not reg.pid_matches(pid, entry.get(start_key)):
        return None
    return pid


def stop_chat(n: int, kill_sitl: bool = True) -> None:
    entry = reg.get(n)
    if entry is None:
        print(f"chat {n}: no registry entry (already stopped?)")
    else:
        backend_pid = _verified_pid(entry, "backend_pid", "backend_pid_start")
        frontend_pid = _verified_pid(entry, "frontend_pid", "frontend_pid_start")
        _kill_tree(backend_pid)
        _kill_tree(frontend_pid)
        print(f"chat {n}: stopped backend pid={backend_pid} "
              f"frontend pid={frontend_pid}")
    # Reap anything still holding this chat's frontend/backend ports — e.g. a Vite
    # dev server that outlived the pid tree-kill. Otherwise it orphans onto the
    # port, stays invisible to the registry, and blocks the next session's slot.
    # Companion UDP ports too (shared helper — see reap_companion_ports).
    protected = reg.registered_pids()
    reaped = reg.reap_port(ip.frontend_port(n)) + reg.reap_port(ip.backend_port(n))
    reaped += reg.reap_companion_ports(n, exclude=protected)
    if reaped:
        print(f"chat {n}: reaped orphan pids {reaped} on this chat's ports")
    if kill_sitl:
        # Kill the swarm_run supervisor FIRST: if it is still inside its
        # bring-up/verify loop, killing only the WSL SITL would make it
        # observe missing heartbeats and RELAUNCH the swarm we just stopped.
        if entry is not None:
            supervisor_pid = _verified_pid(entry, "sitl_pid", "sitl_pid_start")
            if supervisor_pid:
                _kill_tree(supervisor_pid)
                print(f"chat {n}: stopped SITL supervisor pid={supervisor_pid}")
        swarm_run.cleanup(n)  # per-chat: only this chat's sys_ids + its router
        print(f"chat {n}: stopped SITL (sys_ids {ip.sysids_for_chat(n)})")
    reg.release(n)
    print(f"chat {n}: released slot")


def main() -> int:
    parser = argparse.ArgumentParser(description="Stop a GCS chat instance (only its own stack).")
    parser.add_argument("--chat", type=int, default=None, help="chat index to stop")
    parser.add_argument("--all", action="store_true",
                        help="stop ALL running instances (explicit broad teardown)")
    parser.add_argument("--eval", action="store_true",
                        help="target this directory's EVAL-band (SITL-only) slot "
                             "instead of its interactive GCS slot")
    parser.add_argument("--list", action="store_true", help="list running instances and exit")
    parser.add_argument("--keep-sitl", action="store_true", help="leave the SITL swarm running")
    args = parser.parse_args()

    if args.list:
        instances = reg.live()
        if not instances:
            print("No running GCS instances.")
        for e in instances:
            print(f"  chat {e['chat_index']}: backend :{e['backend']} sysids {e['sysids']} "
                  f"[{e.get('branch') or '?'} | {e.get('clone') or '?'}]")
        return 0

    if args.all:
        instances = reg.live()
        if not instances:
            print("No running GCS instances to stop.")
        for e in instances:
            stop_chat(e["chat_index"], kill_sitl=not args.keep_sitl)
        return 0

    chat = args.chat
    if chat is None:
        # Default to the stack launched from THIS directory (registry) — no --chat.
        # Scope to the band we mean (interactive unless --eval) so an eval sweep's
        # teardown never stops a concurrent interactive session's slot in the same
        # directory, and vice versa.
        label = "sitl-eval" if args.eval else "gcs"
        entry = reg.find_for_owner(reg.owner_for(str(ROOT)), label=label)
        if entry is None:
            band = "eval" if args.eval else "interactive"
            options = ["--chat N"]
            if not args.eval:
                options.append("--eval (this directory's SITL-only eval slot)")
            options.append("--all")
            print(f"No {band}-band GCS instance for this directory. "
                  f"Use {', '.join(options)} (see --list).", file=sys.stderr)
            return 2
        chat = entry["chat_index"]

    stop_chat(chat, kill_sitl=not args.keep_sitl)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
