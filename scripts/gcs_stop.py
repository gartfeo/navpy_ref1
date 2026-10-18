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
from gcs_launch_video import reap_sim_video_ports  # noqa: E402
from gcs_stop_lockstep import stop_lockstep  # noqa: E402


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


def stop_chat(n: int, kill_sitl: bool = True) -> bool:
    """Stop chat *n*'s stack; False when its SITL could not be reaped (slot kept)."""
    entry = reg.get(n)
    if entry is not None and entry.get("lockstep") is not None:
        stop_lockstep(n, kill=_kill_tree)  # by ownership records, never by sysid or port
        return True
    if entry is None:
        print(f"chat {n}: no registry entry (already stopped?)")
    else:
        backend_pid = _verified_pid(entry, "backend_pid", "backend_pid_start")
        frontend_pid = _verified_pid(entry, "frontend_pid", "frontend_pid_start")
        video_pid = _verified_pid(entry, "video_pid", "video_pid_start")
        _kill_tree(backend_pid)
        _kill_tree(frontend_pid)
        _kill_tree(video_pid)  # sim MediaMTX; the tree takes its FFmpeg children
        print(f"chat {n}: stopped backend pid={backend_pid} "
              f"frontend pid={frontend_pid} video pid={video_pid}")
    # Reap anything still holding this chat's frontend/backend ports — e.g. a Vite
    # dev server that outlived the pid tree-kill. Otherwise it orphans onto the
    # port, stays invisible to the registry, and blocks the next session's slot.
    # Companion UDP ports too (shared helper — see reap_companion_ports).
    protected = reg.registered_pids()
    reaped = reg.reap_port(ip.frontend_port(n)) + reg.reap_port(ip.backend_port(n))
    reaped += reap_sim_video_ports(n, set())
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
        try:
            swarm_run.cleanup(n)  # by ownership: this chat's SWARM_OFFSET, sys_ids, router
        except (RuntimeError, OSError) as error:  # a failed reap, or WSL did not start
            print(f"chat {n}: SITL NOT stopped, slot kept: {error}", file=sys.stderr)
            return False
        print(f"chat {n}: stopped SITL (sys_ids {ip.sysids_for_chat(n)})")
    reg.release(n)
    print(f"chat {n}: released slot")
    return True


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
        results = [stop_chat(e["chat_index"], kill_sitl=not args.keep_sitl) for e in instances]
        return 0 if all(results) else 1

    chat = args.chat
    if chat is None:
        # Default to the stack launched from THIS directory (registry) — no --chat.
        # Scope to the band we mean (interactive unless --eval) so an eval sweep's
        # teardown never stops a concurrent interactive session's slot in the same
        # directory, and vice versa.
        label = "sitl-eval" if args.eval else "gcs"
        owner = reg.owner_for(str(ROOT))
        entry = reg.find_for_owner(owner, label=label)
        if entry is None and args.eval:
            # Only with no wall eval slot: this directory's lockstep slot (by ownership).
            entry = reg.find_for_owner(owner, label=label, include_lockstep=True)
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

    return 0 if stop_chat(chat, kill_sitl=not args.keep_sitl) else 1


if __name__ == "__main__":
    raise SystemExit(main())
