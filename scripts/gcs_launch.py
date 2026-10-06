"""Launch an isolated GCS stack (backend + frontend + SITL) on its own free ports.

Each "chat" (separate dev session) gets its own non-colliding ports so several
GCS stacks can run on one machine at once. Everything derives from one chat
index ``N`` (see ``gcs.backend.instance_ports``):

    frontend 3000+N   backend 8000+N   Mission Planner 14550+N   GCS monitor 15550+N

By default this is the *single* command for the whole stack: it starts the
backend, the frontend, **and** the SITL swarm for chat ``N`` (via
``scripts/swarm_run.py``). Pass ``--no-sitl`` to skip SITL (real hardware, or
when you run SITL separately).

The slot is reserved atomically through the shared cross-clone registry
(``gcs.backend.instance_registry``, default ``~/.gcs/instances.json``) so two
sessions launching at once can't grab the same slot, and each session can later
stop **only its own** stack via ``scripts/gcs_stop.py``.

Usage (from the repo root, inside the venv)::

    python scripts/gcs_launch.py            # full stack (backend + frontend + SITL)
    python scripts/gcs_launch.py --no-sitl  # backend + frontend only
    python scripts/gcs_launch.py --chat 2   # force chat index 2
    python scripts/gcs_launch.py --speedup 1  # SITL at real time (default 10x)
    python scripts/gcs_launch.py --list     # show running instances (all clones)
    python scripts/gcs_launch.py --no-browser

``--list`` also reports each slot's SITL verification verdict. The child's own
console closes when it exits, taking its abort message along, so this is where a
failed swarm launch is normally read back (best-effort: if the registry write
itself failed, swarm_run says so on its stderr instead).
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
import webbrowser
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SRC_DIR = ROOT / "src"
FRONTEND_DIR = SRC_DIR / "gcs" / "frontend"

sys.path.insert(0, str(SRC_DIR))
sys.path.insert(0, str(ROOT / "scripts"))  # sibling scripts (swarm_run)
from gcs.backend import instance_ports as ip  # noqa: E402
from gcs.backend import instance_registry as reg  # noqa: E402
import swarm_run  # noqa: E402
from gcs_launch_list import _print_list, _sitl_status  # noqa: E402,F401


def git_branch() -> str:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "--abbrev-ref", "HEAD"], cwd=ROOT,
        ).decode().strip()
    except Exception:
        return ""


def _spawn(cmd, cwd, env, shell=False):
    flags = subprocess.CREATE_NEW_CONSOLE if os.name == "nt" else 0
    return subprocess.Popen(cmd, cwd=str(cwd), env=env, shell=shell, creationflags=flags)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Launch an isolated GCS stack.")
    parser.add_argument("--chat", type=int, default=None,
                        help="force a specific chat index (default: lowest free)")
    parser.add_argument("--list", action="store_true",
                        help="list running instances (across all clones) and exit")
    parser.add_argument("--no-browser", action="store_true",
                        help="do not open the browser")
    parser.add_argument("--sitl", dest="sitl", action="store_true",
                        help="also start the SITL swarm for this chat (default)")
    parser.add_argument("--no-sitl", dest="sitl", action="store_false",
                        help="do not start the SITL swarm (real hardware / run it separately)")
    parser.add_argument(
        "-n",
        "--instances",
        type=int,
        default=ip.VEHICLES_PER_CHAT,
        help=(
            "how many vehicles of this slot's sysid set to launch "
            f"(1..{ip.VEHICLES_PER_CHAT}, default: {ip.VEHICLES_PER_CHAT}). "
            "The slot always reserves its full sysid range; a smaller count "
            "simply leaves the trailing sysids unused, so isolating one "
            "vehicle needs no separate swarm_run invocation."
        ),
    )
    parser.add_argument("--speedup", type=int, default=swarm_run.DEFAULT_SPEEDUP,
                        help="SITL sim time scale (default: "
                             f"{swarm_run.DEFAULT_SPEEDUP}). The swarm launch FAILS "
                             "if the vehicles come up at any other value")
    parser.add_argument(
        "--home",
        default=None,
        metavar="LAT,LON,ALT,HEADING",
        help=(
            "pin the swarm start point (forwarded to swarm_run as HOME_COORDS). "
            "Home is fixed at SITL start, so a mission upload cannot move it."
        ),
    )
    parser.add_argument(
        "--launch-token",
        default=None,
        help="correlate this exact launcher with its SITL registry verdict",
    )
    parser.set_defaults(sitl=True)
    return parser


def _print_banner(entry: dict, args: argparse.Namespace, branch: str) -> None:
    print(f"=== GCS stack ({branch or 'no-branch'}) ===")
    print(f"  frontend        http://localhost:{entry['frontend']}")
    print(f"  backend         http://localhost:{entry['backend']}")
    print(f"  GCS monitor     udp:0.0.0.0:{entry['monitor']}")
    print(f"  Mission Planner udp:0.0.0.0:{entry['mission_planner']}")
    sitl_note = (
        f"SITL: launching {args.instances} at speedup {args.speedup}"
        if args.sitl
        else "SITL: skipped (--no-sitl) — run: python scripts/swarm_run.py"
    )
    # Show which sysids actually fly, not the slot's whole reservation: a
    # partial launch leaves the trailing sysids idle and printing them would
    # send the operator looking for vehicles that were never started.
    flying = list(entry["sysids"])[: args.instances] if args.sitl else []
    sysid_note = (
        f"{entry['sysids']}"
        if not args.sitl or args.instances >= len(entry["sysids"])
        else f"{flying} (slot reserves {entry['sysids']})"
    )
    print(f"  sysids          {sysid_note}  ({sitl_note})")
    print(f"  stop with       python scripts/gcs_stop.py")


def main() -> int:
    args = _build_parser().parse_args()

    if args.list:
        _print_list()
        return 0

    if args.chat is not None and not (0 <= args.chat <= ip.MAX_CHAT_INDEX):
        print(f"chat index must be 0..{ip.MAX_CHAT_INDEX}", file=sys.stderr)
        return 2

    if not (1 <= args.instances <= ip.VEHICLES_PER_CHAT):
        print(
            f"instances must be 1..{ip.VEHICLES_PER_CHAT}",
            file=sys.stderr,
        )
        return 2

    branch = git_branch()
    owner = reg.owner_for(str(ROOT))
    try:
        # Interactive GCS stays in the interactive band [0, EVAL_CHAT_MIN); eval /
        # SITL-only launches use the eval band (see swarm_run.py --eval).
        # launcher_pid makes the claim a launch guard: a second launch of this
        # directory's slot while the first is still spawning (backend not yet
        # bound) is refused atomically instead of spawning a duplicate stack.
        entry = reg.claim(label="gcs", clone=str(ROOT), branch=branch, owner=owner,
                          prefer=args.chat, hi=ip.interactive_chat_hi(),
                          launcher_pid=os.getpid())
    except reg.SlotBusyError as exc:
        entry = exc.entry
        print(f"GCS already running/launching for this directory "
              f"(frontend http://localhost:{entry['frontend']}) — not launching again.")
        print("(stop it with: python scripts/gcs_stop.py)")
        if not args.no_browser:
            webbrowser.open(f"http://localhost:{entry['frontend']}")
        return 0
    except (RuntimeError, TimeoutError) as exc:
        print(f"Could not claim a chat slot: {exc}", file=sys.stderr)
        print("(see running instances with: python scripts/gcs_launch.py --list)", file=sys.stderr)
        return 1

    n = entry["chat_index"]  # internal slot index; not surfaced to the user
    _print_banner(entry, args, branch)

    # Reap any orphan squatting this slot's ports before we spawn fresh ones —
    # e.g. a Vite dev server left by a crashed prior run, or a dead-slot backend
    # still holding the monitor UDP port (it would silently eat this stack's
    # router packets: vehicles look "lost"). The claim above already refused to
    # run if this slot's own stack is alive, and registered pids of other live
    # sessions are spared, so nothing legitimate is killed.
    protected = reg.registered_pids()
    reaped = (
        reg.reap_port(entry["frontend"], exclude=protected)
        + reg.reap_port(entry["backend"], exclude=protected)
        + reg.reap_port(entry["monitor"], kind="udp", exclude=protected)
    )
    reaped += reg.reap_companion_ports(n, exclude=protected)
    for pid in reaped:
        print(f"  reaped orphan pid {pid} on this slot's port")

    try:
        # --- SITL swarm (WSL), unless --no-sitl ---
        # Started first so it boots while the backend comes up; the backend's
        # auto-connect retries until vehicles appear. swarm_run resolves the same
        # chat and tears down only this chat's SITL on exit.
        if args.sitl:
            # Fork capability preflight runs HERE too, not only inside
            # swarm_run: swarm_run gets its own console (CREATE_NEW_CONSOLE)
            # that closes the instant it exits, so its abort message would
            # flash and vanish while backend+frontend come up anyway — the
            # exact silent companion black hole the guard exists to prevent.
            swarm_run.ensure_fork_supports_companion_udp()
            # Pass --speedup explicitly even at the default, so the requested
            # value is visible in the child's command line rather than implied.
            swarm_command = [
                sys.executable,
                str(ROOT / "scripts" / "swarm_run.py"),
                "--chat",
                str(n),
                "--instances",
                str(args.instances),
                "--speedup",
                str(args.speedup),
            ]
            if args.home:
                swarm_command.extend(("--home", args.home))
            if args.launch_token:
                swarm_command.extend(("--launch-token", args.launch_token))
            _spawn(swarm_command,
                   cwd=ROOT, env=os.environ.copy())

        # --- backend (uvicorn) ---
        backend_env = os.environ.copy()
        backend_env["GCS_CHAT_INDEX"] = str(n)
        backend_env["PYTHONPATH"] = str(SRC_DIR)
        backend = _spawn(
            [sys.executable, "-m", "uvicorn", "gcs.backend.main:app",
             "--host", "0.0.0.0", "--port", str(entry["backend"])],
            cwd=ROOT, env=backend_env,
        )
        # Persist the backend pid IMMEDIATELY: if this launcher dies before the
        # frontend spawn below, the launch guard must still see the live (not
        # yet bound) backend process instead of handing the slot to a duplicate.
        reg.record_pids(n, backend_pid=backend.pid)

        # --- frontend (vite) ---
        frontend_env = os.environ.copy()
        frontend_env["PORT"] = str(entry["frontend"])
        frontend_env["VITE_BACKEND_PORT"] = str(entry["backend"])
        frontend_env["VITE_GIT_BRANCH"] = branch
        # npm is npm.cmd on Windows -> run through the shell.
        frontend = _spawn("npm run dev", cwd=FRONTEND_DIR, env=frontend_env, shell=True)
    except BaseException:
        # Don't leave a phantom reservation if spawning failed — including the
        # SystemExit raised by the fork-capability preflight, which `except
        # Exception` would let through.
        reg.release(n)
        raise

    reg.record_pids(n, frontend_pid=frontend.pid)

    if not args.no_browser:
        time.sleep(3)
        webbrowser.open(f"http://localhost:{entry['frontend']}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
