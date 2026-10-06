"""IDE-friendly GCS backend launcher (chat-aware, no hardcoded port).

Claims (or reuses) THIS clone's chat slot from the shared registry, then runs
uvicorn on that chat's backend port. Replaces the old IDE run config that
hardcoded ``GCS_CHAT_INDEX=0`` + ``--port 8000`` (which made every IDE-launched
backend collide on chat 0). Runs uvicorn in-process so the IDE owns/stops it.

In an IDE there is no ``CLAUDE_CODE_SESSION_ID``, so the registry keys the slot by
clone path -- the backend, frontend, and SITL run configs of one project therefore
resolve to the SAME chat.
"""
from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))
from gcs.backend import instance_ports as ip  # noqa: E402
from gcs.backend import instance_registry as reg  # noqa: E402


def _branch() -> str:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "--abbrev-ref", "HEAD"], cwd=ROOT).decode().strip()
    except Exception:
        return ""


def resolve_slot() -> dict:
    """Claim or reuse this clone/session's chat slot. Refuses while the slot's
    previous launch is still alive so an IDE-launched backend can never
    duplicate a running (or still-spawning) stack."""
    owner = reg.owner_for(str(ROOT))
    try:
        # Interactive (runs a frontend) -> stay in the interactive band.
        # uvicorn runs in-process, so our own PID guards the slot for the
        # backend's entire lifetime, not just the spawn window.
        return reg.claim(label="gcs", clone=str(ROOT), branch=_branch(), owner=owner,
                         hi=ip.interactive_chat_hi(), launcher_pid=os.getpid())
    except reg.SlotBusyError as exc:
        e = exc.entry
        raise RuntimeError(
            f"GCS backend already running/launching for this project on chat "
            f"{e['chat_index']} (:{e['backend']}). Stop it first "
            f"(python scripts/gcs_stop.py).") from exc


def main() -> int:
    try:
        entry = resolve_slot()
    except (RuntimeError, TimeoutError) as exc:
        print(exc, file=sys.stderr)
        return 1
    n = entry["chat_index"]
    reg.record_pids(n, backend_pid=os.getpid())
    print(f"=== GCS backend: chat {n} on http://localhost:{entry['backend']} "
          f"(monitor udp:0.0.0.0:{entry['monitor']}, sysids {entry['sysids']}) ===")
    os.environ["GCS_CHAT_INDEX"] = str(n)
    os.environ.setdefault("PYTHONPATH", str(ROOT / "src"))
    import uvicorn
    uvicorn.run("gcs.backend.main:app", host="0.0.0.0", port=entry["backend"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
