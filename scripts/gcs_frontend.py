"""IDE-friendly GCS frontend launcher (chat-aware, no hardcoded port).

Resolves (or claims) THIS clone's chat slot from the shared registry, then runs
``npm run dev`` (Vite) on that chat's frontend port, pointed at the same chat's
backend. Replaces the old npm IDE config that always used :3000 / backend :8000,
so several IDE frontends don't collide. Keyed by clone path (no session id in an
IDE), so it lands on the same chat as ``gcs_backend.py``.
"""
from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
FRONTEND_DIR = ROOT / "src" / "gcs" / "frontend"
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
    """Reuse this clone/session's chat if one exists, else claim one."""
    owner = reg.owner_for(str(ROOT))
    return reg.find_for_owner(owner, label="gcs") or reg.claim(
        label="gcs", clone=str(ROOT), branch=_branch(), owner=owner,
        hi=ip.interactive_chat_hi())


def main() -> int:
    entry = resolve_slot()
    n = entry["chat_index"]
    reg.record_pids(n, frontend_pid=os.getpid())
    env = os.environ.copy()
    env["PORT"] = str(entry["frontend"])
    env["VITE_BACKEND_PORT"] = str(entry["backend"])
    env["VITE_GIT_BRANCH"] = _branch()
    print(f"=== GCS frontend: chat {n} on http://localhost:{entry['frontend']} "
          f"-> backend :{entry['backend']} ===")
    return subprocess.run("npm run dev", cwd=str(FRONTEND_DIR), env=env, shell=True).returncode


if __name__ == "__main__":
    raise SystemExit(main())
