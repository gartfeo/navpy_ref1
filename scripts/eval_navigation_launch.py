"""Isolated SITL evaluator launch and provenance handoff."""

from __future__ import annotations

import json
import os
import secrets
import subprocess
import time
from pathlib import Path
from typing import Any, Callable

from gcs.backend import instance_ports as ip

from eval_navigation_models import LaunchVerdict


LAUNCH_VERDICT_TIMEOUT_S = 600.0
LAUNCHER_LOG_TAIL_BYTES = 800


def stop_own_stack(
    python: Path,
    *,
    worktree: Path,
    chat: int | None = None,
) -> None:
    """Stop this evaluator's exact eval-band stack or saved chat."""
    selector = ["--eval"] if chat is None else ["--chat", str(chat)]
    subprocess.run(
        [str(python), str(worktree / "scripts" / "gcs_stop.py"), *selector],
        cwd=str(worktree),
        text=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        timeout=45,
        check=False,
    )


def build_swarm_command(
    python: Path,
    speedup: float,
    *,
    worktree: Path,
    launch_token: str | None = None,
) -> list[str]:
    """Build the one-vehicle eval supervisor command."""
    command = [
        str(python),
        str(worktree / "scripts" / "swarm_run.py"),
        "--eval",
        "--instances",
        "1",
        "--speedup",
        str(speedup),
    ]
    if launch_token is not None:
        command.extend(("--launch-token", launch_token))
    return command


def launcher_failure_hint(stderr_log: Path | None) -> str:
    """Return a bounded diagnostic tail from a failed launcher's stderr."""
    if stderr_log is None:
        return ""
    try:
        with stderr_log.open("rb") as handle:
            handle.seek(0, os.SEEK_END)
            size = handle.tell()
            handle.seek(max(0, size - LAUNCHER_LOG_TAIL_BYTES))
            raw = handle.read(LAUNCHER_LOG_TAIL_BYTES)
    except OSError:
        return ""
    text = raw.decode("utf-8", "replace").strip()
    if not text:
        return ""
    if size > LAUNCHER_LOG_TAIL_BYTES:
        text = "..." + text
    return f" - launcher stderr ({stderr_log}): {text}"


def wait_for_eval_chat(
    process: subprocess.Popen[Any],
    timeout_s: float,
    *,
    registry: Any,
    worktree: Path,
    speedup: float | None = None,
    stderr_log: Path | None = None,
    launch_token: str | None = None,
    failure_hint: Callable[[Path | None], str] = launcher_failure_hint,
) -> LaunchVerdict:
    """Wait for this exact supervisor's final verified registry generation.

    A supplied per-launch token is authoritative across the Windows virtual-env
    launcher trampoline. Exact PID matching remains the compatibility fallback
    only for callers that supplied no token. A wrong or missing stored token
    therefore fails closed even when the registry PID equals ``Popen.pid``.
    """
    owner = registry.owner_for(str(worktree))
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        entry = registry.find_for_owner(owner, label="sitl-eval")
        ours = _matching_entry(
            entry,
            process_pid=process.pid,
            launch_token=launch_token,
        )
        verdict = ours.get("sitl_verified") if ours is not None else None
        if verdict is False:
            raise RuntimeError(
                "isolated SITL launch failed verification: "
                f"{ours.get('sitl_error') or 'unknown'}"
            )
        if process.poll() is not None:
            raise RuntimeError(
                f"isolated SITL launcher exited code={process.returncode}"
                + failure_hint(stderr_log)
            )
        if verdict is True:
            return _launch_verdict(ours, speedup=speedup)
        time.sleep(0.1)
    raise RuntimeError(
        f"isolated SITL launcher published no verdict within {timeout_s:g}s"
        + failure_hint(stderr_log)
    )


def _matching_entry(
    entry: dict[str, Any] | None,
    *,
    process_pid: int,
    launch_token: str | None,
) -> dict[str, Any] | None:
    if entry is None:
        return None
    if launch_token is not None:
        return entry if entry.get("sitl_launch_token") == launch_token else None
    return entry if entry.get("sitl_pid") == process_pid else None


def _launch_verdict(
    entry: dict[str, Any],
    *,
    speedup: float | None,
) -> LaunchVerdict:
    chat = int(entry["chat_index"])
    if not ip.EVAL_CHAT_MIN <= chat <= ip.MAX_CHAT_INDEX:
        raise RuntimeError(f"launcher claimed non-eval chat {chat}")
    recorded = entry.get("sitl_speedup")
    if speedup is not None and recorded != speedup:
        raise RuntimeError(
            f"isolated SITL launch verified speedup {recorded}, but this "
            f"case asked for {speedup}"
        )
    rates = entry.get("sitl_measured_rates")
    return LaunchVerdict(
        chat=chat,
        requested_speedup=(
            float(recorded) if isinstance(recorded, (int, float)) else speedup
        ),
        measured_rates=(
            {str(key): float(value) for key, value in rates.items()}
            if isinstance(rates, dict)
            else {}
        ),
    )


def start_swarm(
    python: Path,
    run_case_dir: Path,
    speedup: float,
    *,
    worktree: Path,
    command_builder: Callable[..., list[str]],
    wait_for_chat: Callable[..., LaunchVerdict],
) -> tuple[subprocess.Popen[Any], LaunchVerdict]:
    """Start one supervisor and correlate its venv child with one token."""
    launch_token = secrets.token_hex(16)
    command = command_builder(python, speedup, launch_token=launch_token)
    (run_case_dir / "swarm.cmd.json").write_text(
        json.dumps(command, indent=2), encoding="utf-8"
    )
    stdout_handle = (run_case_dir / "swarm.out.log").open("wb")
    stderr_log = run_case_dir / "swarm.err.log"
    stderr_handle = stderr_log.open("wb")
    process = subprocess.Popen(
        command,
        cwd=str(worktree),
        stdout=stdout_handle,
        stderr=stderr_handle,
    )
    verdict = wait_for_chat(
        process,
        LAUNCH_VERDICT_TIMEOUT_S,
        speedup=speedup,
        stderr_log=stderr_log,
        launch_token=launch_token,
    )
    return process, verdict
