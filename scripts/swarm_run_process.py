"""WSL launcher-process and output-relay lifecycle."""

from __future__ import annotations

import subprocess
import threading
import time
from typing import Callable, Sequence

import swarm_run_relay as relay_io


def start_launcher(
    command: str,
    *,
    is_regular_file: Callable[[int], bool],
    wsl_argv: Callable[[str], list[str]],
    relay_stream: Callable[[relay_io.Relay], None],
    retire: Callable[[relay_io.Relay], None],
) -> tuple[subprocess.Popen[bytes], list[relay_io.Relay]]:
    """Start WSL and relay only descriptors vulnerable to file-position loss."""
    piped = {1: is_regular_file(1), 2: is_regular_file(2)}
    process = subprocess.Popen(
        wsl_argv(command),
        stdout=subprocess.PIPE if piped[1] else None,
        stderr=subprocess.PIPE if piped[2] else None,
    )
    relays: list[relay_io.Relay] = []
    try:
        streams = (
            (1, "stdout", process.stdout),
            (2, "stderr", process.stderr),
        )
        for fd, name, source in streams:
            if not piped[fd] or source is None:
                continue
            relay = relay_io.Relay(fd=fd, name=name, source=source)
            relay.thread = threading.Thread(
                target=relay_stream,
                args=(relay,),
                name=f"relay-{name}",
                daemon=True,
            )
            relay.thread.start()
            relays.append(relay)
    except BaseException:
        for relay in relays:
            retire(relay)
        try:
            process.kill()
            process.wait(timeout=5)
        except Exception:
            pass
        raise
    return process, relays


def relay_warning(
    relay: relay_io.Relay,
    timed_out: bool,
    *,
    reaped: bool,
    drain_timeout: float,
) -> str | None:
    """Return one line describing all capture faults on a relay."""
    parts: list[str] = []
    if relay.error is not None:
        parts.append(f"destination write failed ({relay.error})")
    if relay.source_error is not None:
        parts.append(
            f"source read failed ({relay.source_error}), "
            "so captured output may be truncated"
        )
    if relay.discarded:
        parts.append(f"discarded {relay.discarded} bytes so far")
    if timed_out:
        reason = (
            "wsl.exe was reaped, so it was expected to be at EOF already"
            if reaped
            else "the launcher could not be reaped, so the relay was silenced"
        )
        parts.append(
            "LOG CAPTURE INCOMPLETE: did not reach EOF within "
            f"{drain_timeout:g}s ({reason}), so this launch's log may be "
            "missing its tail"
        )
    if not parts:
        return None
    return f"{relay.name} relay warning: " + "; ".join(parts)


def warn_about_relay(
    chat: int,
    relay: relay_io.Relay,
    timed_out: bool,
    *,
    reaped: bool,
    warning: Callable[..., str | None],
    emit: Callable[..., None],
) -> None:
    """Report a relay problem without allowing logging to alter the verdict."""
    message = warning(relay, timed_out, reaped=reaped)
    if message is None:
        return
    for err in (relay.fd == 1, relay.fd != 1):
        try:
            emit(f"SITL chat {chat}: {message}", err=err)
            return
        except Exception:
            continue


def retire_relays(
    chat: int,
    relays: Sequence[relay_io.Relay],
    *,
    reaped: bool,
    drain_timeout: float,
    retire: Callable[[relay_io.Relay], None],
    warn: Callable[..., None],
) -> None:
    """Bound relay draining, silence unfinished writers, and report faults."""
    deadline = time.monotonic() + drain_timeout
    for relay in relays:
        if relay.thread is not None:
            relay.thread.join(timeout=max(0.0, deadline - time.monotonic()))
    for relay in relays:
        timed_out = relay.thread is not None and relay.thread.is_alive()
        if timed_out:
            retire(relay)
        warn(chat, relay, timed_out, reaped=reaped)


def kill_launch(
    process: subprocess.Popen[bytes],
    chat: int,
    relays: Sequence[relay_io.Relay],
    *,
    cleanup: Callable[[int], None],
    retire_relays: Callable[..., None],
) -> None:
    """Tear down one failed launch and then retire its output relays."""
    cleanup(chat)
    reaped = False
    try:
        process.terminate()
        process.wait(timeout=5)
        reaped = True
    except Exception:
        try:
            process.kill()
            process.wait(timeout=5)
            reaped = True
        except Exception:
            pass
    retire_relays(chat, relays, reaped=reaped)
