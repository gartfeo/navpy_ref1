"""Own the whole sysid span a multi-aircraft launch touches.

A launch of N aircraft numbers them `offset+1 .. offset+N`, and `offset` is
`VEHICLES_PER_CHAT * chat`. Past three aircraft that runs into the FOLLOWING
chats' sysid bands, and none of the shared launcher infrastructure follows it
there:

- `swarm_run_runner.py:145` and `swarm_run_verifier.py:42` both compute
  `sysids_for_chat(chat)[:instances]`, and `sysids_for_chat` returns exactly
  three ids (`instance_ports.py`), so `[:36]` is still three. Only the first
  three aircraft are verified as streaming and clock-checked at launch.
- `swarm_run_wsl.cleanup(chat)` builds its kill patterns from that same
  three-id list, so aircraft four and up SURVIVE teardown. They keep running,
  keep burning CPU, and keep answering on their companion ports.

Both of those are the caller's problem to solve, and the scale probe already
solved them; this module is that solution made reusable so an evaluator does
not have to import a file whose own docstring says to delete it.

The registry claim is the part that matters most for other people. Teardown
kills by sysid pattern and `pkill -f -- --sysid N` does not ask who started the
process, so tearing down a span we do not own would take another session's
aircraft with it. Claiming every chat in the span first, all-or-nothing, is
what makes the teardown safe to perform at all.
"""

from __future__ import annotations

import os
import sys
from collections.abc import Callable
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parent
WORKTREE = SCRIPTS.parent
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))
if str(WORKTREE / "src") not in sys.path:
    sys.path.insert(0, str(WORKTREE / "src"))

import swarm_run_wsl as wsl  # noqa: E402
from gcs.backend import instance_ports as ip  # noqa: E402
from gcs.backend import instance_registry as reg  # noqa: E402


class SpanRefused(RuntimeError):
    """The span is not exclusively ours, so nothing was launched."""


def fleet_sysids(chat: int, instances: int) -> list[int]:
    """The sysids a launch of *instances* aircraft on *chat* will use."""
    offset = ip.VEHICLES_PER_CHAT * chat
    return [offset + index for index in range(1, instances + 1)]


def chats_spanned(sysids: list[int]) -> list[int]:
    """Every chat whose nominal sysids this run would touch."""
    return sorted({(sysid - 1) // ip.VEHICLES_PER_CHAT for sysid in sysids})


def check_span(chat: int, sysids: list[int]) -> None:
    """Refuse a fleet that would address ids no vehicle can hold.

    A MAVLink system id is a uint8. Past 255 the launch produces ids nothing
    answers on, and the failure surfaces much later as aircraft that never
    report -- looking exactly like a capacity ceiling rather than a bad request.
    """
    if sysids[-1] > ip.MAX_SYSID:
        raise ValueError(
            f"{len(sysids)} aircraft from chat {chat} needs sysids up to "
            f"{sysids[-1]}, above MAX_SYSID {ip.MAX_SYSID}"
        )
    span = chats_spanned(sysids)
    if span[-1] > ip.MAX_CHAT_INDEX:
        raise ValueError(
            f"{len(sysids)} aircraft from chat {chat} spans up to chat "
            f"{span[-1]}, above MAX_CHAT_INDEX {ip.MAX_CHAT_INDEX}"
        )


def claim_span(
    sysids: list[int],
    *,
    worktree: Path = WORKTREE,
    supervised_by_launcher: int | None = None,
) -> list[int]:
    """Claim every chat in the blast radius, or take none of them.

    Reading `reg.live()` and then pkilling is check-then-act: another session
    can claim and launch inside that window. `reg.claim` allocates under the
    registry's own lock, which is what makes the later teardown safe.

    `supervised_by_launcher` names the one chat `swarm_run` will launch on.
    That chat is claimed like any other but NOT registered as ours to
    supervise, because `swarm_run` registers itself as its SITL supervisor
    while starting, and `begin_sitl_launch` raises `SitlSupervisorActive` when
    the slot already has a live one -- so doing it here makes the launcher
    refuse to start against our own registration. Its liveness is then the
    launcher's, which is exactly who should own it.
    """
    owner = reg.owner_for(str(worktree))
    wanted = chats_spanned(sysids)
    held: list[int] = []
    for chat in wanted:
        try:
            entry = reg.claim(
                # "sitl-eval" is the only label in `ip.EVAL_LABELS`, and the
                # label is what maps a claim to the eval band rather than to
                # the interactive band the GCS app allocates from. Any other
                # string here would quietly take an operator's slot.
                label="sitl-eval", clone=str(worktree), branch="",
                owner=owner, prefer=chat, lo=chat, hi=chat,
                launcher_pid=os.getpid(),
            )
        except Exception as error:  # noqa: BLE001 - any refusal means: do not run
            release_span(held, worktree=worktree)
            raise SpanRefused(
                f"cannot claim chat {chat} of span {wanted} "
                f"({type(error).__name__}: {error}). Another session most "
                "likely holds it; teardown here kills by sysid and would take "
                "their aircraft down."
            ) from error
        # `chat_index`, not `chat` -- the persisted shape is built by
        # `instance_registry_claims.new_entry`. An earlier guard read
        # `entry["chat"]`, matched nothing, and was silently inert.
        granted = entry.get("chat_index") if isinstance(entry, dict) else None
        if granted != chat:
            release_span(
                held + ([granted] if granted is not None else []), worktree=worktree
            )
            raise SpanRefused(
                f"asked for chat {chat}, registry granted {granted!r}. "
                f"Span {wanted} is not exclusively ours."
            )
        if chat == supervised_by_launcher:
            held.append(chat)
            continue
        # A bare claim does NOT survive the run: `entries.is_alive` consults the
        # startup grace, the backend port and pid, never `launcher_pid`. An
        # evaluator runs no backend, so once the grace expires the entry becomes
        # prunable and a competing claim can take the chat out from under a run
        # still in progress. Registering as the chat's SITL supervisor is the one
        # liveness signal that tracks THIS process for as long as it lives.
        try:
            reg.begin_sitl_launch(chat, supervisor_pid=os.getpid())
        except Exception as error:  # noqa: BLE001 - cannot hold it, do not run
            release_span(held + [chat], worktree=worktree)
            raise SpanRefused(
                f"claimed chat {chat} but could not register as its SITL "
                f"supervisor ({type(error).__name__}: {error}), so the claim "
                "would expire mid-run."
            ) from error
        held.append(chat)
    return held


def release_span(chats: list[int], *, worktree: Path = WORKTREE) -> None:
    """Give back only the chats THIS process still holds.

    `entries.release` is an unconditional pop. If our entry was pruned and
    replaced while we ran, releasing by chat number alone would delete the
    replacing session's entry -- turning our cleanup into their outage.
    """
    mine = reg.owner_for(str(worktree))
    for chat in chats:
        try:
            entry = reg.get(chat)
            if entry is None:
                continue
            if entry.get("owner") != mine or entry.get("sitl_pid") != os.getpid():
                continue
            reg.release(chat)
        except Exception:  # noqa: BLE001 - teardown must not mask the result
            pass


def cleanup_sysids(
    chat: int,
    sysids: list[int],
    *,
    wsl_run: Callable[[str], None] | None = None,
) -> None:
    """Terminate exactly the aircraft this run launched, plus its router.

    NOT `wsl.cleanup(chat)`, which builds its pattern list from
    `sysids_for_chat(chat)` -- always exactly VEHICLES_PER_CHAT entries. A
    fleet deliberately launches past that, so at N>3 the extra aircraft would
    survive teardown and contend with whatever runs next.
    """
    run = wsl.run_wsl if wsl_run is None else wsl_run
    patterns = [wsl.sysid_process_pattern(sysid) for sysid in sysids]
    patterns.append(wsl.router_process_pattern(chat))
    terminate = "; ".join(f"pkill -f -- '{p}' || true" for p in patterns)
    kill = "; ".join(f"pkill -KILL -f -- '{p}' || true" for p in patterns)
    run(f"{terminate}; sleep 2; {kill}")


def hand_back(
    chat: int,
    sysids: list[int],
    held: list[int],
    *,
    release_launcher_chat: Callable[[], None],
    worktree: Path = WORKTREE,
) -> None:
    """Give back the aircraft and then the slots, in that order.

    Aircraft first and by exact sysid, because the shared teardown paths reach
    only `sysids_for_chat(chat)` and would leave a fleet's remainder running.
    Slots second, and from two places: the launcher chat is `swarm_run`'s to
    release, since it registered as that chat's SITL supervisor and
    `release_span` deliberately leaves entries that are not ours alone.

    Every step is attempted even if an earlier one fails -- teardown that stops
    at the first error is how orphans and stuck slots accumulate.
    """
    for step in (
        lambda: cleanup_sysids(chat, sysids),
        release_launcher_chat,
        lambda: release_span(held, worktree=worktree),
    ):
        try:
            step()
        except Exception:  # noqa: BLE001 - teardown must not mask the result
            pass


__all__ = [
    "SpanRefused",
    "chats_spanned",
    "check_span",
    "hand_back",
    "claim_span",
    "cleanup_sysids",
    "fleet_sysids",
    "release_span",
]
