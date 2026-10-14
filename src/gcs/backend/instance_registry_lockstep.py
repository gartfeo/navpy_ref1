"""Lockstep slot ownership: claim, children under the lock, compare-release.

``docs/design/lockstep_slice.md``, "Processes and ownership". A lockstep
entry (label ``lockstep``, eval band, no frontend) carries::

    "lockstep": {"owner": {pid, start, run_token, nonce},
                 "phase": "active" | "stopping",
                 "children": [{"kind": "local", pid, start, ...} |
                              {"kind": "wsl", wsl_pid, start_ticks, cwd, root, ...}]}

- The owner identity is complete: the run token repeats per seed, so a nonce
  of host entropy, fresh per claim, tells two claims apart.
- A child is recorded under the registry lock only while the entry is
  ``active`` and still this owner's; the coordinator sends ``go`` only after.
- ``stopping`` freezes the child inventory: nothing more is recorded.
- Release compares the full identity under the lock (:func:`release_if_owner`).
- Liveness (``instance_registry_entries.lockstep_alive``) keeps an entry while
  its owner or any recorded child lives, so a claim cannot take its slot.
The registry is host-side; none of this reaches a run journal.
"""

from __future__ import annotations

import copy
import os
import secrets
from typing import Any, Callable

from gcs.backend import instance_ports as ip
from gcs.backend import instance_registry_claims as claims
from gcs.backend import instance_registry_entries as entries
from gcs.backend import instance_registry_runtime as runtime
from gcs.backend import instance_registry_store as store

LOCKSTEP_LABEL = "lockstep"
ACTIVE, STOPPING = "active", "stopping"
#: Bytes of host entropy in a claim nonce.
NONCE_BYTES = 16


def claim(
    *,
    run_token: str,
    clone: str,
    owner: str,
    pid: int | None = None,
    port_check: Callable[[int], bool] = runtime.slot_ports_free,
    pid_start: Callable[[object], int | None] = runtime.pid_start_time,
    alive_check: Callable[[dict[str, Any]], bool] = entries.is_alive,
    access: store.StoreAccess = store.DEFAULT_ACCESS,
) -> tuple[int, dict[str, Any]]:
    """Claim a free eval-band slot for this process; return it and the owner identity."""
    pid = os.getpid() if pid is None else pid
    identity = {"pid": pid, "start": pid_start(pid), "run_token": run_token,
                "nonce": secrets.token_hex(NONCE_BYTES)}
    lo, hi = ip.eval_band()
    with access.locked():
        data = entries.prune(access.load(), alive_check=alive_check)
        taken = {int(key) for key in data["instances"]}
        chat = next((n for n in range(lo, hi + 1) if n not in taken and port_check(n)), None)
        if chat is None:
            raise RuntimeError(f"no free lockstep slot in {lo}..{hi}")
        entry = claims.new_entry(chat, LOCKSTEP_LABEL, clone, "", owner, pid_start=pid_start)
        entry["lockstep"] = {"owner": identity, "phase": ACTIVE, "children": []}
        data["instances"][str(chat)] = entry
        access.save(data)
    return chat, identity


def _owned(data: dict[str, Any], chat: int, identity: dict[str, Any]) -> dict[str, Any] | None:
    entry = data["instances"].get(str(chat))
    if entry is None or (entry.get("lockstep") or {}).get("owner") != identity:
        return None
    return entry


def record_child(
    chat: int,
    identity: dict[str, Any],
    child: dict[str, Any],
    *,
    access: store.StoreAccess = store.DEFAULT_ACCESS,
) -> bool:
    """Record a held child; False (the caller kills it before ``go``) unless ``active``."""
    with access.locked():
        data = access.load()
        entry = _owned(data, chat, identity)
        if entry is None or entry["lockstep"]["phase"] != ACTIVE:
            return False
        entry["lockstep"]["children"].append(dict(child))
        access.save(data)
        return True


def forget_child(
    chat: int,
    identity: dict[str, Any],
    pid: int,
    *,
    access: store.StoreAccess = store.DEFAULT_ACCESS,
) -> None:
    """Drop the record of a child the owner killed before admitting it."""
    with access.locked():
        data = access.load()
        entry = _owned(data, chat, identity)
        if entry is None:
            return
        children = entry["lockstep"]["children"]
        children[:] = [c for c in children if not (c.get("kind") == "local" and c.get("pid") == pid)]
        access.save(data)


def mark_stopping(
    chat: int, *, access: store.StoreAccess = store.DEFAULT_ACCESS,
) -> dict[str, Any] | None:
    """Freeze the entry's child inventory; return the frozen entry (None if gone)."""
    with access.locked():
        data = access.load()
        entry = data["instances"].get(str(chat))
        if entry is None or entry.get("lockstep") is None:
            return None
        entry["lockstep"]["phase"] = STOPPING
        access.save(data)
        return copy.deepcopy(entry)


def release_if_owner(
    chat: int,
    identity: dict[str, Any],
    *,
    access: store.StoreAccess = store.DEFAULT_ACCESS,
) -> bool:
    """Remove the entry only if it still carries exactly this owner identity."""
    with access.locked():
        data = access.load()
        if _owned(data, chat, identity) is None:
            return False
        del data["instances"][str(chat)]
        access.save(data)
        return True


def local_child(pid: int, *, pid_start: Callable[[object], int | None] = runtime.pid_start_time,
                **facts: object) -> dict[str, Any]:
    """A local child's record; it is held before ``go``, so its pid cannot be reused yet."""
    return {"kind": "local", "pid": pid, "start": pid_start(pid), **facts}


class RegistryChildRecorder:
    """The coordinator's recorder (``coordinator_spawn.ChildRecorder``) for one claim."""

    def __init__(self, chat: int, identity: dict[str, Any],
                 access: store.StoreAccess = store.DEFAULT_ACCESS) -> None:
        self._chat = chat
        self._identity = identity
        self._access = access

    def record(self, child: Any, spawn: Any) -> bool:
        facts = local_child(child.pid, spawn_id=list(spawn.spawn_id), role=spawn.role,
                            generation=spawn.generation)
        return record_child(self._chat, self._identity, facts, access=self._access)

    def forget(self, child: Any, spawn: Any) -> None:
        forget_child(self._chat, self._identity, child.pid, access=self._access)


__all__ = [
    "ACTIVE", "LOCKSTEP_LABEL", "RegistryChildRecorder", "STOPPING", "claim", "forget_child",
    "local_child",
    "mark_stopping", "record_child", "release_if_owner",
]
