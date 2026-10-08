"""Each helper's current step-4 round, to resolve its owner's APPLIED ack.

A helper repeats its accepted TASK_ASSIGN_RESPONSE ("doing") with a fresh UID
per copy until the owner answers APPLIED, which names one copy by its UID
(docs/design/swarm-task-assignment-ack.md). The GCS handles each packet on
its sender's vehicle link, so a copy and the APPLIED naming it run on
different reader threads and either may come first: an APPLIED that names no
recorded copy waits, one per helper, for the copy it names.
"""
from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import Optional

# A message's UID within its known sender: (boot_id, msg_seq).
Uid = tuple[int, int]


@dataclass(frozen=True)
class AssignedTask:
    """The owner applied the helper's "doing": the helper is ASSIGNED."""

    owner_id: int
    helper_id: int
    task_id: int
    ref: Uid  # the applied step-4 copy
    uid: Uid  # the APPLIED ack itself


@dataclass
class _Round:
    owner_id: int
    task_id: int
    copies: set[Uid] = field(default_factory=set)


@dataclass(frozen=True)
class _Applied:
    owner_id: int
    ref: Uid
    uid: Uid


class AssignRounds:
    """Per helper, its accepted step-4 copies to one owner for one task."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._rounds: dict[int, _Round] = {}
        self._unmatched: dict[int, _Applied] = {}

    def on_accepted_copy(
        self,
        helper_id: int,
        owner_id: int,
        task_id: int,
        uid: Uid,
    ) -> Optional[AssignedTask]:
        """Record one copy; another (owner, task) starts a new round.

        Returns the assignment when an APPLIED already named this copy.
        """
        with self._lock:
            current = self._rounds.get(helper_id)
            if current is None or (current.owner_id, current.task_id) != (
                owner_id, task_id,
            ):
                current = self._rounds[helper_id] = _Round(owner_id, task_id)
            current.copies.add(uid)
            applied = self._unmatched.get(helper_id)
            if applied is None or (applied.owner_id, applied.ref) != (
                owner_id, uid,
            ):
                return None
            del self._unmatched[helper_id]
            return AssignedTask(owner_id, helper_id, task_id, uid, applied.uid)

    def on_applied(
        self,
        owner_id: int,
        helper_id: int,
        ref: Uid,
        uid: Uid,
    ) -> Optional[AssignedTask]:
        """Resolve an owner's APPLIED against the helper's current round."""
        with self._lock:
            current = self._rounds.get(helper_id)
            if (
                current is not None
                and current.owner_id == owner_id
                and ref in current.copies
            ):
                return AssignedTask(
                    owner_id, helper_id, current.task_id, ref, uid,
                )
            self._unmatched[helper_id] = _Applied(owner_id, ref, uid)
            return None


__all__ = ["AssignRounds", "AssignedTask", "Uid"]
