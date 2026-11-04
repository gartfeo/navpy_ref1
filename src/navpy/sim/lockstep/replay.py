"""Replay a run from its action file, the script off (``deterministic_dev_sim.md`` step 12).

The action file (``coordinator_admission.Admission.actions``) holds every
request and cancel the coordinator admitted, scripted and relay alike, with
its tick, id and payload, then the tick the operator returned at (``end``).
A replay feeds each item back at its tick, on the side it came from:

- :class:`ReplayOperator` stands in for the script. In tick k's operator
  phase it returns the script's items of tick k, taking each request's id
  from the shared counter as the script did (ids the script drew without
  sending are skipped; drawing past a recorded id is a divergence). It
  returns at the recorded ``end``; with none, it never returns.
- :class:`ReplayRelay` stands in for the app relay: the admission takes
  the relay's items of tick k after the script's, in their order, and
  gives them their ids again.

So the coordinator admits the same items in the same order with the same
ids, and every participant gets the same ``TICK``. An item left over
(its tick passed and nothing took it) is a :class:`CoordinatorFault`: the
run no longer follows the file. The replay's own action file must equal the
one it fed (:func:`same_actions`).

The script's ``operator`` notes (what its ports saw: receipts, polling,
sleeps) are the one part of a journal a script-off replay cannot have.
:func:`replay_digest` is the journal's chain without them; it is computed
the same way for the run and for its replay, and covers every other record.
"""

from __future__ import annotations

import json
from collections import defaultdict
from collections.abc import Callable, Iterable, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from navpy.sim.lockstep.coordinator_admission import RELAY, SCRIPT, RelayCancel, RelayItem, RelayRequest
from navpy.sim.lockstep.coordinator_journal import RunJournal
from navpy.sim.lockstep.coordinator_types import CoordinatorFault, OperatorCancel, OperatorInbox, OperatorRequest
from navpy.sim.lockstep.operator_journal import OperatorJournal
from navpy.sim.lockstep.part_wire import RequestId
from navpy.sim.lockstep.part_wire_codec import dumps
from navpy.sim.lockstep.participant_hooks import RequestIds

#: The action file's last line: the tick the operator returned at.
END = "end"
#: The journal note only the script makes (``OperatorBaton.observations``).
OPERATOR_NOTE = "operator"


def end_action(k: int) -> dict[str, Any]:
    return {"k": k, "kind": END, "source": SCRIPT}


@dataclass(frozen=True)
class ActionFile:
    #: Every admitted request and cancel, in admission order.
    actions: tuple[dict[str, Any], ...]
    #: The tick the operator returned at; None: the run ended otherwise.
    end: int | None


def load_actions(path: Path) -> ActionFile:
    """Refuses a line that is not an admitted item, items out of tick order,
    and anything after ``end``."""
    actions: list[dict[str, Any]] = []
    end = None
    for number, line in enumerate(path.read_bytes().splitlines(), start=1):
        action = json.loads(line)
        if end is not None:
            raise ValueError(f"{path}:{number}: an action after the end")
        k = action.get("k") if isinstance(action, dict) else None
        if not isinstance(k, int) or isinstance(k, bool) or k < 1:
            raise ValueError(f"{path}:{number}: no tick")
        if actions and k < actions[-1]["k"]:
            raise ValueError(f"{path}:{number}: tick {k} after tick {actions[-1]['k']}")
        if action.get("kind") == END and action.get("source") == SCRIPT:
            end = k
            continue
        _check_item(path, number, action)
        actions.append(action)
    return ActionFile(tuple(actions), end)


def _check_item(path: Path, number: int, action: dict[str, Any]) -> None:
    kind, ident = action.get("kind"), action.get("id")
    if action.get("source") not in (SCRIPT, RELAY) or kind not in ("request", "cancel"):
        raise ValueError(f"{path}:{number}: not an admitted request or cancel")
    if not (isinstance(ident, list) and len(ident) == 2 and all(type(part) is int for part in ident)):
        raise ValueError(f"{path}:{number}: no request id")
    if type(action.get("target")) is not int or (kind == "request") != ("payload" in action):
        raise ValueError(f"{path}:{number}: no target, or a payload on a cancel")


class ActionReplay:
    """The file's items by tick and side, and the tick the run is at."""

    def __init__(self, file: ActionFile) -> None:
        self.end = file.end
        self.k = 0
        self._items: dict[str, dict[int, list[dict[str, Any]]]] = {SCRIPT: defaultdict(list),
                                                                   RELAY: defaultdict(list)}
        for action in file.actions:
            self._items[action["source"]][action["k"]].append(action)

    def take(self, source: str) -> list[dict[str, Any]]:
        """``source``'s items of the current tick; one of an earlier tick is a fault."""
        items = self._items[source]
        late = [k for k in items if k < self.k]
        if late:
            raise CoordinatorFault("replay_diverged", f"{source} items of tick {min(late)} were never "
                                   "admitted", tick=self.k)
        return items.pop(self.k, [])

    def left(self) -> dict[str, int]:
        """Items no admission took, by side."""
        return {source: sum(map(len, items.values())) for source, items in self._items.items()}


class ReplayOperator:
    """An ``OperatorHost`` that sends the script's items from the file.
    ``journal`` keeps the participants' log lines and nothing of the script's
    view, so it journals no ``operator`` note."""

    def __init__(self, ids: RequestIds, replay: ActionReplay) -> None:
        self._ids = ids
        self._replay = replay
        self.journal = OperatorJournal()
        #: The script's requests the run has not answered yet.
        self.open: dict[RequestId, dict[str, Any]] = {}
        self.done = False

    def run_tick(self, inbox: OperatorInbox) -> tuple[OperatorRequest | OperatorCancel, ...]:
        self._replay.k = inbox.k
        self._observe(inbox)
        items: list[OperatorRequest | OperatorCancel] = []
        for action in self._replay.take(SCRIPT):
            request_id = (action["id"][0], action["id"][1])
            if action["kind"] == "cancel":
                items.append(OperatorCancel(action["target"], request_id))
                continue
            self._draw(request_id, inbox.k)
            self.open[request_id] = action["payload"]
            items.append(OperatorRequest(action["target"], request_id, action["payload"]))
        self.done = self._replay.end is not None and inbox.k >= self._replay.end
        return tuple(items)

    def deliver_drain(self, inbox: OperatorInbox) -> None:
        self._observe(inbox)

    def observations(self) -> dict[str, object]:
        return {}

    def _observe(self, inbox: OperatorInbox) -> None:
        for ordinal, line in inbox.log_lines:
            self.journal.log_line(inbox.k, ordinal, line)
        for _ordinal, response in inbox.responses:
            self.open.pop(response.id, None)

    def _draw(self, request_id: RequestId, k: int) -> None:
        while True:
            drawn = self._ids.next()
            if drawn == request_id:
                return
            if drawn[0] != request_id[0] or drawn > request_id:
                raise CoordinatorFault("replay_diverged", f"script request {list(request_id)}: the "
                                       f"shared counter is at {list(drawn)}", tick=k)


class ReplayRelay:
    """A ``RelayPort`` that gives the admission the relay's items from the file.
    Answers are kept by request id; ``broadcast`` is the backend's broadcasts."""

    def __init__(self, target: int, replay: ActionReplay,
                 broadcast: Callable[[str], None] = lambda data: None) -> None:
        self.target = target
        self._replay = replay
        self._on_broadcast = broadcast
        self._tickets: dict[RequestId, int] = {}
        self._ids: list[RequestId] = []
        self.answers: dict[RequestId, object] = {}

    def take(self) -> Sequence[RelayItem]:
        items: list[RelayItem] = []
        for action in self._replay.take(RELAY):
            request_id = (action["id"][0], action["id"][1])
            if action["kind"] == "request":
                self._tickets[request_id] = len(self._ids)
                self._ids.append(request_id)
                items.append(RelayRequest(self._tickets[request_id], action["payload"]))
            elif request_id in self._tickets:
                items.append(RelayCancel(self._tickets[request_id]))
            else:
                raise CoordinatorFault("replay_diverged", f"relay cancel of {list(request_id)}, "
                                       "a request the replay never sent", tick=self._replay.k)
        return items

    def answer(self, ticket: int, answer: object) -> None:
        self.answers[self._ids[ticket]] = answer

    def broadcast(self, data: str) -> None:
        self._on_broadcast(data)

    def open(self) -> list[RequestId]:
        """Relay requests sent and not answered yet."""
        return [request_id for request_id in self._ids if request_id not in self.answers]


def replay_digest(records: Iterable[dict[str, Any]]) -> str:
    """The journal's chain over every record without the script's ``operator`` note."""
    journal = RunJournal()
    for record in records:
        journal.add({key: value for key, value in record.items() if key != OPERATOR_NOTE})
    return journal.digest


def read_journal(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in path.read_bytes().splitlines()]


def first_difference(run: Sequence[dict[str, Any]], replay: Sequence[dict[str, Any]]) -> int | None:
    """The index of the first record (``operator`` notes aside) that differs; None if none does."""
    for index, (a, b) in enumerate(zip(run, replay)):
        if {key: v for key, v in a.items() if key != OPERATOR_NOTE} != \
                {key: v for key, v in b.items() if key != OPERATOR_NOTE}:
            return index
    return None if len(run) == len(replay) else min(len(run), len(replay))


def same_actions(fed: ActionFile, admitted: Sequence[dict[str, Any]]) -> bool:
    """Equal as the action file writes them (canonical JSON)."""
    return [dumps(action) for action in fed.actions] == [dumps(action) for action in admitted]


__all__ = ["ActionFile", "ActionReplay", "END", "OPERATOR_NOTE", "ReplayOperator", "ReplayRelay",
           "end_action", "first_difference", "load_actions", "read_journal", "replay_digest",
           "same_actions"]
