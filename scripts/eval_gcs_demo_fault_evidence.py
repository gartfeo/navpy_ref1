"""One fault run's evidence, and the questions the fault gates ask of it (step 7)."""

from __future__ import annotations

import json
from collections import deque
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass
from datetime import datetime
from math import inf
from pathlib import Path
from typing import Any

from navpy.sim.lockstep.coordinator_types import backend_ordinal
from navpy.sim.lockstep.fault_schedule import PRESENCE_MARKER
from scripts.eval_gcs_demo_constants import GCS_LOG_SUBDIR
from scripts.eval_gcs_demo_fault_schedule import FaultSpan, Lines
from scripts.eval_gcs_demo_fleet_packets import packet_events
from scripts.eval_gcs_demo_sitl_root import SITL_START_TIME_UTC

EPOCH_US = SITL_START_TIME_UTC * 1_000_000


@dataclass(frozen=True)
class Delivery:
    k: int
    sent_k: int  # when its first sender emitted it, however many hops and queues it crossed since
    source: tuple[int, int]
    msgid: int


def _send_key(event: Sequence[Any]) -> Any:
    """What stays the same across a packet's hops: its NAVLINK UID, else its bytes."""
    return tuple(event[7]["uid"]) if len(event) > 7 and "uid" in event[7] else event[3]


def _sitl_of(device: str) -> str:
    """The SITL that receives on ``device``: ``sitl<i>.serial1`` or ``lockstep:serial0.<i>``."""
    return device.split(".")[0] if device.startswith("sitl") else f"sitl{device.rsplit('.', 1)[1]}"


def deliveries(records: Sequence[Mapping[str, Any]]) -> dict[Any, list[Delivery]]:
    """Receiver (a participant's ordinal or a SITL's device) -> what it got, in tick order."""
    roots: dict[tuple, tuple[Sequence[Any], int]] = {}  # packet -> (the emission it came from, when first sent)
    #: A UID is one send, however often it is resent.
    uid_sent: dict[Any, int] = {}
    #: (SITL, bytes) -> when each copy it got and has not passed on was first sent, first in, first out:
    #: a SITL passes bytes on unchanged. Anything else it emits is its own send, as bytes repeat once a
    #: sequence byte wraps. A copy it never passed on only makes a later one of the same bytes look older.
    waiting: dict[tuple[str, Any], deque[int]] = {}
    delivered: dict[Any, list[Delivery]] = {}
    for k, event in packet_events(records):
        packet = tuple(event[1])
        if event[0] == "emitted":
            key = _send_key(event)
            if isinstance(key, str):
                queue = waiting.get((packet[0], key))
                roots[packet] = (event, queue.popleft() if queue else k)
            else:
                roots[packet] = (event, uid_sent.setdefault(key, k))
        elif event[0] == "copy":
            root = roots.get(tuple(event[2]))
            if root is not None:
                roots[packet] = root
        elif event[0] == "delivered" and packet in roots:
            root, sent_k = roots[packet]
            receiver = event[2][0] if isinstance(event[2], list) else event[2]
            if isinstance(receiver, str):
                waiting.setdefault((_sitl_of(receiver), _send_key(root)), deque()).append(sent_k)
            delivered.setdefault(receiver, []).append(Delivery(k, sent_k, tuple(root[5]), root[4]))
    return delivered


def timeline_events(log_dir: Path, name: str) -> dict[int, list[tuple[int, str]]]:
    """System -> the backend's ``name`` changes ``(t_us, outcome)``, from its timeline."""
    events: dict[int, list[tuple[int, str]]] = {}
    for path in sorted((log_dir / GCS_LOG_SUBDIR / "sessions").glob("*/timeline.jsonl")):
        for row in path.read_text(encoding="utf-8").splitlines():
            event = json.loads(row)
            if event.get("event") == name:
                t_us = round(datetime.fromisoformat(event["ts"]).timestamp() * 1e6) - EPOCH_US
                events.setdefault(event["sys_id"], []).append((t_us, event["outcome"]))
    return events


class FaultRun:
    """One run's evidence, and the questions the gates ask of it."""

    def __init__(self, records: Sequence[Mapping[str, Any]], lines: Lines,
                 links: Mapping[int, Sequence[tuple[int, str]]],
                 companions: Mapping[int, Sequence[tuple[int, str]]]) -> None:
        self.records = records
        self.lines = lines
        self.links = links
        self.companions = companions
        self.sys_ids = tuple(sorted(lines))
        self.backend = backend_ordinal(len(self.sys_ids))
        self.times = {record["k"]: record["t_us"] for record in records if "t_us" in record}
        self.delivered = deliveries(records)
        self.lifecycle = [(record["k"], entry) for record in records for entry in record.get("lifecycle", ())]

    def by_tick(self, span: FaultSpan) -> int:
        """The last tick no later than the fault's deadline."""
        return max((k for k, t_us in self.times.items() if t_us <= span.deadline_us), default=0)

    def received(self, receiver: Any, sender: int, msgids: Iterable[int] | None,
                 component: int | None = None) -> list[Delivery]:
        """What ``receiver`` got of ``sender``'s, in tick order."""
        return [d for d in self.delivered.get(receiver, ())
                if d.source[0] == sender and (component is None or d.source[1] == component)
                and (msgids is None or d.msgid in msgids)]

    def heard(self, receiver: Any, sender: int, msgids: Iterable[int] | None, after: int, by: int,
              component: int | None = None) -> int | None:
        """The first tick ``receiver`` got a packet of ``sender``'s sent after tick ``after``, by tick ``by``."""
        return next((d.k for d in self.received(receiver, sender, msgids, component)
                     if d.sent_k > after and d.k <= by), None)

    def gap_s(self, ticks: Sequence[int], heal_k: int) -> tuple[int | None, int | None, float]:
        """The gap in ``ticks`` over the heal: the last at or before it, the first after, seconds between."""
        before = max((k for k in ticks if k <= heal_k), default=None)
        after = min((k for k in ticks if k > heal_k), default=None)
        if before is None or after is None:
            return before, after, inf
        return before, after, (self.times[after] - self.times[before]) / 1e6

    def line_at(self, sys_id: int, text: str, after: int, by: int) -> int | None:
        """The first tick ``sys_id`` logged ``text`` after tick ``after``, by tick ``by``."""
        return next((k for k, line in self.lines[sys_id] if after < k <= by and text in line), None)

    def spawned_at(self, uav: int, k: int) -> int:
        """The tick ``uav``'s generation at tick ``k`` was spawned; 0 for one from the start of the record."""
        return max((at for at, entry in self.lifecycle if entry["kind"] == "spawn" and entry["ordinal"] == uav
                    and entry["result"] == "ok" and at <= k), default=0)

    def present(self, uav: int, k: int) -> bool:
        """``uav``'s generation at tick ``k`` had logged its check-in by then."""
        return self.line_at(uav, PRESENCE_MARKER, self.spawned_at(uav, k), k) is not None

    def companion_at(self, sys_id: int, t_us: int) -> str | None:
        """The backend's companion status of ``sys_id`` at ``t_us``: its last change by then."""
        return next((status for at, status in reversed(self.companions.get(sys_id, ())) if at <= t_us), None)


__all__ = ["Delivery", "FaultRun", "deliveries", "timeline_events"]
