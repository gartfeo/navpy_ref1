"""A small fleet journal with a scheduled fault, in the shapes ``coordinator_packets`` writes.

One tick a second. Each tick every NavPy sends a heartbeat and, once its generation
checked in, a swarm heartbeat; each autopilot sends a heartbeat to the backend; the
backend sends a heartbeat to the fleet. The router and the SITLs carry them within
the tick, unless a cut ends them ``radio_fault``. Each send has its own bytes, the same
on every hop. A run passes the gates as built; a test then breaks one thing.
"""

from __future__ import annotations

import json
from collections.abc import Collection
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

from pymavlink.dialects.v20 import ardupilotmega as mavlink

from gcs.backend.vehicle_status import COMPANION_DOWN_WINDOW_S, COMPANION_OK_WINDOW_S
from navpy.sim.lockstep.coordinator_types import ROUTER_DEVICE, serial0_device, serial1_device
from navpy.sim.lockstep.fault_schedule import FENCE_MARKER, LAUNCH_MARKER, PRESENCE_MARKER, FaultPlan
from scripts.eval_gcs_demo_constants import GCS_LOG_SUBDIR
from scripts.eval_gcs_demo_fault_evidence import deliveries
from scripts.eval_gcs_demo_fault_schedule import NAV_MARKER
from scripts.eval_gcs_demo_sitl_root import SITL_START_TIME_UTC

S = 1_000_000
FLEET = (1, 2, 3)
BACKEND = 4
LAST_TICK = 80
LAUNCH_K = 2
PRESENCE_K = 5
FENCE_K = 70
NAV_K = 78
HEARTBEAT = mavlink.MAVLINK_MSG_ID_HEARTBEAT
SWARM_HEARTBEAT = mavlink.MAVLINK_MSG_ID_SWARM_HEARTBEAT
AUTOPILOT = mavlink.MAV_COMP_ID_AUTOPILOT1
COMPANION = mavlink.MAV_COMP_ID_ONBOARD_COMPUTER
GCS = (255, 0)


@dataclass(frozen=True)
class _Send:
    """One send: what every hop of it carries unchanged."""

    msgid: int
    sender: tuple[int, int]
    uid: tuple[int, int, int] | None
    sha: str


class FaultJournal:
    """The journal, built tick by tick from what each endpoint does."""

    def __init__(self, plans: tuple[FaultPlan, ...]) -> None:
        self.records: list[dict] = [{"grid": {"faults": [plan.to_json() for plan in plans]}}]
        self.records += [{"k": k, "t_us": k * S, "packets": []} for k in range(1, LAST_TICK + 1)]
        self.lines: dict[int, list[tuple[int, str]]] = {i: [] for i in FLEET}
        self.lines[1] += [(LAUNCH_K, f"x {LAUNCH_MARKER} 1"), (PRESENCE_K, PRESENCE_MARKER)]
        self.lines[2] += [(PRESENCE_K, PRESENCE_MARKER), (FENCE_K, f"x {FENCE_MARKER}1"), (NAV_K, NAV_MARKER)]
        self.lines[3] += [(PRESENCE_K, PRESENCE_MARKER)]
        self._seq = 0
        self._sends = 0
        #: Who is not sending now, and what is cut, by the tick.
        self.silent_navpy: dict[int, Collection[int]] = {}
        self.silent_sources: dict[str, Collection[int]] = {}
        self.cut_sources: dict[str, tuple[Collection[int], str]] = {}
        self.cut_paths: dict[tuple[str, str], tuple[Collection[int], str]] = {}
        #: A restarted NavPy's new generation sends from this tick, with a new boot id unless ``same_boot``.
        self.respawn: dict[int, int] = {}
        self.same_boot = False

    def generation(self, i: int, k: int) -> int:
        return 2 if k >= self.respawn.get(i, LAST_TICK + 1) else 1

    def boot(self, i: int, k: int) -> int:
        return 100 * (1 if self.same_boot else self.generation(i, k)) + i

    def record(self, k: int) -> dict:
        return self.records[k]

    def _checked_in(self, i: int, k: int) -> bool:
        """Its generation at tick k has logged its check-in: its swarm heartbeats start then."""
        spawned = self.respawn.get(i, 0)
        since = spawned if k >= spawned else 0
        return any(since <= at <= k and PRESENCE_MARKER in line for at, line in self.lines[i])

    def fault(self, k: int, *event) -> None:
        self.record(k).setdefault("faults", []).append(list(event))

    def lifecycle(self, k: int, kind: str, ordinal: int, generation: int, payload=None, result=None) -> None:
        self.record(k).setdefault("lifecycle", []).append(
            {"kind": kind, "ordinal": ordinal, "generation": generation, "payload": payload, "result": result})

    def _event(self, k: int, *event) -> None:
        self.record(k)["packets"].append(list(event))

    def _send(self, msgid: int, sender: tuple[int, int], uid=None) -> _Send:
        self._sends += 1
        return _Send(msgid, sender, uid, f"{self._sends:064x}")

    def _emit(self, k: int, origin, generation: int, device: str, send: _Send) -> list:
        self._seq += 1
        packet = [origin, generation, device, self._seq]
        event = ["emitted", packet, [0, 1], send.sha, send.msgid, list(send.sender), [0, 0]]
        if send.uid is not None:
            event.append({"uid": list(send.uid)})
        self._event(k, *event)
        return packet

    def _cut(self, table: dict, key, k: int) -> str | None:
        ticks, fault = table.get(key, ((), ""))
        return fault if k in ticks else None

    def _route(self, k: int, source: str, up: list, send: _Send, to: tuple[str, ...]) -> None:
        cut = self._cut(self.cut_sources, source, k)
        if cut:
            self._event(k, "radio_fault", up, cut)
            return
        for destination in to:
            self._seq += 1
            device = ROUTER_DEVICE if destination == "backend" else serial1_device(int(destination[4:]))
            copy = ["router", 1, device, self._seq]
            self._event(k, "copy", copy, up)
            cut = self._cut(self.cut_paths, (source, destination), k)
            if cut:
                self._event(k, "radio_fault", copy, cut)
            elif destination == "backend":
                self._event(k, "delivered", copy, [BACKEND, 1])
            else:
                self._event(k, "delivered", copy, device)
                receiver = int(destination[4:])
                down = self._emit(k, f"sitl{receiver}", 1, serial0_device(receiver), send)
                self._event(k, "delivered", down, [receiver, self.generation(receiver, k)])
        self._event(k, "routed", up, len(to))

    def _navpy_sends(self, k: int, i: int, msgid: int, uid) -> None:
        send = self._send(msgid, (i, COMPANION), uid)
        sent = self._emit(k, i, self.generation(i, k), serial0_device(i), send)
        self._event(k, "delivered", sent, serial0_device(i))
        up = self._emit(k, f"sitl{i}", 1, serial1_device(i), send)
        peers = tuple(f"sitl{j}" for j in FLEET if j != i)
        self._route(k, f"sitl{i}", up, send, (*peers, "backend"))

    def _tick(self, k: int) -> None:
        for i in FLEET:
            if k in self.silent_navpy.get(i, ()):
                continue
            self._navpy_sends(k, i, HEARTBEAT, None)
            if self._checked_in(i, k):
                self._navpy_sends(k, i, SWARM_HEARTBEAT, (i, self.boot(i, k), k))
        for i in FLEET:
            if k not in self.silent_sources.get(f"sitl{i}", ()):
                send = self._send(HEARTBEAT, (i, AUTOPILOT))
                up = self._emit(k, f"sitl{i}", 1, serial1_device(i), send)
                self._route(k, f"sitl{i}", up, send, ("backend",))
        if k not in self.silent_sources.get("backend", ()):
            send = self._send(HEARTBEAT, GCS)
            up = self._emit(k, BACKEND, 1, ROUTER_DEVICE, send)
            self._route(k, "backend", up, send, tuple(f"sitl{i}" for i in FLEET))

    def build(self) -> list[dict]:
        for k in range(1, LAST_TICK + 1):
            self._tick(k)
        return self.records

    def forward_stale(self, k: int, origin, before: int, sender: tuple[int, int], device: str, to) -> None:
        """At tick k a SITL emits and delivers to ``to`` bytes of ``sender``'s that ``origin`` last sent
        before tick ``before``: it got them then and held them in a queue. Call it after :meth:`build`."""
        at, event = next((record["k"], event) for record in reversed(self.records[1:before])
                         for event in record["packets"]
                         if event[0] == "emitted" and event[1][0] == origin and tuple(event[5]) == sender)
        self._event(at, "delivered", event[1], "sitl9.serial1")
        self._seq += 1
        stale = ["sitl9", 1, device, self._seq]
        self._event(k, "emitted", stale, *event[2:])
        self._event(k, "delivered", stale, to)

    def companions(self) -> dict[int, list[tuple[int, str]]]:
        """The backend's companion status of each UAV, one row per change: a pure function of the
        newest heartbeat it got (``VehicleManager.companion_status``). Call it after any forward."""
        got = [d for d in deliveries(self.records).get(BACKEND, ())
               if d.msgid == HEARTBEAT and d.source[1] == COMPANION]
        rows: dict[int, list[tuple[int, str]]] = {i: [] for i in FLEET}
        for i, changes in rows.items():
            for k in range(1, LAST_TICK + 1):
                newest = max((d.k for d in got if d.source[0] == i and d.k <= k), default=None)
                age = None if newest is None else k - newest
                status = ("down" if age is None or age > COMPANION_DOWN_WINDOW_S
                          else "ok" if age <= COMPANION_OK_WINDOW_S else "checking")
                if not changes or changes[-1][1] != status:
                    changes.append((k, status))
        return rows


def window(start: int, heal: int) -> range:
    """The ticks a cut holds: a change at the end of tick k takes effect from k+1."""
    return range(start + 1, heal + 1)


def write_links(log_dir: Path, links: dict[int, list[tuple[int, str]]],
                companions: dict[int, list[tuple[int, str]]]) -> None:
    """The backend's timeline: ``{sys_id: [(k, outcome), ...]}`` of each event, with k in seconds."""
    path = log_dir / GCS_LOG_SUBDIR / "sessions" / "s" / "timeline.jsonl"
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = []
    for name, table in (("vehicle_link_state", links), ("vehicle_companion_state", companions)):
        for sys_id, events in table.items():
            for k, outcome in events:
                ts = datetime.fromtimestamp(SITL_START_TIME_UTC + k, tz=timezone.utc).isoformat()
                rows.append(json.dumps({"ts": ts, "event": name, "sys_id": sys_id, "outcome": outcome}))
    path.write_text("\n".join(rows) + "\n", encoding="utf-8")


__all__ = [
    "AUTOPILOT", "BACKEND", "COMPANION", "FENCE_K", "FLEET", "FaultJournal", "GCS", "HEARTBEAT", "LAST_TICK",
    "LAUNCH_K", "NAV_K", "PRESENCE_K", "S", "SWARM_HEARTBEAT", "window", "write_links",
]
