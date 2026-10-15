"""Passing certified lockstep evidence: a 3-UAV journal, per-UAV lines, pose files and plan.

Built as the coordinator writes them (``scripts/eval_gcs_demo_lockstep.py``):

- the operator's observations of three confirmations come from the real
  workflow with its evidence on (``approval_runs``), vehicle 1's request
  delivered first and vehicles 2 and 3 a pass or two later, so both become
  due while vehicle 1's POST blocks; each POST's response sends (the
  workflow's tallies) become the backend's emissions to its own vehicle on
  the router endpoint, each routed to that vehicle's SITL serial1;
- packet records (``coordinator_packets``) for the auction: the owner's
  advert to both peers, peer 2's offer, and per peer task the owner's
  request, the helper's RECEIVED ACK of it, its response after that ACK and the owner's APPLIED ACK,
  each crossing the five hops of a NAVLINK trace (``navlink_trace``);
- each NavPy's lines with ticks: its role's markers, ``CONFIRMING`` at its
  request's delivery tick, the rest after its approval's answer;
- an ``ok`` outcome whose terminal tick is the last first SNAP.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from functools import lru_cache
from pathlib import Path

from pymavlink.dialects.v20 import ardupilotmega as mavlink

from navpy.core.link.swarm_ack_msg import ACK_STATUS_APPLIED, ACK_STATUS_RECEIVED
from navpy.core.link.types import MsgType
from navpy.sim.lockstep.coordinator_types import ROUTER_DEVICE, serial0_device, serial1_device
from scripts.eval_certificate_source_time import SOURCE_TIME_SUBDIR
from scripts.eval_gcs_demo_constants import backend_log_path
from scripts.eval_gcs_demo_confirmation_timing import APPROVAL_TARGET, RESPONSE_DIRECTION, RESPONSE_ID
from scripts.eval_gcs_demo_fleet_gates import POSE_PRODUCERS, navpy_lines_file
from scripts.eval_gcs_demo_slice_gates import JOURNAL_FILE, POSE_CADENCE_STREAMS
from tests.modules.lockstep.approval_runs import request, run_workflow, uniform

GRID_US = 20_000
SYS_IDS = (1, 2, 3)
OWNER, PEERS = 1, (2, 3)
#: The backend's participant ordinal (``FleetLayout.backend``) and MAVLink system.
BACKEND, GCS = 4, 255
#: Each vehicle's request delivery tick (vehicles 2 and 3 become due during 1's POST).
DELIVERED = {1: 100, 2: 103, 3: 106}
#: The auction's packets start here (before any confirmation request).
AUCTION_TICK = 10
#: Peer -> the auction task it holds (the detector's namespace).
HELD = {2: 2, 3: 3}
REQUEST = mavlink.MAVLINK_MSG_ID_TASK_ASSIGN_REQUEST
RESPONSE = mavlink.MAVLINK_MSG_ID_TASK_ASSIGN_RESPONSE
ACK = mavlink.MAVLINK_MSG_ID_SWARM_ACK
#: The ``MsgType`` a ``SWARM_ACK``'s ref names (``task_messaging``).
REQUEST_TYPE, RESPONSE_TYPE = MsgType.TASK_ASSIGN_REQUEST.value, MsgType.TASK_ASSIGN_RESPONSE.value
ADVERT = mavlink.MAVLINK_MSG_ID_AVAILABLE_TASK_REQUEST
OFFER = mavlink.MAVLINK_MSG_ID_AVAILABLE_TASK_RESPONSE
COMPANION = mavlink.MAV_COMP_ID_ONBOARD_COMPUTER
LENGTH = 40

LINES = {
    "owner": ["GimbalNavigation(siyi_zr10): start_tracking obj_id=0 (prev=None, mode=LOCK)",
              "Self-detect orbit: poi=40.30, 44.45 orbit_r=500m", "POI: P1 (tracking obj_id=0)",
              "CONFIRMING: P1", "Sending confirm request for P1.", "Task 2 accepted by 2"],
    "peer": ["Task {task} assigned by owner 1", "Peer navigation started (orbit, offset=0m)",
             "GimbalNavigation(siyi_zr10): start_tracking obj_id=0 (prev=None, mode=LOCK)",
             "POI: P1 (tracking obj_id=0)", "Sending confirm request for P1.", "CONFIRMING: P1"],
    "after": ["POI 1 confirmed by ground station.", "INIT: NAV MODE", "RESET: PASSED POI",
              "SNAP(VISION-NAV-PN): 3d=1.5(h=1.3; v=0.8)"],
}


@dataclass
class Packets:
    """Packet events by tick, as the coordinator records them."""

    events: dict[int, list[list]] = field(default_factory=dict)
    seqs: dict[tuple, int] = field(default_factory=dict)
    msg_seq: int = 0
    #: The backend log's APPLIED lines: (helper, task, response UID, APPLIED UID).
    applied: list[tuple] = field(default_factory=list)

    def _id(self, origin, device: str) -> list:
        key = (origin, device)
        self.seqs[key] = self.seqs.get(key, 0) + 1
        return [origin, 1, device, self.seqs[key]]

    def _add(self, k: int, *events: list) -> None:
        self.events.setdefault(k, []).extend(events)

    def _emit(self, k, origin, device, sha, msgid, source, target, fields) -> list:
        packet_id = self._id(origin, device)
        event = ["emitted", packet_id, [0, LENGTH], sha, msgid, [source, COMPANION], [target, 0]]
        self._add(k, event + [fields] if fields else event)  # fields only where present
        return packet_id

    def send(self, k: int, msgid: int, sender: int, receivers: tuple[int, ...], target: int,
             **fields) -> list:
        """NavPy ``sender``'s packet crossing the five hops to each receiver; returns its UID."""
        self.msg_seq += 1
        uid = [sender, 7000 + sender, self.msg_seq]
        fields = {"uid": uid, **fields}
        sha = f"{msgid:x}{self.msg_seq:060x}"
        first = self._emit(k, sender, serial0_device(sender), sha, msgid, sender, target, fields)
        self._add(k + 1, ["admitted", first, LENGTH], ["delivered", first, serial0_device(sender)])
        up = self._emit(k + 2, f"sitl{sender}", serial1_device(sender), sha, msgid, sender, target, fields)
        copies = [self._id("router", serial1_device(r)) for r in receivers]
        self._add(k + 2, *(["copy", c, up] for c in copies), ["routed", up, len(copies)])
        for receiver, copy in zip(receivers, copies):
            self._add(k + 3, ["admitted", copy, LENGTH], ["delivered", copy, serial1_device(receiver)])
            down = self._emit(k + 4, f"sitl{receiver}", serial0_device(receiver), sha, msgid, sender,
                              target, fields)
            self._add(k + 5, ["delivered", down, [receiver, 1]])
        return uid

    def respond(self, k: int, sys_id: int) -> list:
        """The backend's ``TASK_CONFIRM_RESPONSE`` to ``sys_id``, routed to its SITL's serial1."""
        self.msg_seq += 1
        sha = f"{RESPONSE_ID:x}{self.msg_seq:060x}"
        emitted = self._emit(k, BACKEND, ROUTER_DEVICE, sha, RESPONSE_ID, GCS, sys_id, {})
        copy = self._id("router", serial1_device(sys_id))
        self._add(k, ["copy", copy, emitted], ["routed", emitted, 1], ["admitted", copy, LENGTH],
                  ["delivered", copy, serial1_device(sys_id)])
        return emitted


def auction_packets(held: dict[int, int] = HELD, k: int = AUCTION_TICK) -> Packets:
    packets = Packets()
    packets.send(k, ADVERT, OWNER, PEERS, -1, task_id=2)
    packets.send(k, OFFER, PEERS[0], (OWNER,), OWNER, task_id=2)
    for peer, task in held.items():
        k += 10
        req = packets.send(k, REQUEST, OWNER, (peer,), peer, task_id=task)
        packets.send(k + 6, ACK, peer, (OWNER,), OWNER,
                     ack={"ref": [req[1], req[2], REQUEST_TYPE], "status": ACK_STATUS_RECEIVED})
        resp = packets.send(k + 6, RESPONSE, peer, (OWNER,), OWNER, task_id=task)
        applied = packets.send(k + 12, ACK, OWNER, (peer,), peer,
                               ack={"ref": [resp[1], resp[2], RESPONSE_TYPE], "status": ACK_STATUS_APPLIED})
        packets.applied.append((peer, task, resp, applied))
    return packets


def _response_sends(records: list[dict]) -> list[list[int]]:
    """Each POST's response sends, (k4, k6], as ``[tick, sys_id]``; the workflow's
    tallies are dropped (it counts every send on one link)."""
    entries = [(r["k"], key, e) for r in records for key in ("sent", "answered")
               for e in r.get("operator", {}).get(key, ())]
    windows = []
    for k, key, e in entries:
        if key == "sent" and e[2] == "POST" and e[3] == APPROVAL_TARGET:
            answer = next(k6 for k6, key6, a in entries if key6 == "answered" and a[:2] == e[:2])
            windows.append((k, answer, e[4]))
    sends = []
    for record in records:
        navlink = record.pop("navlink", None)
        if navlink is None:
            continue
        ((count,),) = [list(d[RESPONSE_DIRECTION].values()) for d in navlink.values()]
        (sys_id,) = [s for k4, k6, s in windows if k4 < record["k"] <= k6]
        sends += [[record["k"], sys_id]] * count
    return sends


@lru_cache
def _workflow() -> tuple[str, str, str]:
    deliveries = {k: [request(sys_id)] for sys_id, k in DELIVERED.items()}
    records, approvals = run_workflow(uniform(GRID_US, 2_000), deliveries, set(SYS_IDS),
                                      max_gap_us=GRID_US)
    sends = _response_sends(records)
    return json.dumps(records), json.dumps(approvals), json.dumps(sends)


def workflow() -> tuple[list[dict], list[dict]]:
    """The confirmation records (grid first) and the approvals, fresh copies."""
    records, approvals, _sends = _workflow()
    return json.loads(records), json.loads(approvals)


def response_sends() -> list[list[int]]:
    """The backend's response sends, ``[tick, sys_id]``."""
    return json.loads(_workflow()[2])


def answer_ticks(records: list[dict]) -> dict[int, int]:
    """Vehicle -> its approval POST's answer tick (k6)."""
    entries = [(r["k"], key, e) for r in records for key in ("sent", "answered")
               for e in r.get("operator", {}).get(key, ())]
    return {e[4]: next(k for k, key6, a in entries if key6 == "answered" and a[:2] == e[:2])
            for _k, key, e in entries if key == "sent" and e[2] == "POST" and e[3] == APPROVAL_TARGET}


def vehicle_lines(records: list[dict]) -> dict[int, list[tuple[int, str]]]:
    """Each vehicle's lines: its role's markers up to ``CONFIRMING`` at its
    delivery tick, the rest one per tick after its answer; SNAPs 1 tick apart."""
    k6 = answer_ticks(records)
    lines = {}
    for sys_id in SYS_IDS:
        before = LINES["owner"] if sys_id == OWNER else [
            line.format(task=HELD[sys_id]) for line in LINES["peer"]]
        start = DELIVERED[sys_id] - len(before) + 1
        rows = [(start + i, line) for i, line in enumerate(before)]
        rows[-1] = (DELIVERED[sys_id], rows[-1][1])
        base = max(k6.values()) + 10 + sys_id
        rows += [(base + 10 * i, line) for i, line in enumerate(LINES["after"])]
        lines[sys_id] = sorted(rows, key=lambda row: row[0])
    return lines


def fleet_records(records: list[dict], lines: dict[int, list[tuple[int, str]]],
                  packets: Packets | None = None) -> list[dict]:
    """The workflow's records, every tick's members and packets (the backend's
    response sends added), up to parking, and the outcome."""
    packets = auction_packets() if packets is None else packets
    for k, sys_id in response_sends():
        packets.respond(k, sys_id)
    snaps = {sys_id: next(k for k, line in rows if "SNAP(VISION-NAV" in line)
             for sys_id, rows in lines.items()}
    terminal = max(snaps.values())
    grid, ticks = records[0], {r["k"]: r for r in records[1:]}
    out = [grid]
    for k in range(1, terminal + 3):
        record = ticks.get(k, {"k": k, "t_us": k * GRID_US})
        record["members"] = [{"ordinal": i, "generation": 1} for i in SYS_IDS]
        record["packets"] = packets.events.get(k, [])
        out.append(record)
    out.append({"outcome": "ok", "terminal_tick": terminal, "final_tick": terminal + 2,
                "parked_tick": terminal + 3,
                "first_snaps": [[sys_id, 1, snaps[sys_id]] for sys_id in SYS_IDS]})
    return out


def write_pose_files(log_dir: Path) -> None:
    directory = log_dir / SOURCE_TIME_SUBDIR
    directory.mkdir(exist_ok=True)
    for sys_id in SYS_IDS:
        for producer in POSE_PRODUCERS:
            for stream in POSE_CADENCE_STREAMS:
                (directory / f"pose_cadence_{stream}_uav_{sys_id}_{producer}.csv").write_text(
                    "wall_s,boot_ms\n1000.0,20\n1000.02,40\n", encoding="utf-8")


def write_plan(log_dir: Path) -> None:
    vehicles = [{"slot": "owner", "role": "owner", "sys_id": 1},
                {"slot": "peer-1", "role": "peer", "sys_id": 2},
                {"slot": "peer-2", "role": "peer", "sys_id": 3}]
    pois = [{"task_id": t, "nav_waypoint_ordinal": o, "lat": 40.1 - t / 10_000, "lon": 44.0}
            for t, o in ((1, 2), (2, 3), (3, 6))]
    (log_dir / "demo_mission_plan.json").write_text(json.dumps({
        "schema_version": 1, "manifest_schema_version": 2, "owner_slot": "owner",
        "vehicles": vehicles, "pois": pois}), encoding="utf-8")


def write_backend_log(log_dir: Path, packets: Packets) -> None:
    """The backend log's assignment lines for ``packets``' chains (``task_assign_listener``)."""
    rows = []
    for peer, task, resp, applied in packets.applied:
        rows += [f"INFO Task assign response: sender={peer} receiver={OWNER} task_id={task} "
                 f"accepted=True uid={resp[1]}:{resp[2]}",
                 f"INFO Task assign ack: owner={OWNER} helper={peer} task_id={task} status=APPLIED "
                 f"ref={resp[1]}:{resp[2]} uid={applied[1]}:{applied[2]}"]
    path = backend_log_path(log_dir)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("".join(row + "\n" for row in rows), encoding="utf-8")


def write_fleet_run(log_dir: Path, records: list[dict] | None = None,
                    lines: dict[int, list[tuple[int, str]]] | None = None) -> list[dict]:
    """Write passing evidence (or the given records and lines); return the approvals."""
    base, approvals = workflow()
    lines = vehicle_lines(base) if lines is None else lines
    records = fleet_records(base, lines) if records is None else records
    write_plan(log_dir)
    write_pose_files(log_dir)
    write_backend_log(log_dir, auction_packets())
    (log_dir / JOURNAL_FILE).write_text("".join(json.dumps(r) + "\n" for r in records), encoding="utf-8")
    for sys_id, rows in lines.items():
        (log_dir / navpy_lines_file(sys_id)).write_text(
            "".join(json.dumps([k, line]) + "\n" for k, line in rows), encoding="utf-8")
    return approvals


__all__ = [
    "ACK", "ADVERT", "AUCTION_TICK", "DELIVERED", "HELD", "OFFER", "OWNER", "PEERS", "REQUEST", "RESPONSE", "SYS_IDS",
    "Packets", "answer_ticks", "auction_packets", "fleet_records", "vehicle_lines", "workflow",
    "write_backend_log", "write_fleet_run", "write_plan", "write_pose_files",
]
