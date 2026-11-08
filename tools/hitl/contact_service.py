"""The isolated contact service (hitl_final_approach.md section 9, stage 1 step 26).

Simulator truth ends here. The service reads the dock's pose (Unreal's applied board centre, for the scene the
scenario service delivered to it, step 14) and the plant's true pose (the bridge's vehicle_state datagrams, a
``--vehicle-state-copy``), and judges contact with the simulator's own contact sensor (``SimDockContactSensor``: a
flown segment strictly inside the 0.5 m net). navpy gets only authenticated, episode-bound contact datagrams
(``navpy/plugins/contact/hitl_contact_wire.py``): this arm touched the net at this board time. No coordinate, distance
or pose ever goes back; the service keeps its own evidence (arms, refusals, the closest pass) in a private JSONL file.

It judges only while navpy has an episode armed (``arm`` .. ``disarm``), only the armed vehicle's poses, and sends at
most one contact per arm. A navpy datagram must carry the run key's MAC, this run's id, the first navpy session the
service saw and a sequence number above the last one; anything else is refused and recorded.

    python -s -m tools.hitl.contact_service link --run-record REC --service IP:PORT --navpy IP:PORT --out FILE
    python -s -m tools.hitl.contact_service serve --link FILE --scenario DIR --poses IP:PORT --evidence FILE
"""
from __future__ import annotations

import argparse
import json
import secrets
from collections import Counter
from collections.abc import Callable, Mapping
from pathlib import Path
from typing import Any, Optional

from navpy.core.kernel.location import Location
from navpy.core.ports.dock_contact import DockContactEvent, DockContactIdentity
from navpy.plugins.contact import hitl_contact_wire as wire
from navpy.plugins.contact.hitl_contact_link import ContactLinkConfig, parse_endpoint
from navpy.sim.world.dock_contact_sensor import PositionSample, SimDockContactSensor
from navpy.sim.world.dock_net_radius import SIM_DOCK_NET_RADIUS_M
from navpy.sim.world.simulation_object import SimulationObject
from tools.hitl.contact_scene import read_dock
from tools.hitl.json_checks import RecordError
from tools.tuning.hitl_bridge.unreal_pose import POSE_PERIOD_US, VEHICLE_STATE_SCHEMA
from tools.tuning.private_data import private_dir

POSE_RATE_HZ = 1e6 / POSE_PERIOD_US     # the bridge's vehicle_state rate (one per board-time slot)
SERVICE_DOCK_UID = 1                    # the service's one dock; navpy's dock uid is only echoed back
_NAVPY_KINDS = frozenset({wire.KIND_ARM, wire.KIND_DISARM})

Sender = Callable[[bytes], None]
Recorder = Callable[[Mapping[str, Any]], None]


class ContactService:
    """One run's judge: navpy's arm/disarm datagrams in, plant poses in, at most one contact datagram per arm out."""

    def __init__(self, config: ContactLinkConfig, dock: Location, send: Sender, record: Recorder,
                 net_radius_m: float = SIM_DOCK_NET_RADIUS_M) -> None:
        self._config = config
        self._send = send
        self._record = record
        self._dock = SimulationObject(SERVICE_DOCK_UID, dock, 0)
        self._sensor = SimDockContactSensor(lambda: (self._dock,), POSE_RATE_HZ, net_radius_m,
                                            report=lambda closest: record({"event": "closest", "pass": repr(closest)}))
        self._session: Optional[str] = None
        self._seq = 0
        self._arm: Optional[dict[str, Any]] = None
        # The bridge: the first pose's sender address and session; no other sender or session is judged.
        self._bridge: Optional[tuple[tuple[str, int], str]] = None
        self._pose_sequence = -1
        self.refusals: Counter[str] = Counter()
        self.sent = 0

    def from_navpy(self, datagram: bytes) -> str:
        """One arm or disarm; returns "accepted" or the refusal reason."""
        try:
            body = wire.decode(datagram, self._config.key, _NAVPY_KINDS)
        except wire.WireRefused as refused:
            return self._refuse(refused.reason)
        if body["run_id"] != self._config.run_id:
            return self._refuse("wrong run")
        if self._session is None:
            self._session = body["session"]
        if body["session"] != self._session:
            return self._refuse("wrong session")
        if body["seq"] <= self._seq:
            return self._refuse("replayed")
        self._seq = body["seq"]
        if body["kind"] == wire.KIND_ARM:
            self._sensor.disarm()
            self._arm = body
            self._sensor.arm(DockContactIdentity(body["vehicle"], body["task_id"], SERVICE_DOCK_UID,
                                                 body["episode"]), self._on_contact)
        elif self._arm is not None and body["arm_id"] == self._arm["arm_id"]:
            self._sensor.disarm()
            self._arm = None
        else:
            return self._refuse("disarm of another arm")
        identity = {key: body[key] for key in ("vehicle", "task_id", "dock_uid", "episode") if key in body}
        self._record({"event": body["kind"], "seq": body["seq"], "arm_id": body["arm_id"], **identity})
        return "accepted"

    def pose(self, datagram: bytes, source: tuple[str, int]) -> str:
        """One vehicle_state datagram from the bridge; judged only while armed and only for the armed vehicle.

        The first well-formed pose binds the bridge (its sender address and session id). A pose from another
        address or session is refused, so a second sender cannot inject a crossing and two sessions' samples are
        never joined into one segment; a restarted bridge needs a new run.
        """
        try:
            state = json.loads(datagram.decode("utf-8"))
            if state["schema"] != VEHICLE_STATE_SCHEMA:
                raise ValueError("another schema")
            sys_id, session, sequence = int(state["sys_id"]), str(state["session_id"]), int(state["sequence"])
            time_us, lla = int(state["board_time_us"]), state["position_lla"]
            location = Location(float(lla["lat_deg"]), float(lla["lon_deg"]), float(lla["alt_m"]), is_absolute=True)
        except (UnicodeDecodeError, ValueError, KeyError, TypeError):
            return "malformed pose"
        if self._bridge is None:
            self._bridge = (tuple(source), session)
            self._record({"event": "bridge", "source": f"{source[0]}:{source[1]}"})
        if tuple(source) != self._bridge[0]:
            return self._refuse("foreign pose source")
        if session != self._bridge[1]:
            return self._refuse("other bridge session")
        if sequence <= self._pose_sequence:
            return "old pose"
        self._pose_sequence = sequence
        if self._arm is None:
            return "disarmed"
        if sys_id != self._arm["vehicle"]:
            return "other vehicle"
        # No velocity in vehicle_state: the bow bound takes the chord's own speed (ground speed 0 here).
        self._sensor.sample(PositionSample(time_us / 1e6, location, 0.0))
        return "judged"

    def _on_contact(self, event: DockContactEvent) -> None:
        arm = self._arm
        if arm is None:
            return
        body = {"schema": wire.SCHEMA, "kind": wire.KIND_CONTACT, "run_id": self._config.run_id,
                "session": arm["session"], "vehicle": arm["vehicle"], "task_id": arm["task_id"],
                "dock_uid": arm["dock_uid"], "episode": arm["episode"], "arm_id": arm["arm_id"],
                "board_time_us": round(event.aircraft_time_s * 1e6)}
        self._send(wire.encode(body, self._config.key))
        self.sent += 1
        self._record({"event": "contact", "arm_id": arm["arm_id"], "board_time_us": body["board_time_us"],
                      "closest_3d_m": event.closest_3d_m, "closest_h_m": event.closest_h_m,
                      "closest_v_m": event.closest_v_m})

    def _refuse(self, reason: str) -> str:
        self.refusals[reason] += 1
        self._record({"event": "refused", "reason": reason})
        return reason


def new_link(record: Mapping[str, Any], service: str, navpy: str, key: Optional[bytes] = None) -> ContactLinkConfig:
    """The run's link: its id and mode from the run record, a fresh random key, the two loopback endpoints."""
    run = record["run"]
    config = ContactLinkConfig(run["run_id"], run["mode"], wire.valid_key(key or secrets.token_bytes(wire.KEY_BYTES)),
                               parse_endpoint(service), parse_endpoint(navpy))
    if config.service == config.navpy:
        raise ValueError("the service and navpy endpoints must differ")
    return config


def write_link(config: ContactLinkConfig, path: Path, private_root: Optional[Path] = None) -> Path:
    """The key is private: written once, only inside the private folder."""
    path, root = Path(path).resolve(), Path(private_dir() if private_root is None else private_root).resolve()
    if root not in path.parents:
        raise RecordError([f"link file {path} is not in the private folder {root}; the run key is private data"])
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        with path.open("x", encoding="utf-8") as handle:
            handle.write(json.dumps(config.record(), indent=2, sort_keys=True) + "\n")
    except FileExistsError:
        raise RecordError([f"{path} exists: one link per run"]) from None
    return path


def _link(args: argparse.Namespace) -> int:
    from tools.hitl.approach_records import read_record
    print(write_link(new_link(read_record(Path(args.run_record)), args.service, args.navpy), Path(args.out)))
    return 0


def _serve(args: argparse.Namespace) -> int:
    from navpy.plugins.contact.hitl_contact_link import load_link
    from tools.hitl.contact_loop import serve
    config = load_link(args.link)
    dock = read_dock(Path(args.scenario))
    return serve(config, dock, parse_endpoint(args.poses), Path(args.evidence), ContactService)


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(prog="contact_service", description=__doc__.split("\n\n")[0])
    sub = parser.add_subparsers(required=True)
    link = sub.add_parser("link", help="write the run's contact link file (key and endpoints) in the private folder")
    link.add_argument("--run-record", required=True)
    link.add_argument("--service", required=True, help="the service's loopback IPv4:PORT")
    link.add_argument("--navpy", required=True, help="navpy's loopback IPv4:PORT")
    link.add_argument("--out", required=True)
    link.set_defaults(func=_link)
    serve = sub.add_parser("serve", help="judge contact for one run until interrupted")
    serve.add_argument("--link", required=True)
    serve.add_argument("--scenario", required=True, help="the run's scenario folder (contact must be a consumer)")
    serve.add_argument("--poses", required=True, help="the loopback IPv4:PORT the bridge's vehicle_state copy uses")
    serve.add_argument("--evidence", required=True, help="a new private JSONL file for the service's evidence")
    serve.set_defaults(func=_serve)
    args = parser.parse_args(argv)
    try:
        return args.func(args)
    except (RecordError, ValueError, OSError) as exc:
        print(f"error: {exc}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["ContactService", "POSE_RATE_HZ", "SERVICE_DOCK_UID", "main", "new_link", "write_link"]
