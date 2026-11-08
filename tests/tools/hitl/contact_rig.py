"""A contact service wired straight to navpy's HITL adapter and the real episode latch (step 26 tests).

No sockets and no threads: each side's send is the other side's receive, recorded on the way, so a test can replay,
alter or re-sign any datagram. The board clock is a number the test moves.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from types import SimpleNamespace
from typing import Any, Optional
from unittest.mock import Mock

import pytest

from navpy.core.approach.dock_contact_episode import GuidanceDockContact
from navpy.core.approach.vision_nav.episode_latch import GuidanceEpisodeLatch, GuidanceTerminal
from navpy.core.kernel.location import Location
from navpy.plugins.contact import hitl_contact_wire as wire
from navpy.plugins.contact.hitl_contact_link import ContactLinkConfig
from navpy.plugins.contact.hitl_contact_sensor import HitlDockContactSensor
from tools.hitl.contact_service import ContactService
from tools.tuning.hitl_bridge.unreal_pose import VEHICLE_STATE_SCHEMA

RUN = "run-26"
KEY = bytes(range(32))
OTHER_KEY = bytes(range(1, 33))
VEHICLE, TASK, DOCK_UID = 7, 10, 4
LAT, LON, HEIGHT = 40.0, 44.0, 1100.0
M_PER_DEG_LAT = 111_132.0
M_PER_DEG_LON = 111_320.0 * math.cos(math.radians(LAT))
DOCK = Location(LAT, LON, HEIGHT, is_absolute=True)
BRIDGE = ("127.0.0.1", 47100)
CONFIG = ContactLinkConfig(RUN, "pc-sitl", KEY, ("127.0.0.1", 47001), ("127.0.0.1", 47002))


def at(north_m: float, east_m: float = 0.0, up_m: float = 0.0) -> dict[str, float]:
    return {"lat_deg": LAT + north_m / M_PER_DEG_LAT, "lon_deg": LON + east_m / M_PER_DEG_LON,
            "alt_m": HEIGHT + up_m}


def sign(body: dict[str, Any], key: bytes = KEY) -> bytes:
    """Re-sign an altered body, bypassing encode's own checks (an attacker holding, or not holding, the key)."""
    import hmac, hashlib  # noqa: E401 - local: only this helper forges
    return wire.canonical({**body, wire.MAC_FIELD: hmac.new(key, wire.canonical(body), hashlib.sha256).hexdigest()})


@dataclass
class Rig:
    clock: list = field(default_factory=lambda: [100.0])
    to_service: list = field(default_factory=list)
    to_navpy: list = field(default_factory=list)
    records: list = field(default_factory=list)
    verdicts: list = field(default_factory=list)
    service_verdicts: list = field(default_factory=list)
    sequence: int = 0

    def __post_init__(self) -> None:
        nonces = iter(f"{n:032x}" for n in range(1, 1000))
        self.sensor = HitlDockContactSensor(RUN, KEY, self._to_service, lambda: self.clock[0],
                                            nonce=lambda: next(nonces))
        self.service = ContactService(CONFIG, DOCK, self._to_navpy, self.records.append)
        self.latch = GuidanceEpisodeLatch()
        self.episode = GuidanceDockContact(
            self.sensor, lambda: VEHICLE, self._begin, self.latch.latch_docked, Mock())
        self.task = TASK

    def _begin(self, identity, _source_s) -> bool:
        self.latch.begin(identity)
        return True

    def _to_service(self, datagram: bytes) -> None:
        self.to_service.append(datagram)
        self.service_verdicts.append(self.service.from_navpy(datagram))

    def _to_navpy(self, datagram: bytes) -> None:
        self.to_navpy.append(datagram)
        self.verdicts.append(self.sensor.receive(datagram))

    def nav_entry(self, dock_uid: int = DOCK_UID) -> None:
        poi = SimpleNamespace(identity=SimpleNamespace(obj_id=dock_uid), task=self.task)
        with pytest.MonkeyPatch.context() as patch:
            patch.setattr("navpy.core.approach.dock_contact_episode.get_poi_task_id", lambda p: p.task)
            self.episode.arm(poi)

    def pose(self, board_s: float, position: dict[str, float], sys_id: int = VEHICLE,
             session: str = "bridge-1", source: tuple[str, int] = BRIDGE) -> str:
        self.sequence += 1
        datagram = wire.canonical({"schema": VEHICLE_STATE_SCHEMA, "sys_id": sys_id, "session_id": session,
                                   "sequence": self.sequence, "board_time_us": round(board_s * 1e6),
                                   "position_lla": position,
                                   "attitude_deg": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}})
        return self.service.pose(datagram, source)

    def fly_past(self, east_m: float = 0.1, t0: Optional[float] = None, **pose: Any) -> None:
        """A straight pass, 2 m short to 2 m past the net at 25 m/s, one sample per 20 ms of board time."""
        t0 = self.clock[0] + 0.1 if t0 is None else t0
        for index in range(9):
            self.pose(t0 + index * 0.02, at(-2.0 + index * 0.5, east_m), **pose)

    @property
    def docked(self) -> bool:
        return self.latch.docked

    @property
    def terminal(self) -> Optional[GuidanceTerminal]:
        snapshot = self.latch.terminal_snapshot()
        return None if snapshot is None else snapshot.terminal

    def last_contact(self) -> dict[str, Any]:
        return wire.decode(self.to_navpy[-1], KEY, frozenset({wire.KIND_CONTACT}))


__all__ = ["BRIDGE", "CONFIG", "DOCK", "DOCK_UID", "KEY", "OTHER_KEY", "RUN", "Rig", "TASK", "VEHICLE", "at", "sign"]
