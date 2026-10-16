"""The bridge's vehicle_state.v1 datagrams for the ZR10 emulator: decode, order, freshness, run validity.

The aircraft attitude comes only from here, never from navpy. Fail closed: the newest attitude is fresh for
VEHICLE_STATE_TIMEOUT_S; a gap, a second bridge session, a board-time step back (a reboot) or a datagram from
another system is counted and makes the run invalid.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass

from navpy.core.kernel.attitude import Attitude

VEHICLE_STATE_SCHEMA = "navpy.unreal.vehicle_state.v1"   # tools/tuning/hitl_bridge/unreal_pose.py
# Five of the bridge's 20 ms pose periods (unreal_pose.POSE_PERIOD_US): an older attitude no longer stabilises
# the rendered gimbal, so nothing is rendered from it.
VEHICLE_STATE_TIMEOUT_S = 0.1


@dataclass(frozen=True)
class VehicleState:
    sys_id: int
    session_id: str
    sequence: int
    board_time_us: int
    attitude: Attitude


def decode_vehicle_state(datagram: bytes) -> VehicleState:
    """One vehicle_state.v1 datagram -> VehicleState; ValueError for anything else."""
    try:
        payload = json.loads(datagram.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"not JSON: {exc}") from None
    if not isinstance(payload, dict) or payload.get("schema") != VEHICLE_STATE_SCHEMA:
        raise ValueError("not a vehicle_state.v1 datagram")
    integers = {}
    for name in ("sys_id", "sequence", "board_time_us"):
        value = payload.get(name)
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise ValueError(f"{name} must be a non-negative integer")
        integers[name] = value
    session_id = payload.get("session_id")
    if not isinstance(session_id, str) or not session_id:
        raise ValueError("session_id must be a non-empty string")
    attitude = payload.get("attitude_deg")
    if not isinstance(attitude, dict):
        raise ValueError("attitude_deg missing")
    angles = {}
    for name in ("roll", "pitch", "yaw"):
        value = attitude.get(name)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"attitude_deg.{name} must be a finite number")
        angles[name] = float(value)
    return VehicleState(integers["sys_id"], session_id, integers["sequence"], integers["board_time_us"],
                        Attitude(angles["pitch"], angles["yaw"], angles["roll"]))


@dataclass
class VehicleInputStats:
    received: int = 0
    accepted: int = 0
    malformed: int = 0
    stale_or_duplicate: int = 0
    other_system: int = 0
    sessions: int = 0
    board_time_regressions: int = 0
    gaps: int = 0
    longest_gap_s: float = 0.0


class VehicleInput:
    """Keeps the newest vehicle_state of one system; counts what makes a run invalid."""

    def __init__(self) -> None:
        self.stats = VehicleInputStats()
        self.latest: VehicleState | None = None
        self.received_s: float | None = None
        self._in_gap_since: float | None = None

    def accept(self, datagram: bytes, now_s: float) -> VehicleState | None:
        self.stats.received += 1
        try:
            state = decode_vehicle_state(datagram)
        except ValueError:
            self.stats.malformed += 1
            return None
        latest = self.latest
        if latest is not None and state.sys_id != latest.sys_id:
            self.stats.other_system += 1
            return None
        if latest is None or state.session_id != latest.session_id:
            self.stats.sessions += 1
        elif state.sequence <= latest.sequence:
            self.stats.stale_or_duplicate += 1
            return None
        elif state.board_time_us < latest.board_time_us:
            self.stats.board_time_regressions += 1
        self.stats.accepted += 1
        self.latest = state
        self.received_s = now_s
        if self._in_gap_since is not None:
            self.stats.longest_gap_s = max(self.stats.longest_gap_s, now_s - self._in_gap_since)
            self._in_gap_since = None
        return state

    def fresh(self, now_s: float) -> bool:
        """True while the newest attitude is younger than VEHICLE_STATE_TIMEOUT_S; counts each gap once."""
        if self.received_s is None:
            return False
        if now_s - self.received_s <= VEHICLE_STATE_TIMEOUT_S:
            return True
        if self._in_gap_since is None:
            self._in_gap_since = self.received_s
            self.stats.gaps += 1
        return False

    def invalid_reasons(self) -> list[str]:
        reasons = []
        if self.stats.accepted == 0:
            reasons.append("no vehicle_state received")
        if self.stats.gaps:
            reasons.append(f"{self.stats.gaps} vehicle_state gap(s) over {VEHICLE_STATE_TIMEOUT_S} s")
        if self.stats.sessions > 1:
            reasons.append(f"{self.stats.sessions} bridge sessions")
        if self.stats.board_time_regressions:
            reasons.append(f"{self.stats.board_time_regressions} board time step(s) back")
        if self.stats.other_system:
            reasons.append(f"{self.stats.other_system} vehicle_state(s) from another system")
        return reasons


__all__ = ["VEHICLE_STATE_SCHEMA", "VEHICLE_STATE_TIMEOUT_S", "VehicleInput", "VehicleInputStats", "VehicleState",
           "decode_vehicle_state"]
