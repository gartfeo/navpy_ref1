"""The HITL-only ``DockContactSensor``: contact events from the PC contact service (hitl_final_approach.md step 26).

The service (``tools/hitl/contact_service.py``) holds the dock pose and the plant's true pose; navpy never does. This
adapter tells it which episode is armed (``arm`` with a fresh ``arm_id``; ``disarm`` at NAV exit) and accepts at most
one ``contact`` per arm, and only when it is

- authentic: the run key's MAC (``hitl_contact_wire.decode``);
- this run's and this navpy session's;
- for the armed ``arm_id`` and identity (vehicle, task, dock uid, episode); a disarmed adapter accepts nothing;
- not replayed: the first accepted contact of an arm ends that arm's acceptance;
- fresh: its board time is not before the arm (navpy's own ATTITUDE boot time when it armed), at most
  ``CONTACT_MAX_AGE_S`` behind that clock now and at most ``CONTACT_MAX_LEAD_S`` ahead of it; no board time on
  either side refuses.

An accepted contact goes to the existing path unchanged (``FinalApproachDockContact``: the armed identity, then the
episode latch, which ignores it after a NAV_FAIL, a miss or an earlier contact, then the terminal handoff's fences).
The event carries no geometry: the service sends none, so its distances are NaN.
"""
from __future__ import annotations

import logging
import math
from collections import Counter
from collections.abc import Callable
from dataclasses import dataclass
from typing import Optional

from navpy.core.kernel import runtime_time as rt
from navpy.core.ports.dock_contact import (
    DockContactCallback,
    DockContactEvent,
    DockContactIdentity,
    DockEpisodeEnd,
)
from navpy.plugins.contact import hitl_contact_wire as wire

_LOG = logging.getLogger(__name__)

# My value: the longest a contact may trail navpy's own board clock. The contact service runs beside SITL on the PC
# and its pose lags the board by one 20 ms pose slot; navpy's ATTITUDE lags the board too, so a fresh contact is
# usually ahead of that clock. One second, confirm's frame freshness, refuses a datagram held back or replayed.
CONTACT_MAX_AGE_S = 1.0
# My value: the most a contact may lead navpy's own board clock. The bridge's pose is ahead of navpy's ATTITUDE by
# the telemetry path only (tens of ms on the PC); one second also refuses a pose from before a board reboot, whose
# board time is far ahead of the rebooted clock.
CONTACT_MAX_LEAD_S = 1.0
ACCEPTED = "accepted"
NO_GEOMETRY_M = math.nan

BoardClock = Callable[[], Optional[float]]
Sender = Callable[[bytes], None]


def new_nonce() -> str:
    """128 bits as 32 hex digits (a version-4 UUID: 122 random bits)."""
    return rt.uuid4().hex


@dataclass
class _Arm:
    identity: DockContactIdentity
    arm_id: str
    on_docked: DockContactCallback
    board_s: Optional[float]
    delivered: bool = False


class HitlDockContactSensor:
    def __init__(self, run_id: str, key: bytes, send: Sender, board_time_s: BoardClock,
                 nonce: Callable[[], str] = new_nonce) -> None:
        self._run_id = run_id
        self._key = wire.valid_key(key)
        self._send = send
        self._board_time_s = board_time_s
        self._nonce = nonce
        self._session = nonce()
        self._seq = 0
        self._lock = rt.Lock()
        self._arm: Optional[_Arm] = None
        self.refusals: Counter[str] = Counter()

    def arm(self, identity: DockContactIdentity, on_docked: DockContactCallback) -> None:
        arm = _Arm(identity, self._nonce(), on_docked, self._board_time_s())
        with self._lock:
            self._arm = arm
            datagram = self._signed(wire.KIND_ARM, arm_id=arm.arm_id, vehicle=identity.vehicle,
                                    task_id=identity.task_id, dock_uid=identity.dock_uid, episode=identity.episode)
        self._deliver(datagram)

    def disarm(self, end: DockEpisodeEnd = DockEpisodeEnd()) -> None:
        """NAV exit: nothing is accepted after this; the service is told to stop judging the arm."""
        del end   # the service keeps its own record; navpy reports nothing back
        with self._lock:
            arm, self._arm = self._arm, None
            datagram = None if arm is None else self._signed(wire.KIND_DISARM, arm_id=arm.arm_id)
        if datagram is not None:
            self._deliver(datagram)

    def end_evidence(self, identity: DockContactIdentity) -> None:
        """No evidence is kept in navpy: the service records its own."""

    def receive(self, datagram: bytes) -> str:
        """One datagram from the service; returns ``ACCEPTED`` or the refusal reason (also counted)."""
        try:
            body = wire.decode(datagram, self._key, frozenset({wire.KIND_CONTACT}))
        except wire.WireRefused as refused:
            return self._refuse(refused.reason)
        now_s = self._board_time_s()
        with self._lock:
            arm = self._arm
            reason = _refusal(body, arm, self._run_id, self._session, now_s)
            if not reason and arm is not None:
                arm.delivered = True
        if reason or arm is None:
            return self._refuse(reason)
        event = DockContactEvent(identity=arm.identity, aircraft_time_s=body["board_time_us"] / 1e6,
                                 closest_3d_m=NO_GEOMETRY_M, closest_h_m=NO_GEOMETRY_M, closest_v_m=NO_GEOMETRY_M)
        arm.on_docked(event)
        return ACCEPTED

    def _signed(self, kind: str, **fields: object) -> bytes:
        self._seq += 1
        return wire.encode({"schema": wire.SCHEMA, "kind": kind, "run_id": self._run_id,
                            "session": self._session, "seq": self._seq, **fields}, self._key)

    def _deliver(self, datagram: bytes) -> None:
        """A lost arm only means no contact (no DOCKED); a send never raises into NAV entry or exit."""
        try:
            self._send(datagram)
        except Exception:  # noqa: BLE001 - the link is evidence-only for navigation
            _LOG.exception("HITL contact send failed")

    def _refuse(self, reason: str) -> str:
        with self._lock:
            self.refusals[reason] += 1
        _LOG.warning("HITL contact refused: %s", reason)
        return reason


def _refusal(body: dict, arm: Optional[_Arm], run_id: str, session: str, now_s: Optional[float]) -> str:
    if body["run_id"] != run_id:
        return "wrong run"
    if body["session"] != session:
        return "wrong session"
    if arm is None:
        return "disarmed"
    if body["arm_id"] != arm.arm_id:
        return "stale arm"
    identity = arm.identity
    if (body["vehicle"], body["task_id"], body["dock_uid"], body["episode"]) != (
            identity.vehicle, identity.task_id, identity.dock_uid, identity.episode):
        return "wrong identity"
    if arm.delivered:
        return "replayed"
    contact_s = body["board_time_us"] / 1e6
    if arm.board_s is None or now_s is None or not math.isfinite(arm.board_s) or not math.isfinite(now_s):
        return "no board time"
    if contact_s < arm.board_s:
        return "before arm"
    if now_s - contact_s > CONTACT_MAX_AGE_S:
        return "stale"
    if contact_s - now_s > CONTACT_MAX_LEAD_S:
        return "ahead of the board clock"
    return ""


__all__ = ["ACCEPTED", "CONTACT_MAX_AGE_S", "CONTACT_MAX_LEAD_S", "HitlDockContactSensor", "new_nonce"]
