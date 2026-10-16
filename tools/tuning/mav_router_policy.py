"""The rules of the board link router (mav_router.py): where a board packet goes, and the stream-rate claims.

Pure logic, no I/O: the router's threads call these under its lock.

Routing. ArduPlane addresses its replies (COMMAND_ACK, PARAM_VALUE to a PARAM_REQUEST_READ, MISSION_*) to the
sender's system and component (GCS_Common.cpp:5188-5191, :5655-5658). A board packet with a target goes only to the
clients that have sent from that system (and that component, unless one of them is component 0, pymavlink's
default, which stands for its whole system); a packet without a target goes to every client. A target no client has
sent from is dropped and counted (never broadcast: another client could take a reply as its own).

Stream rates. Both clients share the board's channel 0, and ArduPilot keeps message intervals per channel.
REQUEST_DATA_STREAM ALL re-sets every message of every stream (GCS_Param.cpp:148-161 ->
initialise_message_intervals_for_stream, GCS_Common.cpp:6500-6514), so the runner's request would undo navpy's
SET_MESSAGE_INTERVAL claims (pose_streams.py). StreamClaims keeps each client's claims; the board gets, per claimed
message, the fastest positive claim, never slower than the fastest REQUEST_DATA_STREAM ALL rate a client asked for
(that request covers every message, so the floor keeps a claim from slowing another client's stream). A claim of 0
(default) or -1 (disable) withdraws that client's claim: with no positive claim left the board gets 0, back to its
stream rate, so no client can stop a message another one streams.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Optional

from pymavlink.dialects.v20 import ardupilotmega as mavlink2

ROUTER_SYSTEM = 250                     # the tools' system id (board.SOURCE_SYSTEM, SYSID_MYGCS in sitl/hitl.parm)
ROUTER_COMPONENT = mavlink2.MAV_COMP_ID_UDP_BRIDGE      # 240: the router's own commands, apart from the runner's
ALL_COMPONENTS = 0                      # MAV_COMP_ID_ALL: a target or a client identity standing for the system
NO_TARGET = 0
INTERVAL_DEFAULT_US = 0                 # MAV_CMD_SET_MESSAGE_INTERVAL param2: back to the stream's rate
US_PER_S = 1_000_000
SET_MESSAGE_INTERVAL = mavlink2.MAV_CMD_SET_MESSAGE_INTERVAL
COMMAND_MESSAGES = ("COMMAND_LONG", "COMMAND_INT")


@dataclass(frozen=True)
class Identity:
    system: int
    component: int


def target_of(msg: mavlink2.MAVLink_message) -> Optional[Identity]:
    """The packet's target, or None when it has none (target_system absent or 0: broadcast)."""
    system = getattr(msg, "target_system", NO_TARGET) or NO_TARGET
    if system == NO_TARGET:
        return None
    return Identity(system, getattr(msg, "target_component", ALL_COMPONENTS) or ALL_COMPONENTS)


def addressed_to(target: Identity, identities: Iterable[Identity]) -> bool:
    """True when a client that sent from these identities is the target."""
    return any(i.system == target.system and ALL_COMPONENTS in (target.component, i.component)
               or i == target for i in identities)


def destinations(msg: mavlink2.MAVLink_message, clients: dict[str, set[Identity]]) -> list[str]:
    """The clients a board packet goes to (empty: no client has sent from its target)."""
    target = target_of(msg)
    if target is None:
        return list(clients)
    return [name for name, identities in clients.items() if addressed_to(target, identities)]


def is_router_target(msg: mavlink2.MAVLink_message) -> bool:
    """A board reply to the router's own commands (the re-sent intervals)."""
    return target_of(msg) == Identity(ROUTER_SYSTEM, ROUTER_COMPONENT)


def interval_request(msg: mavlink2.MAVLink_message) -> Optional[tuple[int, int]]:
    """(message id, interval in us) of a plain SET_MESSAGE_INTERVAL command, else None. A request with param3
    set (a response target, which ArduPilot denies, GCS_Common.cpp:3122), or with a value that is no number (NaN,
    infinity), is not a claim and passes unchanged."""
    if msg.get_type() not in COMMAND_MESSAGES or msg.command != SET_MESSAGE_INTERVAL or msg.param3 != 0:
        return None
    if not (math.isfinite(msg.param1) and math.isfinite(msg.param2)):
        return None
    return int(msg.param1), int(msg.param2)


def all_streams_rate_hz(msg: mavlink2.MAVLink_message) -> Optional[int]:
    """The rate of a REQUEST_DATA_STREAM that starts every stream (MAV_DATA_STREAM_ALL), else None."""
    if (msg.get_type() != "REQUEST_DATA_STREAM" or msg.req_stream_id != mavlink2.MAV_DATA_STREAM_ALL
            or msg.start_stop != 1 or msg.req_message_rate <= 0):
        return None
    return int(msg.req_message_rate)


class StreamClaims:
    """Every client's SET_MESSAGE_INTERVAL claims and the interval the board gets for each claimed message."""

    def __init__(self) -> None:
        self._claims: dict[str, dict[int, int]] = {}
        self._floor_us: Optional[int] = None        # from the fastest REQUEST_DATA_STREAM ALL so far

    def claim(self, client: str, message_id: int, interval_us: int) -> int:
        """Record a client's request; returns the interval the board must get for that message."""
        claims = self._claims.setdefault(client, {})
        if interval_us > 0:
            claims[message_id] = interval_us
        else:
            claims.pop(message_id, None)
        return self.effective(message_id)

    def stream_rate(self, rate_hz: int) -> None:
        """A client asked for every stream at this rate: no claimed message may go slower than the fastest such."""
        floor_us = US_PER_S // rate_hz
        self._floor_us = floor_us if self._floor_us is None else min(self._floor_us, floor_us)

    def effective(self, message_id: int) -> int:
        positive = [c[message_id] for c in self._claims.values() if message_id in c]
        if not positive:
            return INTERVAL_DEFAULT_US
        fastest = min(positive)
        return fastest if self._floor_us is None else min(fastest, self._floor_us)

    def claimed(self) -> dict[int, int]:
        """Every message some client has claimed, with the interval the board must get."""
        ids = {m for c in self._claims.values() for m in c}
        return {m: self.effective(m) for m in sorted(ids)}
