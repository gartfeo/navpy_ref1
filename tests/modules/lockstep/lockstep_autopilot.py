"""A reference autopilot byte stream for the bus contract test.

A reduced ArduPlane behind two serial ports, built with pymavlink encoders:
each port is its own MAVLink channel (decoder, sequence numbers), and the
vehicle state (mode, parameters, mission) is shared. It answers one tick
later, as the firmware does at best. Every second it sends the heartbeats
of ``heartbeats``: the gimbal's (compid 154) and the autopilot's (compid 1).
``param_burst`` unrelated ``PARAM_VALUE`` replies follow each named read's
answer in the same tick. The mission is stored: ``MISSION_CLEAR_ALL``
empties it, an upload (count, then one request per item, then the ack)
replaces it, and a download serves it as uploaded. ``answer_limits``
withholds replies: a message kind is answered at most that many times.
``TERRAIN_CHECK`` is never answered, as by an autopilot with
``TERRAIN_ENABLE 0`` (the lockstep defaults keep terrain on since step 9h). :func:`navpy_parameters` adds what a NavPy companion
reads at boot: the firmware's ``SYSID_THISMAV``, its final-approach airframe
limits and the AAS table that the SITL's ``aas_params.lua`` adds. With ``forward``, it routes as ArduPilot
does between its MAVLink channels: a packet with no target, a broadcast
target, a foreign system's target or this system's without one of its own
components (a NAVLINK packet for the NavPy that shares the sysid) is also
written, as its original bytes, to every other port (before it is handled);
one for its autopilot or gimbal is not.
ArduPilot sends a targeted packet only where it learned that system; the
double sends it to every other port (a fleet's serial1 is the only other).
"""

from __future__ import annotations

import re
from pathlib import Path

from pymavlink.dialects.v20 import ardupilotmega as mavlink

from navpy.sim.lockstep.mavlink_wire import ByteSink, StreamDecoder, encoder

SYS_ID = 1
GIMBAL_COMPONENT = mavlink.MAV_COMP_ID_GIMBAL
PLANE_MANUAL = 0
HEARTBEAT_PERIOD_US = 1_000_000
PARAMETERS = {"AAS_NAV_CGT": 10.0, "SCHED_LOOP_RATE": 50.0}
#: The SITL's ``aas_params.lua`` (``scripts/lockstep_wsl.py`` deploys it).
AAS_LUA = Path(__file__).resolve().parents[3] / "scripts" / "lua" / "aas_params.lua"
_LUA_TABLE = re.compile(r'param:add_table\(PARAM_TABLE_KEY,\s*"([A-Z_]+)"')
_LUA_PARAM = re.compile(r'param:add_param\(PARAM_TABLE_KEY,\s*\d+,\s*"([A-Z_0-9]+)",\s*(-?[0-9.]+)\)')


def aas_lua_parameters() -> dict[str, float]:
    """The AAS table the SITL's Lua script adds, at its defaults."""
    source = AAS_LUA.read_text(encoding="utf-8")
    prefix = _LUA_TABLE.search(source).group(1)
    return {prefix + name: float(value) for name, value in _LUA_PARAM.findall(source)}


#: ArduPlane's defaults (``ArduPlane/config.h``, ``AP_PitchController.cpp``) for
#: what the final-approach law reads (``final_approach_airframe_config.py``).
#: A real autopilot answers them; a missing one costs each law reset a full
#: parameter-read timeout.
AIRFRAME_PARAMETERS = {"PTCH_LIM_MIN_DEG": -25.0, "PTCH_LIM_MAX_DEG": 20.0, "ROLL_LIMIT_DEG": 45.0,
                       "PTCH2SRV_TCONST": 0.5, "TRIM_THROTTLE": 45.0}


def navpy_parameters(sys_id: int = SYS_ID) -> dict[str, float]:
    """What a NavPy companion reads at boot: the firmware's identity and limits, and the AAS table."""
    return {"SYSID_THISMAV": float(sys_id), **AIRFRAME_PARAMETERS, **aas_lua_parameters()}
#: Home, takeoff, one waypoint: (command, latitude e7, longitude e7, altitude m).
MISSION = (
    (mavlink.MAV_CMD_NAV_WAYPOINT, 401_000_000, 444_000_000, 0.0),
    (mavlink.MAV_CMD_NAV_TAKEOFF, 401_000_000, 444_000_000, 40.0),
    (mavlink.MAV_CMD_NAV_WAYPOINT, 401_040_000, 444_000_000, 60.0),
)


class _Port:
    def __init__(self, sys_id: int) -> None:
        self.sink = ByteSink()
        self.decoder = StreamDecoder()
        self.autopilot, _ = encoder(sys_id, mavlink.MAV_COMP_ID_AUTOPILOT1, self.sink)
        self.gimbal, _ = encoder(sys_id, GIMBAL_COMPONENT, self.sink)


class ReferenceAutopilot:
    def __init__(
        self,
        ports: int = 2,
        *,
        heartbeats: frozenset[str] = frozenset({"gimbal", "autopilot"}),
        param_burst: int = 0,
        parameters: dict[str, float] | None = None,
        answer_limits: dict[str, int] | None = None,
        sys_id: int = SYS_ID,
        forward: bool = False,
    ) -> None:
        #: The vehicle's MAVLink system id (a fleet's vehicle i is sysid i).
        self.sys_id = sys_id
        self._forward = forward
        self._ports = [_Port(sys_id) for _ in range(ports)]
        self._next_heartbeat_us = 0
        self._heartbeats = heartbeats
        self._param_burst = param_burst
        self.custom_mode = PLANE_MANUAL
        self.parameters = {**PARAMETERS, **(parameters or {})}
        #: Each item's MISSION_ITEM_INT fields from ``frame`` to ``z``.
        self.mission: list[tuple] = [
            (mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, command, 0, 1, 0.0, 0.0, 0.0, 0.0, lat, lon, alt)
            for command, lat, lon, alt in MISSION
        ]
        self._upload: list[tuple] | None = None
        self._answers_left = dict(answer_limits or {})
        self._upload_count = 0
        #: (port, message type, source system, source component), in arrival order.
        self.received: list[tuple[int, str, int, int]] = []
        #: A lost link: while set, nothing is answered and no heartbeat is sent.
        self.silent = False

    def step(self, now_us: int, inbound: list[bytes]) -> list[bytes]:
        """One firmware loop: parse each port's bytes, answer, maybe heartbeat."""
        for index, data in enumerate(inbound):
            port = self._ports[index]
            for message in port.decoder.feed(data):
                self.received.append((
                    index, message.get_type(), message.get_srcSystem(), message.get_srcComponent(),
                ))
                if self._forward and self._routes(message):
                    for other in self._ports:
                        if other is not port:
                            other.sink.write(message.get_msgbuf())
                if not self.silent:
                    self._answer(port, message)
        if self.silent:
            self._next_heartbeat_us = max(self._next_heartbeat_us, now_us)
        elif now_us >= self._next_heartbeat_us:
            self._next_heartbeat_us += HEARTBEAT_PERIOD_US
            for port in self._ports:
                if "gimbal" in self._heartbeats:
                    port.gimbal.heartbeat_send(mavlink.MAV_TYPE_GIMBAL, mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 4)
                if "autopilot" in self._heartbeats:
                    port.autopilot.heartbeat_send(
                        mavlink.MAV_TYPE_FIXED_WING, mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                        mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, self.custom_mode, 3,
                    )
        return [port.sink.drain() for port in self._ports]

    def _routes(self, message: mavlink.MAVLink_message) -> bool:
        """Forwarded: unless it targets this system's autopilot or gimbal."""
        if getattr(message, "target_system", 0) != self.sys_id:
            return True
        return getattr(message, "target_component", None) not in (mavlink.MAV_COMP_ID_AUTOPILOT1, GIMBAL_COMPONENT)

    def _answer(self, port: _Port, message: mavlink.MAVLink_message) -> None:
        kind = message.get_type()
        if getattr(message, "target_system", self.sys_id) != self.sys_id:
            return
        if kind in self._answers_left:
            if self._answers_left[kind] <= 0:
                return  # withheld
            self._answers_left[kind] -= 1
        reply = port.autopilot
        source = (message.get_srcSystem(), message.get_srcComponent())
        if kind == "SET_MODE":
            self.custom_mode = message.custom_mode
            reply.command_ack_send(mavlink.MAV_CMD_DO_SET_MODE, mavlink.MAV_RESULT_ACCEPTED)
        elif kind == "PARAM_SET":
            self.parameters[message.param_id] = message.param_value
            self._param_value(reply, message.param_id)
        elif kind == "PARAM_REQUEST_READ" and message.param_id in self.parameters:
            self._param_value(reply, message.param_id)
            for index in range(self._param_burst):
                reply.param_value_send(
                    f"BURST_{index:04d}".encode(), float(index), mavlink.MAV_PARAM_TYPE_REAL32,
                    self._param_burst, index,
                )
        elif kind == "MISSION_REQUEST_LIST":
            reply.mission_count_send(*source, len(self.mission), 0)
        elif kind == "MISSION_REQUEST_INT" and message.seq < len(self.mission):
            reply.mission_item_int_send(*source, message.seq, *self.mission[message.seq], 0)
        elif kind == "MISSION_CLEAR_ALL":
            self.mission = []
            reply.mission_ack_send(*source, mavlink.MAV_MISSION_ACCEPTED, 0)
        elif kind == "MISSION_COUNT":
            self._upload, self._upload_count = [], message.count
            reply.mission_request_int_send(*source, 0, 0)
        elif kind == "MISSION_ITEM_INT" and self._upload is not None and message.seq == len(self._upload):
            # ArduPilot stores no frame for a command without a location (a jump)
            # and downloads it as MAV_FRAME_GLOBAL.
            frame = mavlink.MAV_FRAME_GLOBAL if message.command == mavlink.MAV_CMD_DO_JUMP else message.frame
            self._upload.append((
                frame, message.command, message.current, message.autocontinue,
                message.param1, message.param2, message.param3, message.param4,
                message.x, message.y, message.z,
            ))
            if len(self._upload) < self._upload_count:
                reply.mission_request_int_send(*source, len(self._upload), 0)
            else:
                self.mission, self._upload = self._upload, None
                reply.mission_ack_send(*source, mavlink.MAV_MISSION_ACCEPTED, 0)

    def _param_value(self, reply: mavlink.MAVLink, name: str) -> None:
        names = sorted(self.parameters)
        reply.param_value_send(
            name.encode(), self.parameters[name], mavlink.MAV_PARAM_TYPE_REAL32,
            len(names), names.index(name),
        )


__all__ = ["AIRFRAME_PARAMETERS", "MISSION", "PARAMETERS", "ReferenceAutopilot", "SYS_ID", "aas_lua_parameters", "navpy_parameters"]
