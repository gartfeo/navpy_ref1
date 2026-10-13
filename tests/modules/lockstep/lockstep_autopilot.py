"""A reference autopilot byte stream for the bus contract test.

A reduced ArduPlane behind two serial ports, built with pymavlink encoders:
each port is its own MAVLink channel (decoder, sequence numbers), and the
vehicle state (mode, parameters, mission) is shared. It answers one tick
later, as the firmware does at best. Every second it sends the heartbeats
of ``heartbeats``: the gimbal's (compid 154) and the autopilot's (compid 1).
``param_burst`` unrelated ``PARAM_VALUE`` replies follow each named read's
answer in the same tick.
"""

from __future__ import annotations

from pymavlink.dialects.v20 import ardupilotmega as mavlink

from navpy.sim.lockstep.mavlink_wire import ByteSink, StreamDecoder, encoder

SYS_ID = 1
GIMBAL_COMPONENT = mavlink.MAV_COMP_ID_GIMBAL
PLANE_MANUAL = 0
HEARTBEAT_PERIOD_US = 1_000_000
PARAMETERS = {"AAS_NAV_CGT": 10.0, "SCHED_LOOP_RATE": 50.0}
#: Home, takeoff, one waypoint: (command, latitude e7, longitude e7, altitude m).
MISSION = (
    (mavlink.MAV_CMD_NAV_WAYPOINT, 401_000_000, 444_000_000, 0.0),
    (mavlink.MAV_CMD_NAV_TAKEOFF, 401_000_000, 444_000_000, 40.0),
    (mavlink.MAV_CMD_NAV_WAYPOINT, 401_040_000, 444_000_000, 60.0),
)


class _Port:
    def __init__(self) -> None:
        self.sink = ByteSink()
        self.decoder = StreamDecoder()
        self.autopilot, _ = encoder(SYS_ID, mavlink.MAV_COMP_ID_AUTOPILOT1, self.sink)
        self.gimbal, _ = encoder(SYS_ID, GIMBAL_COMPONENT, self.sink)


class ReferenceAutopilot:
    def __init__(
        self,
        ports: int = 2,
        *,
        heartbeats: frozenset[str] = frozenset({"gimbal", "autopilot"}),
        param_burst: int = 0,
    ) -> None:
        self._ports = [_Port() for _ in range(ports)]
        self._next_heartbeat_us = 0
        self._heartbeats = heartbeats
        self._param_burst = param_burst
        self.custom_mode = PLANE_MANUAL
        self.parameters = dict(PARAMETERS)
        #: (port, message type, source system, source component), in arrival order.
        self.received: list[tuple[int, str, int, int]] = []

    def step(self, now_us: int, inbound: list[bytes]) -> list[bytes]:
        """One firmware loop: parse each port's bytes, answer, maybe heartbeat."""
        for index, data in enumerate(inbound):
            port = self._ports[index]
            for message in port.decoder.feed(data):
                self.received.append((
                    index, message.get_type(), message.get_srcSystem(), message.get_srcComponent(),
                ))
                self._answer(port, message)
        if now_us >= self._next_heartbeat_us:
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

    def _answer(self, port: _Port, message: mavlink.MAVLink_message) -> None:
        kind = message.get_type()
        if getattr(message, "target_system", SYS_ID) != SYS_ID:
            return
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
            reply.mission_count_send(*source, len(MISSION), 0)
        elif kind == "MISSION_REQUEST_INT" and message.seq < len(MISSION):
            command, lat, lon, alt = MISSION[message.seq]
            reply.mission_item_int_send(
                *source, message.seq, mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, command,
                0, 1, 0.0, 0.0, 0.0, 0.0, lat, lon, alt, 0,
            )

    def _param_value(self, reply: mavlink.MAVLink, name: str) -> None:
        names = sorted(self.parameters)
        reply.param_value_send(
            name.encode(), self.parameters[name], mavlink.MAV_PARAM_TYPE_REAL32,
            len(names), names.index(name),
        )


__all__ = ["MISSION", "PARAMETERS", "ReferenceAutopilot", "SYS_ID"]
