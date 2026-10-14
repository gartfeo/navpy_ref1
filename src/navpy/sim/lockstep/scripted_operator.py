"""A scripted GCS operator for one serial-step SITL, run under the baton.

It speaks MAVLink bytes on the SITL's serial1 as sysid 255: a 1 Hz GCS
heartbeat, then a fixed script: set a parameter by name and wait for its
echo, upload a mission with the real ``MissionUploader``, wait for a 3D GPS
fix, switch to AUTO and arm, each waiting for the autopilot's answer. It
joins at the boot barrier (the first frame with ``helpers_quiet``), so its
clock starts at that tick's ``source_us``. Every step is recorded with the
tick it finished on.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from types import SimpleNamespace
from typing import Any

from pymavlink.dialects.v20 import ardupilotmega as mavlink

from navpy.plugins.vehicle.ardupilot import mission_inbox, mission_upload
from navpy.plugins.vehicle.ardupilot.logger_ref import LoggerRef
from navpy.plugins.vehicle.ardupilot.mav_transport import MavTransport
from navpy.sim.lockstep.baton import BatonRuntime, LockstepFault
from navpy.sim.lockstep.baton_queue import BatonQueue
from navpy.sim.lockstep.baton_sync import BatonCondition, BatonLock, sleep
from navpy.sim.lockstep.lockstep_runtime import LockstepRuntime, module_port
from navpy.sim.lockstep.mavlink_wire import StreamDecoder, encoder
from navpy.sim.lockstep.step_wire import Frame

GCS_IDENTITY = (255, 190)
PLANE_MODE_AUTO = 10
MAV_CMD_DO_SET_MODE = 176
MAV_CMD_COMPONENT_ARM_DISARM = 400
#: Sim-time budgets for each scripted wait (seconds): generous product-scale
#: limits, so a miss is a real failure, not a race.
REPLY_TIMEOUT_S = 5.0
GPS_FIX_TIMEOUT_S = 120.0
#: Sim seconds after arming before the flight is checked (climbing in AUTO).
FLIGHT_CHECK_AFTER_S = 15.0
#: The flight check: half the 40 m takeoff altitude, and past the takeoff item.
MIN_CLIMB_M = 20.0
MIN_MISSION_SEQ = 2
MAV_RESULT_ACCEPTED = 0
MAV_MODE_FLAG_SAFETY_ARMED = 128


@dataclass(frozen=True)
class ScriptStep:
    name: str
    tick: int
    detail: str


def mission_items(home: str, sysid: int) -> list[mavlink.MAVLink_mission_item_int_message]:
    """Home, takeoff to 40 m, three waypoints, loiter."""
    lat, lon = (float(part) for part in home.split(",")[:2])
    rows = [
        (0, 16, 0.0, 0.0, 0.0),  # home placeholder (ArduPilot rewrites seq 0)
        (3, 22, 0.0, 0.0, 40.0),  # NAV_TAKEOFF
        (3, 16, 0.004, 0.0, 60.0),
        (3, 16, 0.004, 0.004, 60.0),
        (3, 16, 0.0, 0.004, 60.0),
        (3, 17, 0.0, 0.0, 60.0),  # NAV_LOITER_UNLIM
    ]
    items = []
    for seq, (frame, command, dlat, dlon, alt) in enumerate(rows):
        items.append(mavlink.MAVLink_mission_item_int_message(
            sysid, 0, seq, frame, command, 0, 1, 0.0, 0.0, 0.0, 0.0,
            int(round((lat + dlat) * 1e7)), int(round((lon + dlon) * 1e7)), alt, 0,
        ))
    return items


@dataclass
class _GcsSide:
    """The operator's MAVLink side, created at the boot barrier."""

    link: Any
    sink: Any
    send: BatonLock
    inbound: BatonQueue
    mailbox_cond: BatonCondition
    mailbox: list[Any]
    inbox: Any
    uploader: Any


class ScriptedOperator:
    def __init__(self, sysid: int, home: str) -> None:
        self._sysid = sysid
        self._home = home
        self._patched: list[tuple[Any, str, Any]] = []
        self._runtime: BatonRuntime | None = None
        self._gcs: _GcsSide | None = None
        self.tick = 0
        self.steps: list[ScriptStep] = []
        self.statustexts: list[ScriptStep] = []
        self.done = False

    # -- coordinator side -----------------------------------------------------
    def on_tick(self, frame: Frame) -> tuple[bytes, bytes]:
        self.tick = frame.tick
        if self._runtime is None:
            if not frame.helpers_quiet:
                return b"", b""  # boot barrier: only the SITL ticks
            self._start(frame.source_us)
        egress = frame.ports[1].egress
        self._runtime.tick(
            frame.source_us,
            (lambda: self._gcs.inbound.deliver(egress)) if egress else None,
        )
        return b"", self._gcs.sink.drain()

    def close(self) -> None:
        if self._runtime is not None:
            self._runtime.abort()
        while self._patched:  # the mission modules get their host time back
            module, name, original = self._patched.pop()
            setattr(module, name, original)

    def _patch(self, module: Any, name: str, value: Any) -> None:
        self._patched.append((module, name, getattr(module, name)))
        setattr(module, name, value)

    # -- baton side -------------------------------------------------------------
    def _start(self, now_us: int) -> None:
        runtime = BatonRuntime(start_us=now_us)
        self._runtime = runtime
        # The coordinator process stays on the wall runtime (its relay link is
        # host IO), so only the mission modules' ``rt`` is swapped for the run.
        # The operator is ordinal 0; its entropy is unused, so seed 0.
        port = module_port(LockstepRuntime.create(runtime, epoch_us=0, seed=0, ordinal=0))
        for module in (mission_inbox, mission_upload):
            self._patch(module, "rt", port)
        link, sink = encoder(*GCS_IDENTITY)
        send = BatonLock(runtime, reentrant=True)
        inbox = mission_inbox.MissionInbox()
        self._gcs = _GcsSide(
            link=link, sink=sink, send=send, inbound=BatonQueue(runtime),
            mailbox_cond=BatonCondition(runtime), mailbox=[], inbox=inbox,
            uploader=mission_upload.MissionUploader(
                self._sysid,
                MavTransport(SimpleNamespace(mav=link), send),
                store=None, inbox=inbox,
                logger_ref=LoggerRef(logging.getLogger("lockstep.operator")),
            ),
        )
        runtime.spawn(self._reader, name="operator-reader")
        runtime.spawn(self._heartbeat, name="operator-heartbeat")
        runtime.spawn(self._script, name="operator-script")
        runtime.run_until_quiescent()

    def _reader(self) -> None:
        decoder = StreamDecoder()
        while True:
            for message in decoder.feed(self._gcs.inbound.get()):
                if message.get_type() == "STATUSTEXT":
                    self.statustexts.append(ScriptStep("statustext", self.tick, message.text))
                self._gcs.inbox.publish(message, self._runtime.monotonic())
                with self._gcs.mailbox_cond:
                    self._gcs.mailbox.append(message)
                    self._gcs.mailbox_cond.notify_all()

    def _heartbeat(self) -> None:
        while True:
            with self._gcs.send:
                self._gcs.link.heartbeat_send(6, 8, 0, 0, 0)  # GCS, autopilot invalid
            sleep(self._runtime, 1.0)

    def _wait_for(self, predicate: Any, timeout_s: float, what: str) -> Any:
        found: list[Any] = []

        def check() -> bool:
            while self._gcs.mailbox:
                message = self._gcs.mailbox.pop(0)
                if predicate(message):
                    found.append(message)
                    return True
            return False

        with self._gcs.mailbox_cond:
            # Only messages that arrive during this wait count; a reply never
            # lands in the tick its request is sent (it reaches us at k+2).
            self._gcs.mailbox.clear()
            if not self._gcs.mailbox_cond.wait_for(check, timeout_s):
                raise LockstepFault(f"scripted operator: no {what} within {timeout_s} s")
        return found[0]

    def _record(self, name: str, detail: str) -> None:
        self.steps.append(ScriptStep(name, self.tick, detail))

    def _command(self, command: int, *params: float) -> str:
        """Send a COMMAND_LONG; report its result and the tick it was sent on."""
        padded = list(params) + [0.0] * (7 - len(params))
        sent_tick = self.tick
        with self._gcs.send:
            self._gcs.link.command_long_send(self._sysid, 1, command, 0, *padded)
        ack = self._wait_for(
            lambda m: m.get_type() == "COMMAND_ACK" and m.command == command,
            REPLY_TIMEOUT_S, f"COMMAND_ACK {command}",
        )
        if int(ack.result) != MAV_RESULT_ACCEPTED:
            raise LockstepFault(f"scripted operator: command {command} rejected ({ack.result})")
        return f"result={int(ack.result)} sent_tick={sent_tick}"

    def _script(self) -> None:
        sleep(self._runtime, 2.0)
        with self._gcs.send:
            self._gcs.link.param_set_send(self._sysid, 1, b"AAS_NAV_CGT", 20.0, 9)
        echo = self._wait_for(
            lambda m: m.get_type() == "PARAM_VALUE" and m.param_id == "AAS_NAV_CGT",
            REPLY_TIMEOUT_S, "PARAM_VALUE AAS_NAV_CGT",
        )
        self._record("param_set", f"AAS_NAV_CGT={echo.param_value}")
        ok = self._gcs.uploader.upload(timeout=REPLY_TIMEOUT_S, items=mission_items(self._home, self._sysid))
        if not ok:
            raise LockstepFault("scripted operator: mission upload failed")
        self._record("mission_upload", f"ok={ok}")
        self._wait_for(
            lambda m: m.get_type() == "GPS_RAW_INT" and m.fix_type >= 3,
            GPS_FIX_TIMEOUT_S, "3D GPS fix",
        )
        self._record("gps_fix", "3D")
        self._record("mode_auto", self._command(MAV_CMD_DO_SET_MODE, 1, PLANE_MODE_AUTO))
        self._record("arm", self._command(MAV_CMD_COMPONENT_ARM_DISARM, 1))
        self._wait_for(
            lambda m: m.get_type() == "HEARTBEAT" and m.get_srcSystem() == self._sysid
            and m.get_srcComponent() == 1 and m.base_mode & MAV_MODE_FLAG_SAFETY_ARMED
            and m.custom_mode == PLANE_MODE_AUTO,
            REPLY_TIMEOUT_S, "an armed AUTO heartbeat",
        )
        self._record("armed_auto", "heartbeat")
        sleep(self._runtime, FLIGHT_CHECK_AFTER_S)
        position = self._wait_for(
            lambda m: m.get_type() == "GLOBAL_POSITION_INT", REPLY_TIMEOUT_S, "GLOBAL_POSITION_INT",
        )
        current = self._wait_for(
            lambda m: m.get_type() == "MISSION_CURRENT", REPLY_TIMEOUT_S, "MISSION_CURRENT",
        )
        climb_m = position.relative_alt / 1000
        if climb_m < MIN_CLIMB_M or current.seq < MIN_MISSION_SEQ:
            raise LockstepFault(
                f"scripted operator: not flying the mission ({climb_m:.1f} m, item {current.seq})"
            )
        self._record("flying", f"relative_alt_m={climb_m:.1f} mission_seq={current.seq}")
        self.done = True


__all__ = ["ScriptStep", "ScriptedOperator", "mission_items"]
