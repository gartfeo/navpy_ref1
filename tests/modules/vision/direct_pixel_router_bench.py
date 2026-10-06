"""A vehicle over the REAL router, message store and callback registry.

Every message takes a live link's path: ruled on, handed to the admission
tap, stored, then dispatched, all on the caller's thread, which stands for
the bus's one reader thread. The direct-pixel tests that need the router
itself rather than a double of it build on this: the ATTITUDE ledger's
correlation (test_direct_pixel_ledger_correlation.py) and the neutrality of
tracing through the router (test_direct_pixel_router_neutrality.py).
"""

from __future__ import annotations

import math
import sys
from collections.abc import Callable
from types import SimpleNamespace
from typing import Any
from pymavlink.dialects.v20 import ardupilotmega as mavlink

from pymavlink.dialects.v20.ardupilotmega import (
    MAV_COMP_ID_AUTOPILOT1,
    MAV_COMP_ID_ONBOARD_COMPUTER,
    MAV_TYPE_ONBOARD_CONTROLLER,
)

from navpy.modules.common.models.location import Location
from navpy.modules.vehicle.inbound_router import InboundMessageRouter
from navpy.modules.vehicle.link_state import HeartbeatState, PacketLossTracker
from navpy.modules.vehicle.message_store import MessageStore
from navpy.modules.vehicle.message_subscriptions import CallbackRegistry
from navpy.modules.vehicle.mission_inbox import MissionInbox
from navpy.modules.vehicle.parameter_repository import ParameterRepository
from navpy.modules.vehicle.pose_telemetry import (
    CommandDiagnostics,
    PoseTelemetry,
)
from navpy.modules.vehicle.vehicle_identity import VehicleIdentity

TARGET_SYSTEM = 42
POI = Location(40.001, 44.002, 900.0, is_absolute=True)
PITCH, YAW, ROLL = (math.radians(value) for value in (-4.0, 100.0, 3.0))


class Message:
    """A MAVLink message as the router reads it: the header getters, and
    only the fields given, so a field nothing should read is absent."""

    def __init__(self, message_type: str, **fields: float) -> None:
        self._message_type = message_type
        self.__dict__.update(fields)

    def get_type(self) -> str:
        return self._message_type

    def get_msgId(self) -> int:  # noqa: N802 - pymavlink's name
        return 0

    def get_srcSystem(self) -> int:  # noqa: N802 - pymavlink's name
        return TARGET_SYSTEM

    def get_srcComponent(self) -> int:  # noqa: N802 - pymavlink's name
        return MAV_COMP_ID_AUTOPILOT1

    def get_seq(self) -> int:
        return 0


def attitude_message(boot_ms: int) -> Message:
    return Message(
        "ATTITUDE",
        time_boot_ms=boot_ms,
        roll=ROLL,
        pitch=PITCH,
        yaw=YAW,
        rollspeed=0.0,
        pitchspeed=0.0,
        yawspeed=0.0,
    )


def sim_state_message(time_us: int | None = None) -> Message:
    """Truth. Ruled on too, but without a stamp it is always admitted, and
    the ledger is subscribed to ATTITUDE alone. ``time_us`` is the navlink
    stamp on the autopilot clock; without it, as stock firmware sends it,
    truth has no source stamp."""
    stamp = {} if time_us is None else {"time_us": time_us}
    message = Message(
        "SIM_STATE",
        lat=40.0,
        lon=44.0,
        alt=1000.0,
        roll=ROLL,
        pitch=PITCH,
        yaw=YAW,
        **stamp,
    )
    packet = mavlink.MAVLink_sim_state_message(
        1.0, 0.0, 0.0, 0.0, ROLL, PITCH, YAW,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 40.0, 44.0, 1000.0,
        0.0, 0.0, 0.0, 0.0, 0.0, time_us=time_us or 0,
    )
    wire = bytes(packet.pack(mavlink.MAVLink(None, srcSystem=TARGET_SYSTEM, srcComponent=1)))
    message.get_msgbuf = lambda: wire
    return message


class BusLogger:
    """The bus logger the callback registry reports to. Keeps the exception
    being handled at each report, and can fail itself."""

    def __init__(self) -> None:
        self.handling: list[BaseException | None] = []
        self.failure: BaseException | None = None

    def error(self, text: str) -> None:
        self.handling.append(sys.exc_info()[1])
        if self.failure is not None:
            raise self.failure


class RouterVehicle:
    """What the source reads off a vehicle, over the REAL router, store and
    callback registry, so every message takes a live link's path: ruled on,
    handed to the tap, stored, then dispatched. ``failures`` makes the
    attitude read raise while the newest ATTITUDE carries that stamp."""

    def __init__(self) -> None:
        self.logger = BusLogger()
        logger_ref = SimpleNamespace(value=self.logger)
        self._store = MessageStore()
        self._callbacks = CallbackRegistry(logger_ref)
        self._router = InboundMessageRouter(
            VehicleIdentity(
                TARGET_SYSTEM,
                TARGET_SYSTEM,
                MAV_COMP_ID_ONBOARD_COMPUTER,
                MAV_TYPE_ONBOARD_CONTROLLER,
            ),
            self._store,
            self._callbacks,
            ParameterRepository(),
            HeartbeatState(TARGET_SYSTEM, 3.0),
            PacketLossTracker(),
            MissionInbox(),
            logger_ref,
        )
        self._telemetry = PoseTelemetry(self._store, CommandDiagnostics())
        self.air_speed = 40.0
        self.failures: dict[int, BaseException] = {}

    def get_param_or_default(self, name: str, default: float) -> float:
        return 50.0  # SCHED_LOOP_RATE

    def on_message(
        self, name: str, callback: Callable[[Any], None]
    ) -> object:
        return self._callbacks.subscribe(name, callback)

    def on_admission(
        self, name: str, callback: Callable[[Any], None]
    ) -> object:
        return self._router.on_admission(name, callback)

    def ingest(self, message: Message) -> None:
        self._router.ingest(message)

    @property
    def attitude_sample(self) -> object:
        latest = self._store.latest("ATTITUDE")
        if latest is not None:
            failure = self.failures.get(latest.message.time_boot_ms)
            if failure is not None:
                raise failure
        return self._telemetry.attitude_sample

    @property
    def simulator_truth_pose(self) -> object:
        return self._telemetry.simulator_truth_pose


__all__ = [
    "PITCH",
    "ROLL",
    "POI",
    "TARGET_SYSTEM",
    "YAW",
    "BusLogger",
    "Message",
    "RouterVehicle",
    "attitude_message",
    "sim_state_message",
]
