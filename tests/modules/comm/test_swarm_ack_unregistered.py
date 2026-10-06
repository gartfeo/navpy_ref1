"""SWARM_ACK is safe to receive even though production never registers it.

Nothing in production imports swarm_ack_msg any more (the auto-ack hook and
the resend stop-signal that used it were removed as dead code), so
``@register_msg`` never runs for SWARM_ACK in a real GCS/companion process.
The message therefore has no class in MsgRegistry, and the focused inbound
router drops it before any NAVLINK listener (NetworkMavlink,
TaskConfirmListener) is consulted.

These tests pin both halves of that: the production import graph really does
leave SWARM_ACK unregistered, and a packet arriving in that state traverses
the dispatch path without raising and without being delivered.

NOTE: importing SwarmAckMsg below registers it in THIS process (that is how
the class gets built at all), so the dispatch test removes the registration
for its duration to reproduce the production registry.
"""
import os
import subprocess
import sys
import unittest
from contextlib import contextmanager
from unittest.mock import MagicMock

from pymavlink.dialects.v20.ardupilotmega import MAV_TYPE_ONBOARD_CONTROLLER

from navpy.modules.comm.messages.available_task_msg import (
    TaskConfirmRequestMsg, TaskMsgData,
)
from navpy.modules.comm.messages.location_msg import LocationMsgData
from navpy.modules.comm.messages.msg_abc import MsgRegistry
from navpy.modules.comm.messages.swarm_ack_msg import ACK_STATUS_RECEIVED, SwarmAckMsg
from navpy.modules.comm.messages.types import MsgType, TaskTypeMsgData
from navpy.modules.vehicle.inbound_router import InboundMessageRouter
from navpy.modules.vehicle.link_state import HeartbeatState, PacketLossTracker
from navpy.modules.vehicle.message_store import MessageStore
from navpy.modules.vehicle.message_subscriptions import CallbackRegistry
from navpy.modules.vehicle.mission_inbox import MissionInbox
from navpy.modules.vehicle.parameter_repository import ParameterRepository
from navpy.modules.vehicle.vehicle_identity import VehicleIdentity
from gcs.backend.companion_identity import COMPANION_COMPONENT_ID

# The companion shares its aircraft's system id (component 191 tells it apart
# from the autopilot); a foreign system id would be filtered before dispatch.
_VEHICLE_SYS_ID = 1

# Runs in a FRESH interpreter: only production modules are imported, so the
# registry ends up exactly as a real GCS/companion process has it.
_PRODUCTION_IMPORT_PROBE = """
import gcs.backend.task_confirm_listener        # GCS confirm path
import navpy.modules.comm.network_mavlink       # companion network layer
import navpy.modules.nav.confirmation_manager         # companion confirm path
from pymavlink.dialects.v20.ardupilotmega import MAVLINK_MSG_ID_SWARM_ACK
from navpy.modules.comm.messages.msg_abc import MsgRegistry
print("REGISTERED" if MsgRegistry.has_mav_id(MAVLINK_MSG_ID_SWARM_ACK) else "ABSENT")
"""


@contextmanager
def _swarm_ack_absent_from_registry():
    """Reproduce the production registry: no SWARM_ACK entry at all."""
    with MsgRegistry._registry_lock:
        by_mav_id = MsgRegistry._mav_registry.pop(SwarmAckMsg.mav_id(), None)
        by_type = MsgRegistry._registry.pop(MsgType.SWARM_ACK, None)
    try:
        yield
    finally:
        with MsgRegistry._registry_lock:
            if by_mav_id is not None:
                MsgRegistry._mav_registry[SwarmAckMsg.mav_id()] = by_mav_id
            if by_type is not None:
                MsgRegistry._registry[MsgType.SWARM_ACK] = by_type


def _make_router(navlink_callback) -> InboundMessageRouter:
    """Build the focused inbound dispatch owner with a NAVLINK subscription."""
    callbacks = CallbackRegistry()
    callbacks.subscribe("NAVLINK", navlink_callback)
    return InboundMessageRouter(
        VehicleIdentity(
            _VEHICLE_SYS_ID,
            _VEHICLE_SYS_ID,
            COMPANION_COMPONENT_ID,
            MAV_TYPE_ONBOARD_CONTROLLER,
        ),
        MessageStore(),
        callbacks,
        ParameterRepository(),
        HeartbeatState(_VEHICLE_SYS_ID, 3.0),
        PacketLossTracker(),
        MissionInbox(),
        type("LoggerRef", (), {"value": MagicMock()})(),
    )


def _swarm_ack_packet(src_system: int):
    ack = SwarmAckMsg(
        sender_id=0, receiver_id=_VEHICLE_SYS_ID,
        ref_boot_id=1234, ref_msg_seq=7, ref_msg_type=MsgType.TASK_CONFIRM_REQUEST.value,
        status=ACK_STATUS_RECEIVED,
    )
    mav_msg = ack.to_mavlink()
    mav_msg.get_srcSystem = MagicMock(return_value=src_system)
    mav_msg.get_srcComponent = MagicMock(return_value=COMPANION_COMPONENT_ID)
    return mav_msg


def _confirm_request_packet(src_system: int):
    req = TaskConfirmRequestMsg(
        sender_id=_VEHICLE_SYS_ID,
        task=TaskMsgData(
            task_id=7, task_type=TaskTypeMsgData.UNKNOWN,
            location=LocationMsgData(lat=32.5, lng=34.8, alt=100.0),
        ),
    )
    mav_msg = req.to_mavlink()
    mav_msg.get_srcSystem = MagicMock(return_value=src_system)
    mav_msg.get_srcComponent = MagicMock(return_value=COMPANION_COMPONENT_ID)
    return mav_msg


class ProductionRegistryStateTests(unittest.TestCase):
    def test_production_imports_do_not_register_swarm_ack(self):
        """Guard against a future production import silently re-registering
        it -- that would change how an inbound SWARM_ACK is handled."""
        # Hand the child THIS process's import path so it loads the same
        # checkout (a bare interpreter could pick up a different one via a
        # site-packages .pth).
        env = dict(os.environ)
        env["PYTHONPATH"] = os.pathsep.join(p for p in sys.path if p)
        result = subprocess.run(
            [sys.executable, "-c", _PRODUCTION_IMPORT_PROBE],
            capture_output=True, text=True, timeout=300, env=env,
        )
        self.assertEqual(result.returncode, 0, f"probe failed: {result.stderr}")
        self.assertEqual(result.stdout.strip().splitlines()[-1], "ABSENT")


class SwarmAckDispatchDropTests(unittest.TestCase):
    """An inbound SWARM_ACK in production registry shape is dropped quietly."""

    def setUp(self):
        self.delivered = []
        self.router = _make_router(self.delivered.append)

    def test_swarm_ack_from_gcs_is_dropped_at_the_registry_gate(self):
        with _swarm_ack_absent_from_registry():
            self.router.ingest(_swarm_ack_packet(src_system=0))

        self.assertEqual(self.delivered, [], "SWARM_ACK must not reach NAVLINK listeners")

    def test_swarm_ack_from_the_companion_traverses_dispatch_undelivered(self):
        """Same packet from an id the vehicle does accept: it runs the whole
        inbound route (past the source filter and cache publication) and still
        reaches no NAVLINK listener, and does not raise."""
        with _swarm_ack_absent_from_registry():
            self.router.ingest(_swarm_ack_packet(src_system=_VEHICLE_SYS_ID))

        self.assertEqual(self.delivered, [])

    def test_registered_navlink_message_is_still_delivered(self):
        """Control: the same harness DOES deliver a message production still
        registers, so the two assertions above are not vacuous."""
        with _swarm_ack_absent_from_registry():
            self.router.ingest(_confirm_request_packet(src_system=_VEHICLE_SYS_ID))

        self.assertEqual(len(self.delivered), 1)
        self.assertEqual(self.delivered[0].get_type(), "TASK_CONFIRM_REQUEST")

    def test_registry_entry_is_restored_after_the_context(self):
        """The temporary removal must not leak into other tests."""
        with _swarm_ack_absent_from_registry():
            self.assertFalse(MsgRegistry.has_mav_id(SwarmAckMsg.mav_id()))
        self.assertTrue(MsgRegistry.has_mav_id(SwarmAckMsg.mav_id()))
        self.assertIs(MsgRegistry.get_class(MsgType.SWARM_ACK), SwarmAckMsg)


if __name__ == "__main__":
    unittest.main()
