"""SWARM_ACK is registered on purpose and reaches the swarm task actor.

The task-assignment handshake acknowledges every message with SWARM_ACK
(docs/design/swarm-task-assignment-ack.md), so the companion's swarm path
imports swarm_ack_msg and ``@register_msg`` runs in a real companion process.

These tests pin both halves: the production import graph registers SWARM_ACK,
and an inbound packet runs the dispatch path into NAVLINK listeners and out
of NetworkMavlink as a SwarmAckMsg.
"""
import os
import subprocess
import sys
import unittest
from unittest.mock import MagicMock

from pymavlink.dialects.v20.ardupilotmega import MAV_TYPE_ONBOARD_CONTROLLER

from navpy.modules.comm.messages.msg_meta import MsgMeta
from navpy.modules.comm.messages.swarm_ack_msg import ACK_STATUS_APPLIED, SwarmAckMsg
from navpy.modules.comm.messages.types import MsgType
from navpy.modules.comm.network_mavlink import NetworkMavlink
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

# Runs in a FRESH interpreter: only the companion's production swarm path is
# imported, so the registry ends up exactly as a real companion has it.
_PRODUCTION_IMPORT_PROBE = """
import navpy.modules.nav.nav_network            # companion swarm session
from pymavlink.dialects.v20.ardupilotmega import MAVLINK_MSG_ID_SWARM_ACK
from navpy.modules.comm.messages.msg_abc import MsgRegistry
print("REGISTERED" if MsgRegistry.has_mav_id(MAVLINK_MSG_ID_SWARM_ACK) else "ABSENT")
"""


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
        sender_id=src_system, receiver_id=_VEHICLE_SYS_ID,
        ref_boot_id=1234, ref_msg_seq=7,
        ref_msg_type=MsgType.TASK_ASSIGN_RESPONSE.value,
        status=ACK_STATUS_APPLIED,
        meta=MsgMeta(boot_id=99, msg_seq=5, time_ms=0, ttl_ms=5000),
    )
    mav_msg = ack.to_mavlink()
    mav_msg._header.srcSystem = src_system
    mav_msg.get_srcComponent = MagicMock(return_value=COMPANION_COMPONENT_ID)
    return mav_msg


class ProductionRegistryStateTests(unittest.TestCase):
    def test_companion_swarm_imports_register_swarm_ack(self):
        """Guard against the swarm path losing the import that registers it
        -- an unregistered ack would be dropped before any listener."""
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
        self.assertEqual(result.stdout.strip().splitlines()[-1], "REGISTERED")


class SwarmAckDeliveryTests(unittest.TestCase):
    def test_inbound_swarm_ack_reaches_navlink_listeners(self):
        delivered = []
        router = _make_router(delivered.append)

        router.ingest(_swarm_ack_packet(src_system=_VEHICLE_SYS_ID))

        self.assertEqual(len(delivered), 1)
        self.assertEqual(delivered[0].get_type(), "SWARM_ACK")

    def test_network_hands_the_ack_to_listeners_as_a_message(self):
        network = NetworkMavlink(
            node_id=_VEHICLE_SYS_ID, vehicle=MagicMock(), logger=MagicMock(),
        )
        listener = MagicMock()
        network.set_listener(listener)

        network._on_mavlink(_swarm_ack_packet(src_system=2))

        ack = listener.on_message.call_args.args[0]
        self.assertIsInstance(ack, SwarmAckMsg)
        self.assertEqual(
            (ack.sender_id, ack.receiver_id, ack.ref_boot_id, ack.ref_msg_seq),
            (2, _VEHICLE_SYS_ID, 1234, 7),
        )
        self.assertEqual(ack.ref_msg_type, MsgType.TASK_ASSIGN_RESPONSE.value)
        self.assertEqual(ack.status, ACK_STATUS_APPLIED)


if __name__ == "__main__":
    unittest.main()
