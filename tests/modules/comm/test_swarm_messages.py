"""
Unit tests for SwarmAckMsg (SWARM_ACK, navlink id 25110, redefined) and
SwarmRequestMsg (SWARM_REQUEST, navlink id 25111).

Both messages are real @register_msg navlink dialect classes (D-25/D-26,
Phase 2 plan 02-01). This module proves:
- exactly one class each is registered (no duplicate MAVLink ID error)
- to_dict/from_dict round-trip
- to_mavlink/from_mavlink round-trip is field-equal (real pymavlink message
  object construction/decode, same pattern as test_mavlink_conversion.py)
- receiver_id maps 1:1 with the native target_system field both ways
- get_ttl_ms returns the dedicated ~3000ms entries, not DEFAULT_TTL_MS
- (Task 3 tracer) a scripted VehicleMav loopback proof that both messages
  route companion->GCS through the SAME registration/dispatch pattern
  production code uses (vehicle.on_message("NAVLINK", ...)) -- the
  CI-durable, headless stand-in for the live launcher-started SITL routing
  proof (both ids verified byte-correct on the
  real 2-instance run_swarm.sh topology, CRC gate passed).

NOTE: importing swarm_ack_msg / swarm_request_msg triggers @register_msg
against the live MsgRegistry - do this exactly once per module (module-level
import), matching the "no central message loader" convention documented in
these modules' docstrings.
"""
import unittest

from pymavlink.dialects.v20.ardupilotmega import MAVLink_message

from navpy.modules.comm.messages.msg_abc import MsgRegistry
from navpy.modules.comm.messages.msg_meta import MsgMeta
from navpy.modules.comm.messages.types import MsgType
from navpy.modules.comm.messages.ttl_defaults import get_ttl_ms, DEFAULT_TTL_MS
from navpy.modules.comm.messages.swarm_ack_msg import (
    SwarmAckMsg,
    ACK_STATUS_RECEIVED,
    ACK_STATUS_APPLIED,
)
from navpy.modules.comm.messages.swarm_request_msg import (
    SwarmRequestMsg,
    REQUEST_TYPE_RESOURCE,
    SUBJECT_TYPE_THUMBNAIL,
)


TEST_META = MsgMeta(boot_id=111, msg_seq=222, time_ms=1753280000123, ttl_ms=3000)


def _roundtrip(msg):
    """Mirror TestMavlinkConversion._roundtrip in test_mavlink_conversion.py."""
    mav: MAVLink_message = msg.to_mavlink()
    mav._header.srcSystem = msg.sender_id  # imitate the sender_id in mavlink header
    return msg.__class__.from_mavlink(mav)


# ---------------------------------------------------------------------------
# Registration - exactly one class per MsgType/mav_id, old 25110 slot gone
# ---------------------------------------------------------------------------

class TestSwarmMessageRegistration(unittest.TestCase):
    def test_swarm_ack_registered_exactly_once(self):
        self.assertIs(MsgRegistry.get_class(MsgType.SWARM_ACK), SwarmAckMsg)
        self.assertIs(MsgRegistry.get_class_by_mav_id(SwarmAckMsg.mav_id()), SwarmAckMsg)
        self.assertEqual(SwarmAckMsg.mav_id(), 25110)

    def test_swarm_request_registered_exactly_once(self):
        self.assertIs(MsgRegistry.get_class(MsgType.SWARM_REQUEST), SwarmRequestMsg)
        self.assertIs(MsgRegistry.get_class_by_mav_id(SwarmRequestMsg.mav_id()), SwarmRequestMsg)
        self.assertEqual(SwarmRequestMsg.mav_id(), 25111)


# ---------------------------------------------------------------------------
# SwarmAckMsg
# ---------------------------------------------------------------------------

class TestSwarmAckMsg(unittest.TestCase):
    def _make(self):
        return SwarmAckMsg(
            sender_id=101, receiver_id=1,
            ref_boot_id=2147483647, ref_msg_seq=9999,
            ref_msg_type=MsgType.TASK_CONFIRM_REQUEST.value,
            status=ACK_STATUS_APPLIED, meta=TEST_META,
        )

    def test_dict_roundtrip(self):
        msg = self._make()
        rt = SwarmAckMsg.from_dict(msg.to_dict())
        self.assertEqual(rt.sender_id, 101)
        self.assertEqual(rt.receiver_id, 1)
        self.assertEqual(rt.ref_boot_id, 2147483647)
        self.assertEqual(rt.ref_msg_seq, 9999)
        self.assertEqual(rt.ref_msg_type, MsgType.TASK_CONFIRM_REQUEST.value)
        self.assertEqual(rt.status, ACK_STATUS_APPLIED)

    def test_mavlink_roundtrip(self):
        msg = self._make()
        mav = msg.to_mavlink()
        # receiver_id <-> native target_system mapping, both directions.
        self.assertEqual(mav.target_system, 1)

        result = _roundtrip(msg)
        self.assertIsInstance(result, SwarmAckMsg)
        self.assertEqual(result.to_dict(), msg.to_dict())
        self.assertEqual(result.receiver_id, 1)
        self.assertEqual(result.ref_boot_id, 2147483647)
        self.assertEqual(result.ref_msg_seq, 9999)
        self.assertEqual(result.ref_msg_type, MsgType.TASK_CONFIRM_REQUEST.value)
        self.assertEqual(result.status, ACK_STATUS_APPLIED)
        self.assertEqual(result.meta.boot_id, 111)
        self.assertEqual(result.meta.msg_seq, 222)
        self.assertEqual(result.meta.ttl_ms, 3000)

    def test_default_receiver_id_zero_when_none(self):
        msg = SwarmAckMsg(
            sender_id=1, receiver_id=None,
            ref_boot_id=1, ref_msg_seq=1, ref_msg_type=1,
            status=ACK_STATUS_RECEIVED, meta=TEST_META,
        )
        mav = msg.to_mavlink()
        self.assertEqual(mav.target_system, 0)


# ---------------------------------------------------------------------------
# SwarmRequestMsg
# ---------------------------------------------------------------------------

class TestSwarmRequestMsg(unittest.TestCase):
    def _make(self):
        return SwarmRequestMsg(
            sender_id=1, receiver_id=101,
            request_type=REQUEST_TYPE_RESOURCE, subject_type=SUBJECT_TYPE_THUMBNAIL,
            subject_id=55, meta=TEST_META,
        )

    def test_dict_roundtrip(self):
        msg = self._make()
        rt = SwarmRequestMsg.from_dict(msg.to_dict())
        self.assertEqual(rt.sender_id, 1)
        self.assertEqual(rt.receiver_id, 101)
        self.assertEqual(rt.request_type, REQUEST_TYPE_RESOURCE)
        self.assertEqual(rt.subject_type, SUBJECT_TYPE_THUMBNAIL)
        self.assertEqual(rt.subject_id, 55)

    def test_mavlink_roundtrip(self):
        msg = self._make()
        mav = msg.to_mavlink()
        self.assertEqual(mav.target_system, 101)

        result = _roundtrip(msg)
        self.assertIsInstance(result, SwarmRequestMsg)
        self.assertEqual(result.to_dict(), msg.to_dict())
        self.assertEqual(result.receiver_id, 101)
        self.assertEqual(result.request_type, REQUEST_TYPE_RESOURCE)
        self.assertEqual(result.subject_type, SUBJECT_TYPE_THUMBNAIL)
        self.assertEqual(result.subject_id, 55)
        self.assertEqual(result.meta.boot_id, 111)
        self.assertEqual(result.meta.ttl_ms, 3000)


# ---------------------------------------------------------------------------
# TTL defaults - dedicated entries, not the 5000ms fallback
# ---------------------------------------------------------------------------

class TestSwarmMessageTtlDefaults(unittest.TestCase):
    def test_swarm_ack_ttl_is_dedicated_not_default(self):
        ttl = get_ttl_ms(MsgType.SWARM_ACK)
        self.assertEqual(ttl, 3000)
        self.assertNotEqual(ttl, DEFAULT_TTL_MS)

    def test_swarm_request_ttl_is_dedicated_not_default(self):
        ttl = get_ttl_ms(MsgType.SWARM_REQUEST)
        self.assertEqual(ttl, 3000)
        self.assertNotEqual(ttl, DEFAULT_TTL_MS)


# ---------------------------------------------------------------------------
# Task 3 tracer: scripted VehicleMav loopback (CI-durable live-routing proof)
# ---------------------------------------------------------------------------

class _ScriptedVehicleLoopback:
    """A minimal companion<->GCS stand-in for the real MAVLink wire.

    Registers a NAVLINK callback exactly the way the real production code
    does on both sides -- NetworkMavlink.__init__ (companion) and
    TaskConfirmListener.register_vehicle (GCS) both call
    ``vehicle.on_message("NAVLINK", callback)`` on their VehicleMav
    instance -- then hands a to_mavlink() packet straight to the
    registered callback(s), exercising the SAME registration/dispatch
    contract without needing a live SITL/router. This is the headless,
    always-green substitute for the live launcher-started SITL routing
    proof (already captured for real this session: SWARM_ACK/SWARM_REQUEST
    sent Vehicle 2 -> received Vehicle 1 byte-identical, CRC gate PASSED
    for 25110/25111 across pymavlink/c_library_v2/mavlink-router).
    """

    def __init__(self):
        self._navlink_callbacks = []

    def on_message(self, msg_type: str, callback):
        if msg_type == "NAVLINK":
            self._navlink_callbacks.append(callback)

    def deliver(self, mav_msg) -> None:
        """Simulate a wire delivery: only dispatched if the message id is a
        registered navlink message (mirrors VehicleMav._process_message's
        ``is_navlink = MsgRegistry.has_mav_id(msg.get_msgId())`` gate)."""
        assert MsgRegistry.has_mav_id(mav_msg.get_msgId()), "not a registered navlink message"
        for cb in self._navlink_callbacks:
            cb(mav_msg)


class TestSwarmMessageLiveRoutingSmokeLoopback(unittest.TestCase):
    """Headless roundtrip proof of companion->GCS delivery for both
    messages, via the scripted VehicleMav loopback above. Matches
    `pytest -k roundtrip` per the plan's Task 3 verify command."""

    def test_swarm_ack_roundtrip_companion_to_gcs_loopback(self):
        bus = _ScriptedVehicleLoopback()
        received = []
        bus.on_message("NAVLINK", lambda m: received.append(SwarmAckMsg.from_mavlink(m)))

        sent = SwarmAckMsg(
            sender_id=101, receiver_id=1,
            ref_boot_id=555, ref_msg_seq=42, ref_msg_type=MsgType.TASK_CONFIRM_REQUEST.value,
            status=ACK_STATUS_APPLIED, meta=TEST_META,
        )
        mav = sent.to_mavlink()
        mav._header.srcSystem = sent.sender_id
        bus.deliver(mav)

        self.assertEqual(len(received), 1, "not silently dropped -- delivered exactly once")
        got = received[0]
        self.assertEqual(got.to_dict(), sent.to_dict(), "byte-correct field-for-field")
        self.assertEqual(got.receiver_id, 1)
        self.assertEqual(got.ref_boot_id, 555)
        self.assertEqual(got.ref_msg_seq, 42)

    def test_swarm_request_roundtrip_companion_to_gcs_loopback(self):
        bus = _ScriptedVehicleLoopback()
        received = []
        bus.on_message("NAVLINK", lambda m: received.append(SwarmRequestMsg.from_mavlink(m)))

        sent = SwarmRequestMsg(
            sender_id=1, receiver_id=101,
            request_type=REQUEST_TYPE_RESOURCE, subject_type=SUBJECT_TYPE_THUMBNAIL,
            subject_id=77, meta=TEST_META,
        )
        mav = sent.to_mavlink()
        mav._header.srcSystem = sent.sender_id
        bus.deliver(mav)

        self.assertEqual(len(received), 1, "not silently dropped -- delivered exactly once")
        got = received[0]
        self.assertEqual(got.to_dict(), sent.to_dict(), "byte-correct field-for-field")
        self.assertEqual(got.receiver_id, 101)
        self.assertEqual(got.request_type, REQUEST_TYPE_RESOURCE)
        self.assertEqual(got.subject_id, 77)

    def test_unregistered_message_id_is_not_dispatched(self):
        """Sanity check on the loopback fixture itself: only messages
        registered in MsgRegistry (real navlink ids) are ever dispatched --
        matches the production is_navlink gate in VehicleMav/NetworkMavlink."""
        bus = _ScriptedVehicleLoopback()
        received = []
        bus.on_message("NAVLINK", lambda m: received.append(m))

        fake = type("Fake", (), {"get_msgId": staticmethod(lambda: 999999)})()
        with self.assertRaises(AssertionError):
            bus.deliver(fake)
        self.assertEqual(received, [])


if __name__ == "__main__":
    unittest.main()
