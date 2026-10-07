import unittest

from pymavlink.dialects.v20.ardupilotmega import MAVLink_message

from navpy.modules.comm.messages.check_msg import CheckInMsg, CheckOutMsg
from navpy.modules.comm.messages.location_msg import LocationMsgData
from navpy.modules.comm.messages.available_task_msg import (
    AvailableTaskRequestMsg,
    AvailableTaskResponseMsg,
    TaskMsgData,
    TaskHandleMsgData,
    TaskAssignMsgData,
    TaskAssignRequestMsg,
    TaskAssignResponseMsg,
    TaskConfirmRequestMsg,
    TaskConfirmResponseMsg,
)
from navpy.modules.comm.messages.log_status_msg import LogStatusMsg
from navpy.modules.comm.messages.msg_meta import MsgMeta
from navpy.modules.comm.messages.types import TaskTypeMsgData


# Default meta for testing - messages with meta will have symmetric roundtrip
TEST_META = MsgMeta(boot_id=12345, msg_seq=1, time_ms=1000000, ttl_ms=5000)


class TestMavlinkConversion(unittest.TestCase):
    def _roundtrip(self, msg):
        mav: MAVLink_message = msg.to_mavlink()
        mav._header.srcSystem = msg.sender_id  # imitate the sender_id in mavlink header
        result = msg.__class__.from_mavlink(mav)
        self.assertIsInstance(result, msg.__class__)

        self.assertEqual(result.to_dict(), msg.to_dict())

    def test_check_in(self):
        self._roundtrip(CheckInMsg(sender_id=1, meta=TEST_META))

    def test_check_out(self):
        loc = LocationMsgData(lat=1.1, lng=2.2, alt=3.3)
        self._roundtrip(CheckOutMsg(sender_id=1, location=loc, meta=TEST_META))

    def test_log_status(self):
        # LogStatusMsg uses STATUSTEXT which doesn't have meta fields
        self._roundtrip(LogStatusMsg(sender_id=1, status="OK"))

    def test_available_task_request(self):
        task = TaskMsgData(
            task_id=1,
            task_type=TaskTypeMsgData.DOCK,
            location=LocationMsgData(lat=1.0, lng=2.0, alt=3.0),
            class_id=4,  # non-default class id preserved exactly via class_id field
        )
        self._roundtrip(AvailableTaskRequestMsg(sender_id=1, tasks=[task], meta=TEST_META))

    def test_available_task_request_class_2(self):
        """Detection class 2 preserved exactly instead of being collapsed to class 1."""
        task = TaskMsgData(
            task_id=1,
            task_type=TaskTypeMsgData.DOCK,
            location=LocationMsgData(lat=1.0, lng=2.0, alt=3.0),
            class_id=2,  # was lossy before, now exact
        )
        self._roundtrip(AvailableTaskRequestMsg(sender_id=1, tasks=[task], meta=TEST_META))

    def test_available_task_response(self):
        handle = TaskHandleMsgData(task_id=1, time_in_min=10.0)
        self._roundtrip(AvailableTaskResponseMsg(sender_id=1, receiver_id=2, tasks=[handle], meta=TEST_META))

    def test_task_assign_request(self):
        task = TaskAssignMsgData(
            task_id=1,
            task_type=TaskTypeMsgData.DOCK,
            location=LocationMsgData(lat=0.0, lng=0.0, alt=0.0),
            class_id=4,  # preserved exactly
        )
        self._roundtrip(TaskAssignRequestMsg(sender_id=1, receiver_id=2, task=task, meta=TEST_META))

    def test_task_assign_response(self):
        self._roundtrip(TaskAssignResponseMsg(sender_id=1, receiver_id=2, task_id=1, is_accepted=True, meta=TEST_META))

    def test_task_confirm_request(self):
        task = TaskMsgData(
            task_id=1,
            task_type=TaskTypeMsgData.DOCK,
            location=LocationMsgData(lat=1.0, lng=2.0, alt=3.0),
            class_id=4,  # preserved exactly
        )
        self._roundtrip(TaskConfirmRequestMsg(sender_id=1, task=task, meta=TEST_META))

    def test_task_confirm_request_from_dict_preserves_metadata(self):
        task = TaskMsgData(
            task_id=1,
            task_type=TaskTypeMsgData.DOCK,
            location=LocationMsgData(lat=1.0, lng=2.0, alt=3.0),
        )
        message = TaskConfirmRequestMsg(sender_id=1, task=task, meta=TEST_META)

        restored = TaskConfirmRequestMsg.from_dict(message.to_dict())

        self.assertEqual(restored.meta, TEST_META)

    def test_task_confirm_response(self):
        self._roundtrip(TaskConfirmResponseMsg(receiver_id=2, task_id=1, is_confirmed=True, meta=TEST_META))

    def test_task_confirm_response_from_dict_preserves_metadata(self):
        message = TaskConfirmResponseMsg(
            receiver_id=2,
            task_id=1,
            is_confirmed=True,
            meta=TEST_META,
        )

        restored = TaskConfirmResponseMsg.from_dict(message.to_dict())

        self.assertEqual(restored.meta, TEST_META)


class TestClassToTaskType(unittest.TestCase):
    """Tests for class_to_task_type mapping."""

    def test_dock_class_maps_to_dock_task(self):
        from navpy.modules.comm.messages.types import class_to_task_type
        from navpy.modules.vision.vision_class_profile import DOCK_DETECT_CLASS_ID
        self.assertEqual(class_to_task_type(DOCK_DETECT_CLASS_ID), TaskTypeMsgData.DOCK)

    def test_unknown_class_returns_unknown(self):
        from navpy.modules.comm.messages.types import class_to_task_type
        self.assertEqual(class_to_task_type(99), TaskTypeMsgData.UNKNOWN)

    def test_task_type_members_are_dock_and_unknown(self):
        self.assertEqual({m.name for m in TaskTypeMsgData}, {"DOCK", "UNKNOWN"})


if __name__ == "__main__":
    unittest.main()
