import unittest

from navpy.modules.comm.messages.available_task_msg import TaskAssignResponseMsg, TaskAssignMsgData, TaskAssignRequestMsg, \
    TaskHandleMsgData, AvailableTaskResponseMsg, TaskMsgData, AvailableTaskRequestMsg
from navpy.modules.comm.messages.check_msg import CheckInMsg, CheckOutMsg
from navpy.modules.comm.messages.location_msg import LocationMsgData
from navpy.modules.comm.messages.log_status_msg import LogStatusMsg
from navpy.modules.comm.messages.msg_abc import MsgSerializer
from navpy.modules.comm.messages.types import TaskTypeMsgData
from navpy.modules.comm.serial.serial_interface_ebyte import MAX_PACKET_SIZE


class TestMessageSerializationLength(unittest.TestCase):
    def test_message_serialization_length(self):
        # Create instances of each message type
        messages = [
            CheckInMsg(sender_id=1),
            CheckOutMsg(sender_id=1, location=LocationMsgData(lat=0.0, lng=0.0, alt=0.0)),
            LogStatusMsg(sender_id=1, status="OK"),
            AvailableTaskRequestMsg(
                sender_id=1,
                tasks=[
                    TaskMsgData(
                        task_id=1,
                        task_type=TaskTypeMsgData.DOCK,
                        location=LocationMsgData(lat=40.1545411, lng=-105.1127835, alt=1609.34)
                    ),
                ]
            ),
            AvailableTaskResponseMsg(
                sender_id=1,
                receiver_id=2,
                tasks=[
                    TaskHandleMsgData(task_id=1, time_in_min=10.0)
                ]
            ),
            TaskAssignRequestMsg(
                sender_id=1,
                receiver_id=2,
                task=TaskAssignMsgData(
                    task_id=1,
                    task_type=TaskTypeMsgData.DOCK,
                    location=LocationMsgData(lat=0.0, lng=0.0, alt=0.0)
                )
            ),
            TaskAssignResponseMsg(
                sender_id=1,
                receiver_id=2,
                task_id=1,
                is_accepted=True
            )
        ]

        # Test serialization length
        for msg in messages:
            json_str = MsgSerializer.to_json(msg)
            msg_length = len(json_str.encode('utf-8'))
            print(f"{msg.__class__.__name__} serialization length: {msg_length} bytes")
            self.assertLessEqual(msg_length, MAX_PACKET_SIZE, f"{msg.__class__.__name__} exceeds 190 bytes")


if __name__ == "__main__":
    unittest.main()
