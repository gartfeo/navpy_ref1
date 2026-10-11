import unittest

from navpy.core.link.task_assignment_msg import (
    TaskAssignResponseMsg,
    TaskAssignRequestMsg,
)
from navpy.core.link.task_message_data import (
    TaskAssignMsgData,
    TaskHandleMsgData,
    TaskMsgData,
)
from navpy.core.link.task_availability_msg import (
    AvailableTaskResponseMsg,
    AvailableTaskRequestMsg,
)
from navpy.core.link.check_msg import CheckInMsg, CheckOutMsg
from navpy.core.link.location_msg import LocationMsgData
from navpy.core.link.log_status_msg import LogStatusMsg
from navpy.core.link.msg_abc import MsgSerializer
from navpy.core.link.msg_meta import MsgMeta
from navpy.core.link.swarm_ack_msg import ACK_STATUS_APPLIED, SwarmAckMsg
from navpy.core.link.types import MsgType, TaskTypeMsgData
from navpy.plugins.radio.serial.serial_interface_ebyte import MAX_PACKET_SIZE


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
            ),
            SwarmAckMsg(
                sender_id=1,
                receiver_id=2,
                ref_boot_id=0xFFFFFFFF,
                ref_msg_seq=0xFFFFFFFF,
                ref_msg_type=MsgType.TASK_ASSIGN_RESPONSE.value,
                status=ACK_STATUS_APPLIED,
                meta=MsgMeta(
                    boot_id=0xFFFFFFFF,
                    msg_seq=0xFFFFFFFF,
                    time_ms=1753280000123,
                    ttl_ms=5000,
                ),
            ),
        ]

        # Test serialization length
        for msg in messages:
            json_str = MsgSerializer.to_json(msg)
            msg_length = len(json_str.encode('utf-8'))
            print(f"{msg.__class__.__name__} serialization length: {msg_length} bytes")
            self.assertLessEqual(msg_length, MAX_PACKET_SIZE, f"{msg.__class__.__name__} exceeds 190 bytes")


if __name__ == "__main__":
    unittest.main()
