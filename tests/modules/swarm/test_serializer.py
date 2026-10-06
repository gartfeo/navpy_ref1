import unittest

from navpy.modules.comm.messages.available_task_msg import TaskAssignResponseMsg, TaskAssignRequestMsg, TaskAssignMsgData, \
    AvailableTaskResponseMsg, TaskHandleMsgData, AvailableTaskRequestMsg, TaskMsgData, TaskConfirmRequestMsg, \
    TaskConfirmResponseMsg
from navpy.modules.comm.messages.check_msg import CheckOutMsg, CheckInMsg
from navpy.modules.comm.messages.location_msg import LocationMsgData
from navpy.modules.comm.messages.msg_abc import MsgSerializer
from navpy.modules.comm.messages.types import TaskTypeMsgData


class SerializerTest(unittest.TestCase):
    def test_checkin(self):
        check_in_msg = CheckInMsg(123)
        json_str = MsgSerializer.to_json(check_in_msg)
        self.assertIsNotNone(json_str, "Serialization failed, JSON string is None")

        deserialized_msg = MsgSerializer.from_json(json_str)
        self.assertIsInstance(deserialized_msg, CheckInMsg, "Deserialization failed, object is not of type CheckInMsg")
        self.assertEqual(check_in_msg.sender_id, deserialized_msg.sender_id,
                         "Deserialized sender_id does not match the original")

        print("Serialized:", json_str)
        print("Deserialized:", deserialized_msg)

    def test_checkout(self):
        location = LocationMsgData(40.7128, -74.0060, 10)
        check_out_msg = CheckOutMsg(123, location)
        json_str = MsgSerializer.to_json(check_out_msg)
        self.assertIsNotNone(json_str, "Serialization failed, JSON string is None")

        deserialized_msg = MsgSerializer.from_json(json_str)
        self.assertIsInstance(deserialized_msg, CheckOutMsg,
                              "Deserialization failed, object is not of type CheckOutMsg")
        self.assertEqual(check_out_msg.sender_id, deserialized_msg.sender_id,
                         "Deserialized sender_id does not match the original")
        self.assertEqual(check_out_msg.location.lat, deserialized_msg.location.lat,
                         "Deserialized location.lat does not match the original")
        self.assertEqual(check_out_msg.location.lng, deserialized_msg.location.lng,
                         "Deserialized location.lng does not match the original")
        self.assertEqual(check_out_msg.location.alt, deserialized_msg.location.alt,
                         "Deserialized location.alt does not match the original")

    def test_task_confirm_request_msg(self):
        location = LocationMsgData(40.7128, -74.0060, 898.5884136)
        task = TaskMsgData(11, TaskTypeMsgData.MEDIUM, location)
        msg = TaskConfirmRequestMsg(3221, task)

        json_str = MsgSerializer.to_json(msg)
        self.assertIsNotNone(json_str, "Serialization failed, JSON string is None")

        deserialized_msg = MsgSerializer.from_json(json_str) # type: TaskConfirmRequestMsg
        self.assertIsInstance(deserialized_msg, TaskConfirmRequestMsg,
                              "Deserialization failed, object is not of type TaskConfirmRequestMsg")
        self.assertEqual(msg.sender_id, deserialized_msg.sender_id,
                         "Deserialized sender_id does not match the original")
        self.assertEqual(msg.receiver_id, deserialized_msg.receiver_id,
                         "Deserialized receiver_id does not match the original")
        self.assertEqual(msg.task.task_id, deserialized_msg.task.task_id,
                         "Deserialized task_id does not match the original")
        self.assertEqual(msg.task.task_type, deserialized_msg.task.task_type,
                         "Deserialized task_type does not match the original")
        self.assertEqual(msg.task.location.lat, deserialized_msg.task.location.lat,
                         "Deserialized location.lat does not match the original")
        self.assertEqual(msg.task.location.lng, deserialized_msg.task.location.lng,
                         "Deserialized location.lng does not match the original")
        self.assertEqual(msg.task.location.alt, deserialized_msg.task.location.alt,
                         "Deserialized location.alt does not match the original")

    def test_task_confirm_response_msg(self):
        msg = TaskConfirmResponseMsg(321, 123, True)
        json_str = MsgSerializer.to_json(msg)
        self.assertIsNotNone(json_str, "Serialization failed, JSON string is None")

        deserialized_msg = MsgSerializer.from_json(json_str) # type: TaskConfirmResponseMsg
        self.assertIsInstance(deserialized_msg, TaskConfirmResponseMsg,
                                "Deserialization failed, object is not of type TaskConfirmResponseMsg")
        self.assertEqual(msg.sender_id, deserialized_msg.sender_id,
                            "Deserialized sender_id does not match the original")
        self.assertEqual(msg.receiver_id, deserialized_msg.receiver_id,
                            "Deserialized receiver_id does not match the original")
        self.assertEqual(msg.task_id, deserialized_msg.task_id, "Deserialized task_id does not match the original")
        self.assertEqual(msg.is_confirmed, deserialized_msg.is_confirmed,
                            "Deserialized is_confirmed does not match the original")


    def test_available_task_request_msg(self):
        location = LocationMsgData(40.7128, -74.0060, 10)
        task = TaskMsgData(333, TaskTypeMsgData.SMALL, location)
        msg = AvailableTaskRequestMsg(123, [task])
        json_str = MsgSerializer.to_json(msg)
        self.assertIsNotNone(json_str, "Serialization failed, JSON string is None")

        deserialized_msg = MsgSerializer.from_json(json_str)
        self.assertIsInstance(deserialized_msg, AvailableTaskRequestMsg,
                              "Deserialization failed, object is not of type AvailableTaskRequestMsg")
        self.assertEqual(msg.sender_id, deserialized_msg.sender_id,
                         "Deserialized sender_id does not match the original")
        self.assertEqual(msg.tasks[0].task_id, deserialized_msg.tasks[0].task_id,
                         "Deserialized task_id does not match the original")
        self.assertEqual(msg.tasks[0].task_type, deserialized_msg.tasks[0].task_type,
                         "Deserialized task_type does not match the original")
        self.assertEqual(msg.tasks[0].location.lat, deserialized_msg.tasks[0].location.lat,
                         "Deserialized location.lat does not match the original")
        self.assertEqual(msg.tasks[0].location.lng, deserialized_msg.tasks[0].location.lng,
                         "Deserialized location.lng does not match the original")
        self.assertEqual(msg.tasks[0].location.alt, deserialized_msg.tasks[0].location.alt,
                         "Deserialized location.alt does not match the original")

    def test_available_task_response_msg(self):
        task_handle_msg = TaskHandleMsgData(123, 30)
        msg = AvailableTaskResponseMsg(321, 123, [task_handle_msg])
        json_str = MsgSerializer.to_json(msg)
        self.assertIsNotNone(json_str, "Serialization failed, JSON string is None")

        deserialized_msg = MsgSerializer.from_json(json_str)
        self.assertIsInstance(deserialized_msg, AvailableTaskResponseMsg,
                              "Deserialization failed, object is not of type AvailableTaskResponseMsg")
        self.assertEqual(msg.sender_id, deserialized_msg.sender_id,
                         "Deserialized sender_id does not match the original")
        self.assertEqual(msg.receiver_id, deserialized_msg.receiver_id, "Deserialized receiver_id does not match the original")
        self.assertEqual(msg.tasks[0].task_id, deserialized_msg.tasks[0].task_id,
                         "Deserialized task_id does not match the original")
        self.assertEqual(msg.tasks[0].time_in_min, deserialized_msg.tasks[0].time_in_min,
                         "Deserialized time_in_min does not match the original")

    def test_task_assign_request_msg(self):
        location = LocationMsgData(40.7128, -74.0060, 10)
        task = TaskAssignMsgData(123, TaskTypeMsgData.SMALL, location)
        msg = TaskAssignRequestMsg(123, 321, task)
        json_str = MsgSerializer.to_json(msg)
        self.assertIsNotNone(json_str, "Serialization failed, JSON string is None")

        deserialized_msg = MsgSerializer.from_json(json_str)
        self.assertIsInstance(deserialized_msg, TaskAssignRequestMsg,
                              "Deserialization failed, object is not of type TaskAssignRequestMsg")
        self.assertEqual(msg.sender_id, deserialized_msg.sender_id,
                         "Deserialized sender_id does not match the original")
        self.assertEqual(msg.receiver_id, deserialized_msg.receiver_id,
                         "Deserialized receiver_id does not match the original")
        self.assertEqual(msg.task.task_id, deserialized_msg.task.task_id,
                         "Deserialized task_id does not match the original")
        self.assertEqual(msg.task.task_type, deserialized_msg.task.task_type,
                         "Deserialized task_type does not match the original")
        self.assertEqual(msg.task.location.lat, deserialized_msg.task.location.lat,
                         "Deserialized location.lat does not match the original")
        self.assertEqual(msg.task.location.lng, deserialized_msg.task.location.lng,
                         "Deserialized location.lng does not match the original")
        self.assertEqual(msg.task.location.alt, deserialized_msg.task.location.alt,
                         "Deserialized location.alt does not match the original")

    def test_task_assign_response_msg(self):
        msg = TaskAssignResponseMsg(321, 123, 333, True)
        json_str = MsgSerializer.to_json(msg)
        self.assertIsNotNone(json_str, "Serialization failed, JSON string is None")

        deserialized_msg = MsgSerializer.from_json(json_str)
        self.assertIsInstance(deserialized_msg, TaskAssignResponseMsg,
                              "Deserialization failed, object is not of type TaskAssignResponseMsg")
        self.assertEqual(msg.sender_id, deserialized_msg.sender_id,
                         "Deserialized responder_id does not match the original")
        self.assertEqual(msg.receiver_id, deserialized_msg.receiver_id,
                         "Deserialized receiver_id does not match the original")
        self.assertEqual(msg.task_id, deserialized_msg.task_id, "Deserialized task_id does not match the original")
        self.assertEqual(msg.is_accepted, deserialized_msg.is_accepted,
                         "Deserialized is_accepted does not match the original")


if __name__ == "__main__":
    unittest.main()
