import unittest

from navpy.modules.comm.messages.check_msg import CheckInMsg
from navpy.modules.comm.messages.msg_abc import MsgRegistry
from navpy.modules.comm.messages.types import MsgType


class MsgRegistryTest(unittest.TestCase):
    def test_message_registration(self):
        # Act
        msg_class = MsgRegistry.get_class(MsgType.CHECK_IN)

        # Assert
        self.assertIsNotNone(msg_class)
        self.assertEqual(msg_class, CheckInMsg)

    def test_message_registration_with_mavlink_id(self):
        # Act
        msg_class = MsgRegistry.get_class_by_mav_id(CheckInMsg.mav_id())

        # Assert
        self.assertIsNotNone(msg_class)
        self.assertEqual(msg_class, CheckInMsg)

    def test_unregistered_message(self):
        # Act
        msg_class = MsgRegistry.get_class(MsgType.UNKNOWN)

        # Assert
        self.assertIsNone(msg_class)

    def test_has_mav_id(self):
        # Act
        has_mav_id = MsgRegistry.has_mav_id(CheckInMsg.mav_id())

        # Assert
        self.assertTrue(has_mav_id)


if __name__ == '__main__':
    unittest.main()
