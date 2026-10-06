import unittest

from navpy.modules.comm.messages.check_msg import CheckOutMsg
from navpy.modules.comm.messages.location_msg import LocationMsgData
from navpy.modules.comm.messages.msg_abc import MsgSerializer


class MsgSerializerTest(unittest.TestCase):
    def test_serialization_deserialization(self):
        # Arrange
        location = LocationMsgData(12.34, 56.78, 90.0)
        check_out_msg = CheckOutMsg(sender_id=1, location=location)

        # Act
        json_str = MsgSerializer.to_json(check_out_msg)
        reconstructed_msg = MsgSerializer.from_json(json_str)

        # Assert
        self.assertIsInstance(reconstructed_msg, CheckOutMsg)
        self.assertEqual(check_out_msg.sender_id, reconstructed_msg.sender_id)
        self.assertEqual(check_out_msg.location.lat, reconstructed_msg.location.lat)

if __name__ == '__main__':
    unittest.main()