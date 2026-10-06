import unittest

from navpy.modules.comm.messages.check_msg import CheckInMsg, CheckOutMsg
from navpy.modules.comm.messages.location_msg import LocationMsgData


class CheckInOutMsgTest(unittest.TestCase):
    def test_check_in_msg_serialization(self):
        # Arrange
        check_in_msg = CheckInMsg(sender_id=1)

        # Act
        dict_data = check_in_msg.to_dict()
        reconstructed_msg = CheckInMsg.from_dict(dict_data)

        # Assert
        self.assertEqual(check_in_msg.sender_id, reconstructed_msg.sender_id)
        self.assertEqual(check_in_msg.msg_type(), reconstructed_msg.msg_type())

    def test_check_out_msg_serialization(self):
        # Arrange
        location = LocationMsgData(12.34, 56.78, 90.0)
        check_out_msg = CheckOutMsg(sender_id=1, location=location)

        # Act
        dict_data = check_out_msg.to_dict()
        reconstructed_msg = CheckOutMsg.from_dict(dict_data)

        # Assert
        self.assertEqual(check_out_msg.sender_id, reconstructed_msg.sender_id)
        self.assertEqual(check_out_msg.location.lat, reconstructed_msg.location.lat)
        self.assertEqual(check_out_msg.location.lng, reconstructed_msg.location.lng)
        self.assertEqual(check_out_msg.location.alt, reconstructed_msg.location.alt)


if __name__ == '__main__':
    unittest.main()
