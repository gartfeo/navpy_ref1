import unittest

from navpy.modules.comm.messages.location_msg import LocationMsgData


class MyTestCase(unittest.TestCase):
    def test_serialization_deserialization(self):
        # Arrange
        original_location = LocationMsgData(12.34, 56.78, 90.0)

        # Act
        dict_data = original_location.to_dict()
        reconstructed_location = LocationMsgData.from_dict(dict_data)

        # Assert
        self.assertEqual(original_location.lat, reconstructed_location.lat)
        self.assertEqual(original_location.lng, reconstructed_location.lng)
        self.assertEqual(original_location.alt, reconstructed_location.alt)


if __name__ == '__main__':
    unittest.main()
