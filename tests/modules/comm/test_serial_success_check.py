import unittest
from unittest import mock
import time

from navpy.modules.comm.serial.serial_interface_ebyte import SerialInterfaceEbyte, MAX_PACKET_SIZE
from navpy.args.conn.serial_network_args import SerialNetworkArgs
from navpy.logger.cache_logger import ConsoleLogger
from navpy.modules.comm.serial.serial_listener import ISerialListener

# Fake serial port to simulate write/read behavior
class FakeSerial:
    def __init__(self):
        self.written = []
        self.to_read = bytearray()
        self.in_waiting = 0

    def write(self, data):
        self.written.append(data)

    def read(self, n):
        chunk = self.to_read[:n]
        self.to_read = self.to_read[n:]
        self.in_waiting = len(self.to_read)
        return bytes(chunk)

    def close(self):
        pass

# Dummy listener to record received packets
class DummyListener(ISerialListener):
    def __init__(self):
        self.received = []
    def receive_packet(self, data: bytes):
        self.received.append(data)

class TestSerialInterfaceEbyte(unittest.TestCase):
    def setUp(self):
        # Patch serial.Serial to use FakeSerial
        self.fake = FakeSerial()
        patcher = mock.patch('serial.serial_for_url', return_value=self.fake)
        self.addCleanup(patcher.stop)
        self.mock_serial = patcher.start()

        args = mock.MagicMock(spec=SerialNetworkArgs)
        args.conn = 'COM1'
        args.baud = 9600
        self.logger = ConsoleLogger()
        # Use short timeout for tests
        self.iface = SerialInterfaceEbyte(args, self.logger)
        self.listener = DummyListener()
        self.iface.register_listener(self.listener)

    def tearDown(self):
        # Stop threads
        self.iface.running = False
        self.iface.send_thread.join(timeout=1)
        self.iface.receive_thread.join(timeout=1)

    def test_wait_for_success(self):
        data1 = b'first'
        data2 = b'second'
        self.iface.send_data(data1)
        self.iface.send_data(data2)

        # first packet must appear in write buffer
        self._wait_for(lambda: self.fake.written == [data1])

        # inject SUCCESS and let receive thread free the queue
        self.fake.to_read += b'SUCCESS\r\n'
        self.fake.in_waiting = len(self.fake.to_read)

        # second packet must now be written
        self._wait_for(lambda: self.fake.written == [data1, data2])

    def test_frame_parsing(self):
        # Build a frame: type=0xC1, length=3, net=1, src=2, tgt=3, payload=b'XYZ'
        header = bytes([0xC1, 3, 1, 0, 2, 0, 3, 0])
        frame = header + b'XYZ'
        self.fake.to_read += frame
        self.fake.in_waiting = len(self.fake.to_read)

        # Allow receive thread to parse
        time.sleep(0.1)
        self.assertEqual(self.listener.received, [b'XYZ'])

    def test_text_line_routed_to_listener(self):
        # Send a custom text not in module_responses
        self.fake.to_read += b'HELLO_WORLD\r\n'
        self.fake.in_waiting = len(self.fake.to_read)

        time.sleep(0.1)
        self.assertIn(b'HELLO_WORLD', self.listener.received)

    def test_oversize_is_not_queued(self):
        oversize = b'A' * (MAX_PACKET_SIZE + 1)
        self.iface.send_data(oversize)
        time.sleep(0.1)
        # Nothing should be written
        self.assertEqual(self.fake.written, [])

    def _wait_for(self, cond, timeout=1.0, step=0.005):
        """Poll until cond() is True or timeout hits."""
        end = time.time() + timeout
        while time.time() < end:
            if cond():
                return
            time.sleep(step)
        self.fail("timeout waiting for condition")

if __name__ == '__main__':
    unittest.main()
