import base64
import json
from typing import Optional

from navpy.args.conn.serial_network_args import SerialNetworkArgs
from navpy.modules.comm.message_filter import MessageFilter
from navpy.modules.comm.network_abc import NetworkAbc
from navpy.modules.comm.serial.serial_interface_rfd900 import SerialInterfaceRFD900
from navpy.modules.comm.serial.serial_listener import ISerialListener
from navpy.modules.comm.serial.sim.serial_sim_client import SerialSimClient
from navpy.logger.cache_logger import ILogger
from navpy.utils.serializer_abc import JsonSerializer

# Smaller chunk size for serial (limited bandwidth)
SERIAL_CHUNK_SIZE = 512


class NetworkSerial(NetworkAbc, ISerialListener):
    def __init__(self, node_id, args: SerialNetworkArgs, logger: ILogger, simulation=False,
                 message_filter: Optional[MessageFilter] = None):
        super().__init__(node_id, JsonSerializer(), logger, message_filter,
                         image_chunk_size=SERIAL_CHUNK_SIZE,
                         inter_chunk_delay_ms=10.0)  # Slower for serial
        self.node_id = node_id
        self.logger = logger

        # Initialize the Serial communication layer
        if simulation:
            self.serial_com = SerialSimClient('localhost', 5000, logger)
        else:
            self.serial_com = SerialInterfaceRFD900(args, logger)

        self.serial_com.register_listener(self)

    def broadcast_data(self, data):
        self.serial_com.send_data(data)

    def _send_image_header(self, target_id: int, total_size: int, num_chunks: int) -> None:
        """Send image header as JSON message."""
        header = {
            'type': 'img_header',
            'tid': target_id,
            'size': total_size,
            'chunks': num_chunks,
        }
        self.serial_com.send_data(json.dumps(header).encode('utf-8'))

    def _send_image_chunk(self, target_id: int, sequence: int, data: bytes) -> None:
        """Send image chunk as JSON message with base64 encoded data."""
        chunk = {
            'type': 'img_chunk',
            'tid': target_id,
            'seq': sequence,
            'data': base64.b64encode(data).decode('utf-8'),
        }
        self.serial_com.send_data(json.dumps(chunk).encode('utf-8'))

    def close(self):
        if self.is_closed:
            return
        self.is_closed = True
        self.serial_com.close()
