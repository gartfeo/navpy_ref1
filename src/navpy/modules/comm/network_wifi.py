import base64
import json
import time
import threading
from typing import Optional

import zmq

from navpy.args.conn.wifi_network_args import WifiNetworkArgs
from navpy.modules.comm.message_filter import MessageFilter
from navpy.modules.comm.network_abc import NetworkAbc, DEFAULT_CHUNK_SIZE
from navpy.logger.cache_logger import ILogger
from navpy.utils.serializer_abc import JsonSerializer


class NetworkWifi(NetworkAbc):

    def __init__(self, node_id, args: WifiNetworkArgs, logger: ILogger,
                 message_filter: Optional[MessageFilter] = None):
        super().__init__(node_id, JsonSerializer(), logger, message_filter,
                         image_chunk_size=DEFAULT_CHUNK_SIZE)
        self.port = args.self_port
        self.context = zmq.Context()

        # Prepare a socket for broadcasting
        self.broadcast_socket = self.context.socket(zmq.PUB)
        self.broadcast_socket.bind(f"tcp://*:{self.port}")

        # Prepare a socket for receiving
        self.receive_socket = self.context.socket(zmq.SUB)
        # Connect to the ports in listening_ports
        for listening_address in args.peers:
            self.receive_socket.connect(listening_address)
        self.receive_socket.setsockopt_string(zmq.SUBSCRIBE, "")

        # Start listening in a separate thread
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._listen, daemon=True)
        self.thread.start()

        self._wait_sockets_initialize()
        self.logger = logger
        logger.info(f'Network: Creating WIFI zmq node: {args.self_port}, peers: {args.peers}')

    def broadcast_data(self, data):
        self.broadcast_socket.send_string(data.decode('utf-8'))

    def _listen(self):
        try:
            while not self.stop_event.is_set():
                message = self.receive_socket.recv_string()
                self.receive_packet(message.encode('utf-8'))
                time.sleep(0.01)
        except zmq.error.ContextTerminated:
            # Context was terminated, exit gracefully.
            pass
        except Exception as e:
            self.logger.error(f"Error in _listen: {e}")

    def close(self):
        if self.is_closed:
            return
        self.is_closed = True
        self.stop_event.set()
        time.sleep(0.5)  # Allow thread some time to exit before we close everything
        self.broadcast_socket.close()
        self.receive_socket.close()
        self.context.term()
        self.thread.join()
        self.logger.info("NetworkWifi closed")

    def _send_image_header(self, target_id: int, total_size: int, num_chunks: int) -> None:
        """Send image header as JSON message."""
        header = {
            'type': 'img_header',
            'tid': target_id,
            'size': total_size,
            'chunks': num_chunks,
        }
        self.broadcast_socket.send_string(json.dumps(header))

    def _send_image_chunk(self, target_id: int, sequence: int, data: bytes) -> None:
        """Send image chunk as JSON message with base64 encoded data."""
        chunk = {
            'type': 'img_chunk',
            'tid': target_id,
            'seq': sequence,
            'data': base64.b64encode(data).decode('utf-8'),
        }
        self.broadcast_socket.send_string(json.dumps(chunk))

    @staticmethod
    def _wait_sockets_initialize():
        time.sleep(0.5)
