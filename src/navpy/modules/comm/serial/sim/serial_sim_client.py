import socket
import threading
import struct
from typing import Optional

from navpy.modules.comm.serial.serial_listener import ISerialListener
from navpy.logger.cache_logger import ILogger


class SerialSimClient:
    """Client that connects to the SerialSimulatorServer."""

    def __init__(self, host, port, logger: ILogger):
        self.listener: Optional[ISerialListener] = None
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((host, port))
        self.logger = logger
        self.running = True
        threading.Thread(target=self.receive_loop, daemon=True).start()

        logger.info(f'Network: Creating Serial Socket Sim: {host}, baud: {port}')

    def register_listener(self, listener: ISerialListener):
        """
        Register a listener to receive incoming data packets.

        Args:
            listener (ISerialListener): The listener to register.
        """
        self.listener = listener
        self.logger.info("Serial listener registered.")

    def send_data(self, packet_bytes):
        """Transmit a packet to the server."""
        length_prefix = struct.pack('>I', len(packet_bytes))
        self.client_socket.sendall(length_prefix + packet_bytes)

    def receive_loop(self):
        """Continuously receive data from the server."""
        try:
            while self.running:
                # Receive the length prefix
                length_prefix = self.client_socket.recv(4)
                if not length_prefix:
                    break
                message_length = struct.unpack('>I', length_prefix)[0]
                # Receive the message based on the length
                message = b''
                while len(message) < message_length:
                    chunk = self.client_socket.recv(message_length - len(message))
                    if not chunk:
                        break
                    message += chunk
                if not message:
                    break
                if self.listener is not None:
                    self.listener.receive_packet(message)
        except Exception as e:
            print(f"Exception in receive_loop: {e}")
        finally:
            self.client_socket.close()

    def close(self):
        """Close the client socket."""
        self.running = False
        self.client_socket.close()
