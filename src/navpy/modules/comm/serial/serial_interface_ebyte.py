import random
import threading
import time
from typing import Optional

import serial

from navpy.args.conn.serial_network_args import SerialNetworkArgs
from navpy.modules.comm.serial.serial_listener import ISerialListener
from navpy.logger.cache_logger import ILogger

MAX_PACKET_SIZE = 200
LOOP_WAIT = 0.01
SEND_DELAY_MIN = 0.1  # Delay between sends to avoid flooding the serial port
SEND_DELAY_MAX = 0.3  # Delay between sends to avoid flooding the serial port


class SerialInterfaceEbyte:

    def __init__(self, args: SerialNetworkArgs, logger: ILogger):
        self.logger = logger
        self.listener: Optional[ISerialListener] = None

        try:
            self.serial_port = serial.serial_for_url(args.conn, args.baud, timeout=0)
            self.logger.info(f"Serial port {args.conn} opened with baud rate {args.baud}")
        except serial.SerialException as e:
            self.logger.error(f"Failed to open serial port {args.conn}: {e}")
            raise

        self.send_buffer = []
        self.running = True
        self.send_lock = threading.Lock()
        self.receive_lock = threading.Lock()
        self.receive_buffer = bytearray()  # Buffer to accumulate incoming data
        self.send_thread = threading.Thread(target=self._send_data_thread)
        self.receive_thread = threading.Thread(target=self._receive_data_thread)
        self.send_thread.daemon = True
        self.receive_thread.daemon = True
        self.send_thread.start()
        self.receive_thread.start()
        self.logger.info("SerialInterfaceEbyte initialized and threads started.")

    def register_listener(self, listener: ISerialListener):
        """
        Register a listener to receive incoming data packets.

        Args:
            listener (ISerialListener): The listener to register.
        """
        self.listener = listener
        self.logger.info("Serial listener registered.")

    def send_data(self, data: bytes):
        """
        Queue data to be sent over the serial interface.

        Args:
            data (bytes): The data to send.
        """
        if not data:
            self.logger.warning("No data provided to send_data.")
            return

        if len(data) > MAX_PACKET_SIZE:
            self.logger.error(f"Data exceeds maximum packet size of {MAX_PACKET_SIZE} bytes.")
            return

        with self.send_lock:
            self.send_buffer.append(data)
        self.logger.debug(f"Data queued for sending. Packet size: {len(data)} bytes.")

    def _send_data_thread(self):
        """
        Thread method for sending data without blocking the main thread.
        """
        while self.running:
            try:
                with self.send_lock:
                    if self.send_buffer:
                        packet = self.send_buffer.pop(0)
                    else:
                        packet = None

                if packet:
                    time.sleep(random.uniform(SEND_DELAY_MIN, SEND_DELAY_MAX))
                    self.serial_port.write(packet)
                    self.logger.debug(f"Packet sent. Packet size: {len(packet)} bytes.")
                else:
                    time.sleep(LOOP_WAIT)
            except Exception as e:
                self.logger.error(f"Error in send thread: {e}")

    def _receive_data_thread(self):
        """
        Thread method for receiving data without blocking the main thread.
        """
        while self.running:
            try:
                data = self.serial_port.read(self.serial_port.in_waiting or 1)
                if data:
                    with self.receive_lock:
                        self.receive_buffer.extend(data)
                        self.logger.debug(f"Data received. Buffer size: {len(self.receive_buffer)} bytes.")
                        self._process_receive_buffer()
                else:
                    time.sleep(0.01)
            except Exception as e:
                self.logger.error(f"Error in receive thread: {e}")

    def _process_receive_buffer(self):
        """
        Process the receive buffer to extract complete messages.
        """
        while True:
            if not self.receive_buffer:
                break

            # Check for ASCII response messages ending with \r\n
            newline_index = self.receive_buffer.find(b'\r\n')
            if newline_index != -1:
                # Found a line ending with \r\n
                line = self.receive_buffer[:newline_index]
                del self.receive_buffer[:newline_index + 2]  # Remove line and \r\n
                response = line.decode('utf-8', errors='ignore').strip()
                self.logger.info(f"Response received: {response}")
                # List of known module responses
                module_responses = ['SUCCESS', 'NO ROUTE', 'NO ACK', 'ERROR', 'AT+RESET=OK', 'AT+OPTION=OK',
                                    'AT+HEAD=OK']
                if response in module_responses or response.startswith('AT+'):
                    # Handle module responses
                    self.logger.info(f"Module response: {response}")
                    # You can add additional handling here if needed
                elif self.listener:
                    self.listener.receive_packet(response.encode('utf-8'))
                continue

            # Check if we have at least enough data for a frame header
            if len(self.receive_buffer) < 8:
                # Not enough data to parse a frame header
                break

            frame_type = self.receive_buffer[0]
            if frame_type in [0xC1, 0xC2, 0xC3, 0xC4]:
                # Frame type recognized
                data_length = self.receive_buffer[1]
                total_length = 8 + data_length  # frame header (8 bytes) + data

                if len(self.receive_buffer) < total_length:
                    # Not enough data yet
                    break

                frame = self.receive_buffer[:total_length]
                del self.receive_buffer[:total_length]

                # Now parse the frame
                self._parse_frame(frame)
            else:
                # Unknown data, discard one byte
                self.logger.warning(f"Unknown data in buffer, discarding one byte.")
                del self.receive_buffer[0]
                continue

    def _parse_frame(self, frame: bytes):
        """
        Parse a complete frame and notify the listener.
        """
        if len(frame) < 8:
            self.logger.error("Frame too short to parse.")
            return

        frame_type = frame[0]
        data_length = frame[1]

        if len(frame) != 8 + data_length:
            self.logger.error("Frame length does not match data length.")
            return

        # Extract frame fields
        network_id = int.from_bytes(frame[2:4], byteorder='little')
        initial_address = int.from_bytes(frame[4:6], byteorder='little')
        target_address = int.from_bytes(frame[6:8], byteorder='little')
        user_data = frame[8:]

        self.logger.debug(f"Frame received: type={hex(frame_type)}, data_length={data_length}, "
                          f"network_id={hex(network_id)}, initial_address={hex(initial_address)}, "
                          f"target_address={hex(target_address)}, user_data={user_data}")

        if self.listener:
            self.listener.receive_packet(user_data)
        else:
            self.logger.warning("No listener registered to receive data.")

    def close(self):
        """
        Safely close the serial interface and stop background threads.
        """
        self.running = False
        self.send_thread.join()
        self.receive_thread.join()
        self.serial_port.close()
        self.logger.info("SerialInterfaceEbyte closed.")
