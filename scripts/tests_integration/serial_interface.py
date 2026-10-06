import argparse
import time

from navpy.args.conn.serial_network_args import SerialNetworkArgs
from navpy.modules.comm.serial.serial_interface_ebyte import SerialInterfaceEbyte
from navpy.modules.comm.serial.serial_listener import ISerialListener
from navpy.logger.cache_logger import ConsoleLogger

# Define the COM ports for the transmitter and receiver
TRANSMITTER_PORT = 'COM10'  # Replace with your transmitter COM port
RECEIVER_PORT = 'COM8'  # Replace with your receiver COM port
BAUD_RATE = 115200  # Adjust as needed


class TestNode(ISerialListener):
    def __init__(self, node_number, args: SerialNetworkArgs, logger: ConsoleLogger):
        self.node_id = node_number
        self.logger = logger

        # Initialize SerialInterfaceEbyte
        self.serial_com = SerialInterfaceEbyte(args, logger)
        self.serial_com.register_listener(self)

        self.received_messages = []
        self.running = True
        self.logger.info(f"TestNode {self.node_id} initialized.")

    def broadcast(self, message: str):
        """
        Sends a message by encoding it to UTF-8 bytes.
        """
        self.serial_com.send_data(message.encode('utf-8'))
        self.logger.info(f"Node {self.node_id} sent message: {message}")

    def receive_packet(self, message_bytes: bytes):
        """
        Callback method invoked by SerialInterfaceEbyte when a packet is received.
        """
        # For testing, we'll assume the message is a UTF-8 string
        try:
            message = message_bytes.decode('utf-8')
            self.logger.info(f"Node {self.node_id} received message: {message}")
            self.received_messages.append(message)
        except UnicodeDecodeError as e:
            self.logger.error(f"Node {self.node_id} failed to decode message: {e}")

    def close(self):
        """
        Closes the SerialInterfaceEbyte and stops the node.
        """
        self.logger.info(f"Closing TestNode {self.node_id}.")
        self.running = False
        self.serial_com.close()


def main():
    logger = ConsoleLogger()
    parser = argparse.ArgumentParser(description="Serial Interface Test Script")
    SerialNetworkArgs.add_args(parser)

    # Parse command-line arguments
    parsed_args = parser.parse_args()

    # Create SerialNetworkArgs for Receiver Node
    args_receiver = SerialNetworkArgs(parsed_args)
    args_receiver.conn = RECEIVER_PORT
    args_receiver.baud = BAUD_RATE

    # Initialize Receiver Node
    receiver_node = TestNode(1, args_receiver, logger=logger)

    # Give the receiver time to initialize
    time.sleep(1)

    # Create SerialNetworkArgs for Transmitter Node
    args_transmitter = SerialNetworkArgs(parsed_args)
    args_transmitter.conn = TRANSMITTER_PORT
    args_transmitter.baud = BAUD_RATE

    # Initialize Transmitter Node
    transmitter_node = TestNode(2, args_transmitter, logger=logger)

    # Send a test message
    test_message = '{"mt": "CHECK_IN", "sid": 1.0, "rid": null}'
    transmitter_node.broadcast(test_message)

    # Allow some time for the message to be received
    time.sleep(2)

    # Check if the receiver received the message
    if test_message in receiver_node.received_messages:
        logger.info("Test passed: Receiver received the message.")
    else:
        logger.error("Test failed: Receiver did not receive the message.")

    # Clean up
    transmitter_node.close()
    receiver_node.close()


if __name__ == "__main__":
    main()
