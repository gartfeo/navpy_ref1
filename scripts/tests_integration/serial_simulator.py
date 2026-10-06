import time
from multiprocessing import Process

from navpy.modules.comm.messages.check_msg import CheckInMsg
from navpy.modules.comm.serial.sim.serial_sim_server import SerialSimServer


def start_node(n_id):
    from navpy.logger.cache_logger import ConsoleLogger
    from navpy.modules.comm.network_serial import NetworkSerial

    logger = ConsoleLogger()
    network = NetworkSerial(n_id, None, logger=logger, simulation=True)

    class MessageListener:
        def on_message(self, message):
            print(f"Node {n_id} received message: {message}")

    network.listener = MessageListener()

    # Send a message after a short delay
    time.sleep(2)
    network.broadcast(CheckInMsg(n_id))

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        network.close()


if __name__ == "__main__":
    server_process = Process(target=SerialSimServer.start_server)
    server_process.start()
    time.sleep(1)  # Give the server time to start

    node_processes = []
    for node_id in range(1, 4):  # Start 3 nodes
        p = Process(target=start_node, args=(node_id,))
        p.start()
        node_processes.append(p)

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        for p in node_processes:
            p.terminate()
        server_process.terminate()
