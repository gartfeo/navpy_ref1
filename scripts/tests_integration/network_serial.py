import argparse
import sys
import threading
import time
from unittest.mock import Mock

from navpy.logger.cache_log_level import CacheLogLevel
from navpy.modules.common.models.location import Location
from navpy.modules.comm.network_serial import NetworkSerial
from navpy.modules.vision.models.detect_data import DetectedObject
from navpy.modules.vehicle.vehicle_interface import IVehicle
from navpy.modules.swarm.task_actor import TaskActor
from navpy.logger.cache_logger import ConsoleCacheLogger

print_lock = threading.Lock()


def thread_safe_print(*args, **kwargs):
    with print_lock:
        print(*args, **kwargs)


class Listener:
    def __init__(self, listener_id):
        self.id = listener_id

    def on_message(self, message):
        thread_safe_print(f"Listener {self.id} Received: {message}")


def main(args):
    node1 = None
    node2 = None
    try:
        logger = ConsoleCacheLogger(CacheLogLevel.INFO)
        node1_port = 'COM23'
        node2_port = 'COM26'
        logger.info(f'Creating mesh on: {node1_port} and {node2_port}')
        node1 = NetworkSerial(1, node1_port, 57600, logger)
        node2 = NetworkSerial(2, node2_port, 57600, logger)

        vehicle1_mock = Mock(spec=IVehicle)
        vehicle1_mock.location.return_value = Location(1, 1, 1)
        vehicle1_mock.sys_id = 1
        actor1 = TaskActor(vehicle1_mock, node1, ConsoleCacheLogger())
        node1.set_listener(actor1)

        vehicle2_mock = Mock(spec=IVehicle)
        vehicle2_mock.sys_id = 2
        vehicle2_mock.location.return_value = Location(2, 2, 2)
        vehicle2_mock.ground_speed = 25
        vehicle2_mock.wind.speed = 5
        vehicle2_mock.wind.direction = 180
        vehicle2_mock.battery_level.return_value = 80
        actor2 = TaskActor(vehicle2_mock, node2, logger)
        node2.set_listener(actor2)

        time.sleep(2)

        actor1.checkin()

        time.sleep(1)

        actor2.checkin()

        actor1.notify_targets([
            DetectedObject(3, 1, 1, 0, None, None, None, Location(2, 2, 2), Location(2.001, 2.002, 0)),
        ])

        time.sleep(60)
        actor2.checkout()

        print("Done")
    except Exception as e:
        print("Error occurred:")
        print(f"Exception: {e}")
    finally:
        # Handle cleanup if necessary
        print("\nExiting program...")
        if node1 is not None:
            node1.close()
        if node2 is not None:
            node2.close()
        sys.exit(0)


if __name__ == '__main__':
    # Start the event loop

    parser = argparse.ArgumentParser()
    parser.add_argument('-sp', '--self-port', type=str, default='COM23',
                        help=f'Self Port: Default COM23')
    parsed_args = parser.parse_args()
    main(parsed_args)
