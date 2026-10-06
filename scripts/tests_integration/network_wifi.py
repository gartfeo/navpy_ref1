import argparse
import sys
import threading
import time
from unittest.mock import Mock

from navpy.modules.common.models.location import Location
from navpy.modules.comm.network_wifi import NetworkWifi
from navpy.modules.vehicle.vehicle_interface import IVehicle
from navpy.modules.swarm.task_actor import TaskActor
from navpy.utils.arg_helper import parse_boolean
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
    network1 = None
    network2 = None
    actor2 = None
    try:
        print(f'Creating mesh on: {args.self_port} ({args.peers})')
        network1 = NetworkWifi(port=args.self_port or 14550,
                               listening_addresses=args.peers or ['tcp://localhost:14551'])
        vehicle1 = Mock(spec=IVehicle)
        vehicle1.sys_id = 1
        actor1 = TaskActor(vehicle1, network1, ConsoleCacheLogger())
        actor1.vehicle.location.return_value = Location(1, 1, 1)
        network1.set_listener(actor1)

        if args.simulation:
            network2 = NetworkWifi(port=14551, listening_addresses=['tcp://localhost:14550'])
            vehicle2 = Mock(spec=IVehicle)
            vehicle2.sys_id = 2
            actor2 = TaskActor(vehicle2, network2, ConsoleCacheLogger())
            network2.set_listener(actor2)

        time.sleep(2)

        actor1.checkin()
        time.sleep(1)
        if actor2 is not None:
            actor2.checkin()

        # for i in range(args.retry_count):
        #     actor1.notify_targets([
        #         DetectedObject(2, 1, 1, 0, None, None, None, LocationAbc(1, 1, 1), LocationAbc(1, 1, 1)),
        #         DetectedObject(3, 1, 1, 0, None, None, None, LocationAbc(2, 2, 2), LocationAbc(2, 2, 2)),
        #     ])
        #     time.sleep(5)

        time.sleep(5)

        actor1.checkout()

        print("Done")
    except Exception as e:
        print(f"Error: {e}, {sys.exc_info()}")
    finally:
        # Handle cleanup if necessary
        print("\nExiting program...")
        if network1 is not None:
            network1.close()
        if network2 is not None:
            network2.close()
        sys.exit(0)


if __name__ == '__main__':
    # Start the event loop

    parser = argparse.ArgumentParser()
    parser.add_argument('-rc', '--retry-count', type=int, default=3,
                        help=f'Retry Count: Default 3')
    parser.add_argument('-sp', '--self-port', type=int, default=14550,
                        help=f'Self Port: Default 14550')

    parser.add_argument('-ps', '--peers', type=str, nargs='+',
                        default=['tcp://192.168.0.111:14551', 'tcp://192.168.0.111:14552'],
                        help=f'Peers Addresses: Default [14551, 14552]')
    parser.add_argument('-sim', '--simulation', type=parse_boolean, default=True,
                        help=f'Simulation: Default False')
    parsed_args = parser.parse_args()
    main(parsed_args)
