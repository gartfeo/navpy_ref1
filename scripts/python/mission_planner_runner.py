import argparse
import sys

from navpy.args.logger_args import LoggerArgs
from navpy.modules.vehicle.vehicle_factory import create_vehicle
from navpy.modules.navigation.mission_planner import MissionPlanner, MissionPlannerArgs
from navpy.logger.cache_logger import ConsoleCacheLogger
from navpy.main import ConnArgs


def main(args):
    vehicle = create_vehicle(args, skip_mission_download=True)
    mp_args = MissionPlannerArgs(args)
    mp_args.mp_enable = True
    wind_vel_cal = MissionPlanner(ConsoleCacheLogger(), mp_args)
    wind_vel_cal.adjust_nav_bearing(vehicle, mp_args.custom_bearings)


if __name__ == '__main__':
    baud = 115200
    if sys.platform == 'linux':
        default_conn = '/dev/ttyACM0'
        default_c_conn = ''
        default_c_conn2 = ''
        # default_c_conn = '/dev/ttyUSB0'
        # default_c_conn2 = '/dev/ttyUSB1'
    else:
        default_conn = f'tcp:127.0.0.1:5763'
        default_c_conn = ''
        default_c_conn2 = ''
        # default_conn = f'COM16'
        # default_c_conn = 'COM11'
        # default_c_conn2 = 'COM14'
    parser = argparse.ArgumentParser()
    # Vehicle conn
    ConnArgs.add_args(parser)

    LoggerArgs.add_args(parser)

    MissionPlannerArgs.add_args(parser)

    parsed_args = parser.parse_args()

    main(parsed_args)
