from __future__ import annotations

from pymavlink.dialects.v20.ardupilotmega import MAV_TYPE_ONBOARD_CONTROLLER, MAV_COMP_ID_ONBOARD_COMPUTER

from navpy.args.conn_args import ConnArgs
from navpy.modules.vehicle.vehicle_interface import IVehicle
from navpy.modules.vehicle.vehicle_mav import VehicleMav
from navpy.logger.cache_logger import ILogger, ConsoleLogger


def create_vehicle(conn_args: ConnArgs,
                 logger: ILogger | None = ConsoleLogger(),
                 skip_mission_download=False,
                 wait_for_heartbeat=True,
                 interface_type='vehicle-mav',
                 send_heartbeat=True,
                 component_id=MAV_COMP_ID_ONBOARD_COMPUTER,
                 mav_type=MAV_TYPE_ONBOARD_CONTROLLER) -> IVehicle:
    print(f'Connecting to vehicle on: {conn_args.connection} ({conn_args.baud})')
    if interface_type == 'vehicle-mav':
        return VehicleMav(conn_args.connection,
                          conn_args.source_system,
                          conn_args.baud,
                          logger,
                          skip_mission_download,
                          wait_for_heartbeat,
                          send_heartbeat,
                          mav_type=mav_type,
                          mav_comp_id=component_id)
    else:
        raise ValueError('Unknown interface type: {}'.format(interface_type))
