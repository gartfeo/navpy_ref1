from __future__ import annotations

from pymavlink.dialects.v20.ardupilotmega import MAV_TYPE_ONBOARD_CONTROLLER, MAV_COMP_ID_ONBOARD_COMPUTER

from navpy.app.conn_args import ConnArgs
from navpy.core.ports.vehicle_contract import IVehicle
from navpy.plugins.vehicle.ardupilot.mav_bus import MavBus, MavBusOpener
from navpy.plugins.vehicle.ardupilot.vehicle_mav import VehicleMav
from navpy.core.kernel.logging.cache_logger import ILogger, ConsoleLogger


def create_vehicle(conn_args: ConnArgs,
                 logger: ILogger | None = ConsoleLogger(),
                 skip_mission_download=False,
                 wait_for_heartbeat=True,
                 interface_type='vehicle-mav',
                 send_heartbeat=True,
                 component_id=MAV_COMP_ID_ONBOARD_COMPUTER,
                 mav_type=MAV_TYPE_ONBOARD_CONTROLLER,
                 open_bus: MavBusOpener = MavBus.get_or_create) -> IVehicle:
    """``open_bus`` opens the device's bus: a lockstep link's ``open`` under lockstep."""
    print(f'Connecting to vehicle on: {conn_args.connection} ({conn_args.baud})')
    if interface_type == 'vehicle-mav':
        bus = open_bus(
            conn_args.connection, conn_args.baud, conn_args.source_system, component_id,
        )
        return VehicleMav(conn_args.connection,
                          conn_args.source_system,
                          conn_args.baud,
                          logger,
                          skip_mission_download,
                          wait_for_heartbeat,
                          send_heartbeat,
                          mav_type=mav_type,
                          mav_comp_id=component_id,
                          bus=bus)
    else:
        raise ValueError('Unknown interface type: {}'.format(interface_type))
