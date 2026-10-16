from __future__ import annotations

from pymavlink.dialects.v20.ardupilotmega import MAV_TYPE_ONBOARD_CONTROLLER, MAV_COMP_ID_ONBOARD_COMPUTER

from navpy.app.conn_args import ConnArgs
from navpy.core.ports.vehicle_contract import IVehicle
from navpy.plugins.vehicle.ardupilot.mav_bus import MavBus, MavBusOpener
from navpy.plugins.vehicle.ardupilot.vehicle_mav import VehicleMav
from navpy.core.kernel.logging.cache_logger import ILogger, ConsoleLogger


def open_vehicle_bus(conn_args: ConnArgs,
                     open_bus: MavBusOpener = MavBus.get_or_create,
                     component_id: int = MAV_COMP_ID_ONBOARD_COMPUTER) -> MavBus:
    """Open the device's bus: a lockstep link's ``open`` under lockstep."""
    print(f'Connecting to vehicle on: {conn_args.connection} ({conn_args.baud})')
    return open_bus(
        conn_args.connection, conn_args.baud, conn_args.source_system, component_id,
    )


def create_vehicle(conn_args: ConnArgs,
                 logger: ILogger | None = ConsoleLogger(),
                 skip_mission_download=False,
                 wait_for_heartbeat=True,
                 interface_type='vehicle-mav',
                 send_heartbeat=True,
                 component_id=MAV_COMP_ID_ONBOARD_COMPUTER,
                 mav_type=MAV_TYPE_ONBOARD_CONTROLLER,
                 open_bus: MavBusOpener = MavBus.get_or_create,
                 bus: MavBus | None = None) -> IVehicle:
    """``bus``, when the caller opened it, else opened with ``open_bus``."""
    if interface_type == 'vehicle-mav':
        if bus is None:
            bus = open_vehicle_bus(conn_args, open_bus, component_id)
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
