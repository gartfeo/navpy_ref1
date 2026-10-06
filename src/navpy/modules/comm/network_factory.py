from typing import Optional

from navpy.args.conn.network_args import NetworkArgs, NetworkType
from navpy.modules.comm.network_abc import NetworkAbc
from navpy.modules.comm.message_filter import MessageFilter
from navpy.modules.comm.network_serial import NetworkSerial
from navpy.modules.comm.network_wifi import NetworkWifi
from navpy.modules.comm.network_mavlink import NetworkMavlink
from navpy.logger.cache_logger import ILogger
from navpy.modules.vehicle.vehicle_interface import IVehicle


def create_network(args: NetworkArgs, node_id: int, logger: ILogger, vehicle: Optional[IVehicle]) -> Optional[NetworkAbc]:
    if args.network_type is None or args.network_type == NetworkType.NONE:
        return None

    message_filter = MessageFilter(
        logger=logger,
        enable_ttl=not args.disable_ttl,
        enable_dedup=not args.disable_dedup,
    )

    if args.network_type == NetworkType.WIFI:
        return NetworkWifi(node_id, args.wifi_args, logger, message_filter=message_filter)
    elif args.network_type == NetworkType.SERIAL or args.network_type == NetworkType.SERIAL_MULTI:
        return NetworkSerial(node_id, args.serial_args, logger, message_filter=message_filter)
    elif args.network_type == NetworkType.MAV or args.network_type == NetworkType.MAV_MULTI:
        return NetworkMavlink(node_id, vehicle, logger, message_filter=message_filter)
    else:
        return None
