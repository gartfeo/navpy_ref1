from typing import Optional

from navpy.app.network_args import NetworkArgs, NetworkType
from navpy.core.ports.network_abc import NetworkAbc
from navpy.core.link.message_filter import MessageFilter
from navpy.plugins.radio.serial.network_serial import NetworkSerial
from navpy.plugins.radio.wifi.network_wifi import NetworkWifi
from navpy.plugins.radio.mavlink.network_mavlink import NetworkMavlink
from navpy.core.kernel.logging.cache_logger import ILogger
from navpy.core.ports.vehicle_contract import IVehicle


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
