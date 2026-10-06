from enum import Enum

from navpy.args.conn.serial_multi_networkArgs import SerialMultiNetworkArgs
from navpy.args.conn.serial_network_args import SerialNetworkArgs
from navpy.args.conn.wifi_network_args import WifiNetworkArgs


class NetworkType(Enum):
    NONE = 'none'
    WIFI = 'wifi'
    SERIAL = 'serial'
    SERIAL_MULTI = 'serial_multi'
    MAV = 'mav'
    MAV_MULTI = 'mav_multi'


class NetworkArgs(object):
    def __init__(self, args):
        self.network_type = args.network_type
        self.disable_ttl = getattr(args, "disable_ttl", False)
        self.disable_dedup = getattr(args, "disable_dedup", False)
        self.serial_args = SerialNetworkArgs(args)
        self.serial_multi_args = SerialMultiNetworkArgs(args)
        self.wifi_args = WifiNetworkArgs(args)

    @staticmethod
    def add_args(parser):
        args = parser.add_argument_group("Mesh arguments")

        args.add_argument('-nt', '--network-type', type=NetworkType, default=NetworkType.NONE,
                          help=f'Network Type: Default none, options: {[e.value for e in NetworkType]}')
        args.add_argument('--disable-ttl', action='store_true',
                          help='Disable TTL checks for incoming messages. Default: False')
        args.add_argument('--disable-dedup', action='store_true',
                          help='Disable message deduplication. Default: False')

        SerialNetworkArgs.add_args(parser)
        WifiNetworkArgs.add_args(parser)
        SerialMultiNetworkArgs.add_args(parser)
