import sys


class ConnArgs(object):
    def __init__(self, args):
        self.connection = args.connection
        self.baud = args.baud
        # The companion shares its aircraft's MAVLink system id; it is told
        # apart on the wire by component id (autopilot 1, companion
        # MAV_COMP_ID_ONBOARD_COMPUTER 191), so `-ss` is the single identity.
        self.source_system = args.source_system

    @staticmethod
    def add_args(conn_args_parser):
        baud = 115200
        if sys.platform == 'linux':
            default_conn = '/dev/ttyACM0'
        else:
            # default_conn = f'tcp:192.168.59.129:5762'
            default_conn = f'tcp:127.0.0.1:5762'
        args = conn_args_parser.add_argument_group("Vehicle connection arguments")
        args.add_argument("-c", "--connection", type=str, default=default_conn,
                          help=f"The connection to vehicle. Default: {default_conn}")
        args.add_argument("-b", "--baud", type=int, default=baud,
                          help=f"The baud rate of the vehicle. Default: {baud}")
        args.add_argument("-ss", "--source-system", type=int, required=True,
                          help=f"The source system of the vehicle.")
