import sys


class SerialNetworkArgs(object):
    def __init__(self, args):
        self.conn = args.serial_conn
        self.baud = args.serial_baud

    @staticmethod
    def add_args(parser):
        args = parser.add_argument_group("Serial arguments")
        if sys.platform == 'linux':
            default_conn = '/dev/ttyACM0'
        else:
            default_conn = f'COM24'

        args.add_argument('-sc', '--serial-conn', type=str, default=default_conn,
                          help=f'Self Port: Default {default_conn}')
        args.add_argument('-sb', '--serial-baud', type=int, default=115200,
                          help=f'Baud Rate: Default 115200')
