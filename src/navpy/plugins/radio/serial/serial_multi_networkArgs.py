class SerialMultiNetworkArgs(object):
    def __init__(self, args):
        self.serial_conn_multi = args.serial_conn_multi

    @staticmethod
    def add_args(parser):
        args = parser.add_argument_group("Serial MultiLink arguments")

        args.add_argument("-scm", "--serial-conn-multi", type=str, nargs='+',
                          default=['1-socket://127.0.0.1:5761', '2-socket://127.0.0.1:5771'],
                          help=f"The serial multi connections. Default: ['1-socket://127.0.0.1:5761', '2-socket://127.0.0.1:5771']")
