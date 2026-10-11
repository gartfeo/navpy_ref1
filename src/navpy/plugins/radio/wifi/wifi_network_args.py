class WifiNetworkArgs:
    def __init__(self, args):
        self.self_port = args.wifi_self_port
        self.peers = args.wifi_peers_ports

    @staticmethod
    def add_args(conn_args_parser):
        args = conn_args_parser.add_argument_group("wifi connection arguments")

        args.add_argument("-wsp", "--wifi-self-port", type=str, default=14550,
                          help=f"The connection to vehicle. Default: 14550")
        args.add_argument("-wpp", "--wifi-peers-ports", type=str, nargs='+',
                          default=['tcp://localhost:14551', 'tcp://localhost:14552'],
                          help=f"The source system of the vehicle. Default: [tcp://localhost:14551', 'tcp://localhost:14552]")
