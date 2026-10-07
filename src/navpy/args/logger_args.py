import argparse
from enum import Enum

from navpy.logger.cache_log_level import CacheLogLevel

_LOG_LEVEL_NAMES = ", ".join(level.name for level in CacheLogLevel)


def _log_level(value):
    """argparse ``type`` for the log-level flags.

    Accepts a case-insensitive level NAME and maps it to a ``CacheLogLevel``.
    Re-raises the enum's ``ValueError`` as ``ArgumentTypeError`` so argparse
    reports the valid names verbatim instead of its generic message.
    """
    try:
        return CacheLogLevel.from_name(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc


class LogStatusDest(Enum):
    NETWORK = 'Network'
    DRONE = 'Vehicle'


class LoggerArgs:
    def __init__(self, args):
        self.vehicle = None
        self.status_update_interval = 1.0 / args.log_status_interval
        self.status_dest = [LogStatusDest(dest) for dest in args.log_status_dest]
        self.log_level = args.log_level
        self.status_level = args.log_status_level
        self.defer_status_logs = False

    @staticmethod
    def add_args(parser):
        parser.add_argument("-lsi", "--log-status-interval", type=int, default=2,
                            help="Status update interval in Hz. Default: 2 hz")
        parser.add_argument("-ll", "--log-level", type=_log_level, metavar="LEVEL",
                            default=CacheLogLevel.INFO,
                            help=f"Log level (one of: {_LOG_LEVEL_NAMES}). "
                                 f"Default: {CacheLogLevel.INFO.name}.")
        log_status_dest_default = [LogStatusDest.DRONE.value, LogStatusDest.NETWORK.value]
        parser.add_argument("-lsd", "--log-status-dest", type=LogStatusDest, choices=list(LogStatusDest),
                            nargs="+", default=log_status_dest_default,
                            help=f"Log Status Destination. Default: {log_status_dest_default}.")
        parser.add_argument("-lsl", "--log-status-level", type=_log_level, metavar="LEVEL",
                            default=CacheLogLevel.INFO,
                            help=f"Log status level (one of: {_LOG_LEVEL_NAMES}). "
                                 f"Default: {CacheLogLevel.INFO.name}.")

    def set_vehicle(self, vehicle):
        self.vehicle = vehicle
        self.refresh()

    def refresh(self):
        if self.vehicle is None:
            return

        status_rate = self.vehicle.get_param_or_default("AAS_LOG_RATE", 2)
        if status_rate > 0:
            self.status_update_interval = 1.0 / status_rate
        self.defer_status_logs = self.vehicle.get_param_or_default("AAS_LOG_DEFER", 0)


class LoggerArgsStub(LoggerArgs):
    def __init__(self):
        network_params = {
            'log_status_interval': 0.2,
            'log_status_dest': [LogStatusDest.NETWORK.value],
            'log_level': CacheLogLevel.INFO,
            'log_status_level': CacheLogLevel.INFO
        }
        super().__init__(argparse.Namespace(**network_params))
