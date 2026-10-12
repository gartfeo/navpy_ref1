from argparse import ArgumentParser
from typing import Callable, Sequence

from navpy.args.conn_args import ConnArgs
from navpy.args.conn.network_args import NetworkArgs
from navpy.core.kernel.config.pid_args import PIDArgs
from navpy.core.kernel.config.navigation_args import NavigationArgs
from navpy.sim.world.navigation_poi_args import NavigationPoiArgs
from navpy.core.kernel.config.logger_args import LoggerArgs
from navpy.core.kernel.config.mission_planner_args import MissionPlannerArgs
from navpy.core.kernel.config.nav_args import NavArgs
from navpy.args.vision_args import VisionArgs

# every (*class*, extra-positional-args)
_ARG_MODULES: Sequence[tuple[Callable, tuple]] = (
    # vehicle
    (ConnArgs.add_args, ()),
    # simulator POIs / navigation
    (NavigationPoiArgs.add_args, ()),
    (NavigationArgs.add_args, ()),
    # navigation
    (NavArgs.add_args, ()),
    # vision (profile-based)
    (VisionArgs.add_args, ()),
    # controllers
    (PIDArgs.add_args, ('pitch',)),
    # mesh/network
    (NetworkArgs.add_args, ()),
    # mission planner
    (MissionPlannerArgs.add_args, ()),
    # logging
    (LoggerArgs.add_args, ()),
)


def make_parser(*args, **kwargs) -> ArgumentParser:
    """Return a fully populated navpy ArgumentParser."""
    parser = ArgumentParser(*args, **kwargs)
    for fn, extra in _ARG_MODULES:
        fn(parser, *extra)
    return parser
