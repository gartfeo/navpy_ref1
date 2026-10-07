from enum import Enum
from typing import Optional

from navpy.args.pid_args import PIDArgs
from navpy.args.vehicle_arg_ports import NavigationParameterReader
from navpy.utils.arg_helper import parse_boolean, StoreWithFlag
from navpy.logger.cache_logger import ILogger


class NavigationAlgorithm(str, Enum):
    PID = "pid"
    PN = "pn"
    VISION_NAV_PN = "vision-nav-pn"


NAVIGATION_ALGORITHM_BY_PARAM = {
    0: NavigationAlgorithm.PID,
    1: NavigationAlgorithm.PN,
    2: NavigationAlgorithm.VISION_NAV_PN,
}
NAVIGATION_ALGORITHM_PARAM_BY_VALUE = {
    algorithm.value: param
    for param, algorithm in NAVIGATION_ALGORITHM_BY_PARAM.items()
}
# Final approach defaults to the pure-vision law. PID/PN (0/1) are legacy
# geo laws: their Roll-L1 lateral command reads compass yaw/heading, NED
# ground velocity and the POI geo position (see AGENTS.md "Pure Vision
# Approach Constraint"), so they must be selected explicitly.
DEFAULT_NAVIGATION_ALGORITHM = NavigationAlgorithm.VISION_NAV_PN


def resolve_navigation_algorithm(value, logger: Optional[ILogger] = None) -> NavigationAlgorithm:
    """Resolve CLI/MAVLink final-approach controller selection to a known algorithm."""
    if isinstance(value, NavigationAlgorithm):
        return value
    if isinstance(value, str):
        try:
            return NavigationAlgorithm(value)
        except ValueError:
            try:
                return NAVIGATION_ALGORITHM_BY_PARAM[int(value)]
            except (TypeError, ValueError, KeyError):
                pass
    else:
        try:
            return NAVIGATION_ALGORITHM_BY_PARAM[int(value)]
        except (TypeError, ValueError, KeyError):
            pass

    if logger is not None:
        logger.warning(
            f"Navigation: invalid AAS_DEL_CTRL={value!r}; "
            f"using {DEFAULT_NAVIGATION_ALGORITHM.value}"
        )
    return DEFAULT_NAVIGATION_ALGORITHM


class NavigationArgs(object):
    # Single source of truth: param_name -> default
    PARAMS = {
        'AAS_DEL_PITCH': 0,
        'AAS_DEL_THR': -1,
        'AAS_DEL_DIR': False,
        'AAS_DEL_P_KP': 1.5,
        'AAS_DEL_CTRL': NAVIGATION_ALGORITHM_PARAM_BY_VALUE[DEFAULT_NAVIGATION_ALGORITHM.value],  # 0=PID (legacy geo), 1=PN (legacy geo), 2=vision-nav-pn
        'AAS_USE_TRN': True,
        'AAS_DEL_PLD': 100,
        'AAS_DEL_PLRD': 2.0,
    }

    def __init__(self, args, vehicle: NavigationParameterReader, logger: ILogger):
        self.use_direct_poi = None
        self.use_terrain = None
        self.final_approach_throttle = None
        self.final_approach_angle = None
        self.pitch_lock_dist = None  # Min distance to lock pitch while maneuvering
        self.pitch_lock_roll_diff = None  # Roll error threshold for pitch lock
        self.navigation_algorithm = None
        self.pitch_controller = None

        self._args = args
        self._vehicle = vehicle
        self._logger = logger

        self.pitch_args = PIDArgs(self._args, 'pitch',
                                  param_getter=self._get_param,
                                  param_kp_name='AAS_DEL_P_KP')

        self.refresh()

    def _get_param(self, param_name: str):
        """Get value with priority: explicit CLI arg > MAVLink param > default."""
        default = self.PARAMS[param_name]
        overrides = getattr(self._args, "_cli_overrides", set())
        if param_name in overrides:
            return getattr(self._args, param_name)
        # Otherwise try MAVLink param (1 attempt), fall back to default
        return self._vehicle.get_param_or_default(param_name, default)

    def refresh(self):
        old_final_approach_angle = self.final_approach_angle
        old_final_approach_throttle = self.final_approach_throttle

        self.use_direct_poi = bool(self._get_param('AAS_DEL_DIR'))
        desired_final_approach_angle = float(self._get_param('AAS_DEL_PITCH'))
        throttle = self._get_param('AAS_DEL_THR')
        self.final_approach_throttle = throttle if throttle is not None and throttle >= 0 else None
        self.use_terrain = bool(self._get_param('AAS_USE_TRN'))
        self.pitch_lock_dist = float(self._get_param('AAS_DEL_PLD'))
        self.pitch_lock_roll_diff = float(self._get_param('AAS_DEL_PLRD'))
        # Fixed airframe calibration (see module constant) — not a parameter.

        # Controller type: 0=PID, 1=PN, 2=vision-nav-pn (MAVLink param),
        # or the same string values from CLI.
        ctrl_val = self._get_param('AAS_DEL_CTRL')
        algorithm = resolve_navigation_algorithm(ctrl_val, self._logger)
        self.navigation_algorithm = algorithm.value
        self.pitch_controller = (
            "pid" if algorithm is NavigationAlgorithm.PID else "pn"
        )

        self.pitch_args.refresh()
        self._calc_final_approach_angle(desired_final_approach_angle)

        if old_final_approach_angle != self.final_approach_angle or old_final_approach_throttle != self.final_approach_throttle:
            self._logger.info(f'T: {self.final_approach_angle} deg, 'f'{self.final_approach_throttle}%')

    def _calc_final_approach_angle(self, desired_final_approach_angle: float):
        min_final_approach_angle = self._vehicle.min_pitch * 0.9

        if desired_final_approach_angle < min_final_approach_angle:
            if self.final_approach_angle is None or self.final_approach_angle != min_final_approach_angle:
                self._logger.warning(
                    f'Navigation: desired final approach angle: {desired_final_approach_angle} is too low, '
                    f'setting to {min_final_approach_angle} (min pitch -10%)'
                )
            self.final_approach_angle = min_final_approach_angle
        else:
            if self.final_approach_angle is not None and self.final_approach_angle != desired_final_approach_angle:
                self._logger.info(f'Navigation: setting final approach angle to {desired_final_approach_angle}')
            self.final_approach_angle = desired_final_approach_angle

    @classmethod
    def add_args(cls, parser):
        p = cls.PARAMS
        g = parser.add_argument_group("Navigation Navigation Arguments")
        g.add_argument("-da", dest='AAS_DEL_PITCH', type=int, action=StoreWithFlag,
                       default=p['AAS_DEL_PITCH'],
                       help=f"Approach pitch (clipped by vehicle pitch -10%%). Default: {p['AAS_DEL_PITCH']}")
        g.add_argument('-dt', dest='AAS_DEL_THR', type=int, action=StoreWithFlag,
                       default=p['AAS_DEL_THR'],
                       help=f"Throttle percentage during final-approach descent. Default: {p['AAS_DEL_THR']}")
        g.add_argument('-ut', dest='AAS_USE_TRN', type=parse_boolean, action=StoreWithFlag,
                       default=p['AAS_USE_TRN'],
                       help=f"Enable Terrain. Default: {p['AAS_USE_TRN']}")
        g.add_argument("-udt", dest='AAS_DEL_DIR', type=parse_boolean, action=StoreWithFlag,
                       default=p['AAS_DEL_DIR'],
                       help=f"Use simulator reference position (legacy navigation). Default: {p['AAS_DEL_DIR']}")
        g.add_argument("-pld", dest='AAS_DEL_PLD', type=float, action=StoreWithFlag,
                       default=p['AAS_DEL_PLD'],
                       help=f"Min distance (m) to lock pitch while maneuvering. -1 to disable. Default: {p['AAS_DEL_PLD']}")
        g.add_argument("-plrd", dest='AAS_DEL_PLRD', type=float, action=StoreWithFlag,
                       default=p['AAS_DEL_PLRD'],
                       help=f"Roll error (deg) threshold for pitch lock. -1 to disable. Default: {p['AAS_DEL_PLRD']}")

        choices = tuple(algorithm.value for algorithm in NavigationAlgorithm) + tuple(
            str(value) for value in NAVIGATION_ALGORITHM_BY_PARAM
        )
        g.add_argument("--pitch-controller", "--navigation-algorithm",
                       dest='AAS_DEL_CTRL', choices=choices,
                       action=StoreWithFlag, default=DEFAULT_NAVIGATION_ALGORITHM.value,
                       help=(
                           "Final approach algorithm "
                           "(0=PID legacy geo, 1=PN legacy geo, 2=vision-nav-pn). "
                           f"Default: {DEFAULT_NAVIGATION_ALGORITHM.value}"
                       ))
