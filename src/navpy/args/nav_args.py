from navpy.args.navigation_poi_args import nav_wp_indices
from navpy.args.vehicle_arg_ports import MissionParameterReader
from navpy.utils.arg_helper import parse_boolean, StoreWithFlag, StoreTrueWithFlag


class NavArgs(object):
    # Single source of truth: param_name -> default
    PARAMS = {
        'AAS_NAV_LAST_WP': 4,
        'AAS_NAV_MIN_ALT': 130,
        'AAS_NAV_CWT': 30,
        'AAS_NAV_AUTO_CM': True,
        # MISS-04: REJECT on confirm-window expiry — never auto-commit a
        # delivery destination the operator never approved. Explicit -tcfc true (or the
        # AAS_NAV_CM_FL MAVLink param) re-enables confirm-on-fail.
        'AAS_NAV_CM_FL': False,
        'AAS_NAV_ONESHOT': False,
        'AAS_NAV_CGT': 15,
    }
    # Nav sim speedup (0 = disabled). Disabled by default: NavPy no longer
    # overrides SIM_SPEEDUP during NAV, so the dive runs at whatever speed
    # the SITL launch set (no slow-to-1x). Pass -gsu <N> to re-enable.
    NAV_SIM_SPEEDUP = 0.0

    def __init__(self, args, vehicle: MissionParameterReader):
        self.vehicle = vehicle
        self._args = args

        self.min_wp = None
        self.min_alt = None
        self.confirm_wait_time_sec = None
        self.confirm_gate_timeout_sec = None
        self.is_auto_confirm = None
        self.is_confirm_on_fail = None
        self.is_oneshot = None

        # Nav sim speedup (0 = disabled)
        self.nav_sim_speedup = getattr(args, 'nav_sim_speedup', self.NAV_SIM_SPEEDUP)

        self.refresh()

    def _get_param(self, param_name: str):
        """Get value with priority: explicit CLI arg > MAVLink param > default."""
        default = self.PARAMS[param_name]
        overrides = getattr(self._args, "_cli_overrides", set())
        if param_name in overrides:
            return getattr(self._args, param_name)
        return self.vehicle.get_param_or_default(param_name, default)

    @classmethod
    def add_args(cls, parser):
        p = cls.PARAMS
        g = parser.add_argument_group("Navigation arguments")
        g.add_argument("-mwp", dest='AAS_NAV_LAST_WP', type=int, action=StoreWithFlag,
                       default=p['AAS_NAV_LAST_WP'],
                       help=f"Min waypoint index to start detection. Default: {p['AAS_NAV_LAST_WP']}")
        g.add_argument("-ma", dest='AAS_NAV_MIN_ALT', type=int, action=StoreWithFlag,
                       default=p['AAS_NAV_MIN_ALT'],
                       help=f"Min altitude to start navigation. Default: {p['AAS_NAV_MIN_ALT']}")
        g.add_argument("-tct", dest='AAS_NAV_CWT', type=int, action=StoreWithFlag,
                       default=p['AAS_NAV_CWT'],
                       help=f"Confirm wait time (sec). Default: {p['AAS_NAV_CWT']}")
        g.add_argument("-tac", dest='AAS_NAV_AUTO_CM', action=StoreTrueWithFlag,
                       default=p['AAS_NAV_AUTO_CM'],
                       help=f"Automatically confirm detected delivery destinations. Default: {p['AAS_NAV_AUTO_CM']}")
        g.add_argument("-tcfc", dest='AAS_NAV_CM_FL', type=parse_boolean, action=StoreWithFlag,
                       default=p['AAS_NAV_CM_FL'],
                       help=f"Confirm on fail. Default: {p['AAS_NAV_CM_FL']}")
        g.add_argument("-nos", dest='AAS_NAV_ONESHOT', action=StoreTrueWithFlag,
                       default=p['AAS_NAV_ONESHOT'],
                       help="Single-run mode: stop starting new delivery attempts after navigation completion; disarm only a simulated autopilot.")
        g.add_argument("-cgt", dest='AAS_NAV_CGT', type=int, action=StoreWithFlag,
                       default=p['AAS_NAV_CGT'],
                       help=f"CONFIRM zoom-stability timeout (sec). Recognition-size pixels are still required;"
                            f" after this, a sufficiently detailed frame may be sent without waiting for stable zoom. Default: {p['AAS_NAV_CGT']}")
        g.add_argument("-gsu", dest='nav_sim_speedup', type=float, default=cls.NAV_SIM_SPEEDUP,
                       help=f"Sim speedup during NAV (0=disabled). Default: {cls.NAV_SIM_SPEEDUP}")

    def refresh(self):
        """Reset the navigation arguments to their default values."""
        wp_number = int(self._get_param('AAS_NAV_LAST_WP'))
        nav_indices = nav_wp_indices(self.vehicle)
        if nav_indices and 1 <= wp_number <= len(nav_indices):
            self.min_wp = nav_indices[wp_number - 1]
        else:
            self.min_wp = max(min(wp_number, self.vehicle.mission_items_count - 1), 0)
        self.min_alt = float(self._get_param('AAS_NAV_MIN_ALT'))
        self.confirm_wait_time_sec = float(self._get_param('AAS_NAV_CWT'))
        self.confirm_gate_timeout_sec = float(self._get_param('AAS_NAV_CGT'))
        self.is_auto_confirm = self._get_param('AAS_NAV_AUTO_CM')
        self.is_confirm_on_fail = self._get_param('AAS_NAV_CM_FL')
        self.is_oneshot = bool(self._get_param('AAS_NAV_ONESHOT'))
