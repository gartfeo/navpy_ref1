from navpy.utils.arg_helper import parse_none_boolean


class MissionPlannerArgs:
    def __init__(self, args):
        self.mp_enable = args.mp_enable
        self.wind_dir = args.mp_wind_dir
        self.min_air_speed = args.mp_min_air_speed
        self.alt_loc_same = args.mp_alt_loc_same
        self.aligned_delta = args.mp_aligned_delta
        self.loiter_coefficient = args.mp_loiter_coefficient
        self.custom_start_distance = args.mp_custom_start_distance
        self.plan_loiter = not args.mp_plan_dir
        self.dir_distance = args.mp_dir_distance
        self.plan_loiter_wind_direction = args.mp_plan_loiter_wind_direction
        self.custom_bearings = args.mp_custom_bearings

    @staticmethod
    def add_args(parser):
        mp_args = parser.add_argument_group("Mission Planner arguments")

        mp_args.add_argument('-mpenb', '--mp-enable', type=parse_none_boolean, default=False,
                             help='Enable Mission Planner. Default: False')

        mp_args.add_argument('-mpwd', '--mp-wind-dir', type=parse_none_boolean, default=False,
                             help='Plan aligned wind direction. '
                                  'If None will do the closest, If False Opposite direction. Default: True')
        mp_args.add_argument('-mpcsd', '--mp-custom-start-distance', type=int, default=700,
                             help='Start custom distance from home location in meters. '
                                  'If None predicted distance from current location to the delivery reference. Default: None')
        mp_args.add_argument('-mpmspd', '--mp-min-air-speed', type=float, default=5.0,
                             help='Minimum air speed. Default: 5.0')
        mp_args.add_argument('-mpad', '--mp-aligned-delta', type=float, default=3,
                             help='Aligned delta in degrees. Default: 3')
        mp_args.add_argument('-mpals', '--mp-alt-loc-same', type=parse_none_boolean, default=True,
                             help='Use the same location for altitude and loiter. Default: True')
        mp_args.add_argument('-mpplwd', '--mp-plan-loiter-wind-direction', type=parse_none_boolean, default=True,
                             help='location aligned with wind when --mp_plan_dir false. Default: True')

        mp_args.add_argument('-mppd', '--mp-plan-dir', type=parse_none_boolean, default=False,
                             help='Plan direction. Default: True')
        mp_args.add_argument('-mplcf', '--mp-loiter-coefficient', type=float, default=1.8,
                             help='Loiter coefficient. Default: 1.8')
        mp_args.add_argument('-mpdd', '--mp-dir-distance', type=float, default=250,
                             help='Direction distance. Default: 150')
        mp_args.add_argument('-mpcb', '--mp-custom-bearings', type=float, nargs=2, default=None,
                             help='Custom bearings for wind planning. Default: None')
