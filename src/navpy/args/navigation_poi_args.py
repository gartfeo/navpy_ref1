from pymavlink.dialects.v20.ardupilotmega import MAV_CMD_NAV_WAYPOINT

from navpy.args.vehicle_arg_ports import MissionParameterReader


def encode_wp_bitmask(wp_numbers: list[int]) -> int:
    """Encode 1-based waypoint numbers as a bitmask (bit i-1 set = WP i)."""
    return sum(1 << (i - 1) for i in wp_numbers if 1 <= i <= 24)


def decode_wp_bitmask(bitmask: int) -> list[int]:
    """Decode a bitmask to a sorted list of 1-based waypoint numbers."""
    return [i + 1 for i in range(24) if bitmask & (1 << i)]


NAV_POI_COMMANDS = frozenset({MAV_CMD_NAV_WAYPOINT})


def is_nav_poi_command(command: object) -> bool:
    return type(command) is int and command in NAV_POI_COMMANDS


def nav_wp_indices(vehicle: MissionParameterReader) -> list[int]:
    """Return mission item indices for all NAV_WAYPOINT items (skipping HOME at seq 0)."""
    indices = []
    for seq in range(1, vehicle.mission_items_count):
        wp = vehicle.get_mission_item(seq)
        if wp is not None and is_nav_poi_command(getattr(wp, 'command', None)):
            indices.append(seq)
    return indices


class NavigationPoiArgs:
    """Simulator POI settings; retain AAS_TARG_* wire names."""

    # Single source of truth: param_name -> default
    PARAMS = {
        'AAS_TARG_WPS': 16,  # WP bitmask (16 = WP 5); CLI -twps takes a WP list
        'AAS_TARG_ALT': 150,
    }

    def __init__(self, args, vehicle: MissionParameterReader):
        self.poi_wp_indices: dict[int, int] = {}  # {wp_number: mission_item_index}
        self.poi_count = None
        self.poi_alt = None

        self._args = args
        self.vehicle = vehicle

        self.refresh()

    def _get_param(self, param_name: str):
        """Get value with priority: explicit CLI arg > MAVLink param > default."""
        default = self.PARAMS[param_name]
        cli_value = getattr(self._args, param_name)
        if cli_value != default:
            return cli_value
        return self.vehicle.get_param_or_default(param_name, default)

    def refresh(self):
        raw = self._get_param('AAS_TARG_WPS')
        if isinstance(raw, str):
            wp_numbers = [int(float(s.strip())) for s in raw.split(',') if s.strip()]
        else:
            wp_numbers = decode_wp_bitmask(int(raw))

        # Build 1-based WP number → mission-item-index mapping
        nav_indices = nav_wp_indices(self.vehicle)

        # Resolve 1-based wp_numbers to actual mission item indices
        self.poi_wp_indices = {n: nav_indices[n - 1] for n in wp_numbers
                                  if 1 <= n <= len(nav_indices)}
        self.poi_count = len(self.poi_wp_indices)
        self.poi_alt = float(self._get_param('AAS_TARG_ALT'))

    @classmethod
    def add_args(cls, parser):
        p = cls.PARAMS
        g = parser.add_argument_group("Simulator POI arguments")
        g.add_argument("-twps", dest='AAS_TARG_WPS', type=str, default=p['AAS_TARG_WPS'],
                       help=f"Simulation POI waypoint numbers (1-based, comma-separated). Default: WP bitmask {p['AAS_TARG_WPS']}")
        g.add_argument('-talt', dest='AAS_TARG_ALT', type=int, default=p['AAS_TARG_ALT'],
                       help=f"Simulator POI altitude offset (m). Default: {p['AAS_TARG_ALT']}")
