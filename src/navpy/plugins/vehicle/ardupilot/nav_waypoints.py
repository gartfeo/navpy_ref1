"""Mission NAV_WAYPOINT items that mark POIs, and their mission indices."""

from pymavlink.dialects.v20.ardupilotmega import MAV_CMD_NAV_WAYPOINT

from navpy.args.vehicle_arg_ports import MissionParameterReader


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
