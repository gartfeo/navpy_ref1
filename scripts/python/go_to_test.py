import time

from pymavlink import mavutil
from pymavlink.dialects.v20.ardupilotmega import MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, POSITION_TARGET_TYPEMASK_VX_IGNORE, \
    POSITION_TARGET_TYPEMASK_VY_IGNORE, \
    POSITION_TARGET_TYPEMASK_VZ_IGNORE, POSITION_TARGET_TYPEMASK_AX_IGNORE, POSITION_TARGET_TYPEMASK_AY_IGNORE, \
    POSITION_TARGET_TYPEMASK_AZ_IGNORE, POSITION_TARGET_TYPEMASK_YAW_IGNORE, POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE, \
    MAV_FRAME_GLOBAL_RELATIVE_ALT, MAV_CMD_NAV_WAYPOINT

from navpy.modules.common.models.location import Location

BOOT_TIME = time.time()
vehicle_id = 2
conn = mavutil.mavlink_connection(
    "udp:0.0.0.0:14570",
    baud=115200,
    autoreconnect=True,
)
conn.wait_heartbeat(blocking=True, timeout=30.0)

mode_id = conn.mode_mapping().get("GUIDED")

conn.mav.set_mode_send(
    vehicle_id,
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id,
)

conn.mav.statustext_send(
    mavutil.mavlink.MAV_SEVERITY_INFO,
    b"Mode set to GUIDED"
)

print(f"Mode set to GUIDED for vehicle {vehicle_id}")
TYPE_MASK_POS_ONLY = (POSITION_TARGET_TYPEMASK_VX_IGNORE
                      | POSITION_TARGET_TYPEMASK_VY_IGNORE
                      | POSITION_TARGET_TYPEMASK_VZ_IGNORE
                      | POSITION_TARGET_TYPEMASK_AX_IGNORE
                      | POSITION_TARGET_TYPEMASK_AY_IGNORE
                      | POSITION_TARGET_TYPEMASK_AZ_IGNORE
                      | POSITION_TARGET_TYPEMASK_YAW_IGNORE
                      | POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)

t_loc = Location(
    lat=40.3117411,
    lng=44.4552112,
    alt=200.0,  # Altitude in meters
    is_absolute=False)

# conn.mav.set_position_target_global_int_send(
#     int((time.time() - BOOT_TIME) * 1000) & 0xFFFFFFFF,
#     vehicle_id,  # target_system
#     0,  # target_component (0 for all components)
#     MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
#     TYPE_MASK_POS_ONLY,
#     int(round(t_loc.lat * 1e7)),
#     int(round(t_loc.lng * 1e7)),
#     float(t_loc.alt),
#     0, 0, 0,  # vx (ignored)
#     0, 0, 0,  # ax (ignored)
#     0,  # yaw (ignored)
#     0  # yaw_rate (ignored)
# )

conn.mav.mission_item_send(vehicle_id, 0,
                           0,
                           MAV_FRAME_GLOBAL_RELATIVE_ALT,
                           MAV_CMD_NAV_WAYPOINT, 2, 0, 0,
                           0, 0, 0, t_loc.lat, t_loc.lng,
                           t_loc.alt)
# conn.mav.command_long_send(
#     vehicle_id,
#     0,
#     MAV_CMD_DO_REPOSITION,
#     0,
#     0, 0, 0, 0,                              # params 1..4 unused here
#     float(t_loc.lat), float(t_loc.lng), float(t_loc.alt)
# )
