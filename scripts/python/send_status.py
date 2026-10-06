from pymavlink import mavutil
from pymavlink.dialects.v20.ardupilotmega import MAV_SEVERITY_INFO

vehicle_id = 158

conn = mavutil.mavlink_connection(
    "udp:0.0.0.0:14570",
    baud=115200,
    source_system=vehicle_id,
    source_component=191,
    autoreconnect=True,
)
conn.wait_heartbeat(blocking=True, timeout=30.0)

mav_as_vehicle = mavutil.mavlink.MAVLink(conn.mav.file)
mav_as_vehicle.srcSystem = 0
mav_as_vehicle.srcComponent = 0
msg = mavutil.mavlink.MAVLink_statustext_message(MAV_SEVERITY_INFO, b"157 Mode set to GUIDED")
mav_as_vehicle.send(msg)

conn.mav.statustext_send(
    mavutil.mavlink.MAV_SEVERITY_INFO,
    b"SHOULD NOT SEND"
)

# status_msg = MAVLink_statustext_message(MAV_SEVERITY_INFO, b"253 Mode set to GUIDED")
# status_msg._header.srcSystem = 2
# conn.mav.send(status_msg)
