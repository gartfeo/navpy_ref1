import time

from pymavlink.dialects.v20.ardupilotmega import MAV_TYPE_GCS, MAV_COMP_ID_SYSTEM_CONTROL

from navpy.modules.comm.messages.available_task_msg import (
    TaskMsgData,
    AvailableTaskRequestMsg,
    TaskHandleMsgData,
    AvailableTaskResponseMsg)
from navpy.modules.comm.messages.location_msg import LocationMsgData
from navpy.modules.comm.messages.types import TaskTypeMsgData
from navpy.modules.vehicle.vehicle_mav import VehicleMav

vehicle1 = VehicleMav("udp:0.0.0.0:14560",
                    1,
                    baud=115200,
                    wait_heartbeat=True,
                    skip_mission_download=True)
vehicle1.on_message("NAVLINK", lambda msg: print(
    f"[DRONE 1] ← [DRONE {msg.get_srcSystem()}] - {str(msg.get_type())}: {msg.to_dict()}"))

vehicle2 = VehicleMav("udp:0.0.0.0:14570",
                    2,
                    baud=115200,
                    wait_heartbeat=True,
                    skip_mission_download=True)
vehicle2.on_message("NAVLINK", lambda msg: print(
    f"[DRONE 2] ← [DRONE {msg.get_srcSystem()}] - {str(msg.get_type())}: {msg.to_dict()}"))

gcs = VehicleMav("udp:0.0.0.0:14500",
                 255,
                 baud=115200,
                 wait_heartbeat=False,
                 skip_mission_download=True,
                 mav_type=MAV_TYPE_GCS,
                 mav_comp_id=MAV_COMP_ID_SYSTEM_CONTROL)

gcs.on_message("NAVLINK",
               lambda msg: print(f"[GCS] ← [DRONE {msg.get_srcSystem()}] - {str(msg.get_type())}: {msg.to_dict()}"))
# gcs.send_status_text("Alive and ready to receive messages.")
#
# gcs.on_message("*", lambda msg: print(f"[GCS] ← {str(msg.msg_type())}: {msg.to_dict()}"))

#
# vehicle1.send_mavlink_message(CheckInMsg(vehicle1.target_system).to_mavlink())
# vehicle2.send_mavlink_message(CheckInMsg(vehicle2.target_system).to_mavlink())

task = TaskMsgData(111, TaskTypeMsgData.MEDIUM, LocationMsgData(44.0, 45.0, 1300))
available_request_msg = AvailableTaskRequestMsg(vehicle1.source_system, [task])
vehicle1.send_mavlink_message(available_request_msg.to_mavlink())

time.sleep(1)

available_response_msg = AvailableTaskResponseMsg(vehicle2.source_system, vehicle1.target_system,
                                                  [TaskHandleMsgData(task.task_id, 15)])
vehicle2.send_mavlink_message(available_response_msg.to_mavlink())

print("\nWaiting for messages...")
try:
    while True:
        time.sleep(0.01)

finally:
    vehicle1.close()
    vehicle2.close()
    print("Connection closed.")
