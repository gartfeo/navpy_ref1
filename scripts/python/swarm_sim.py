import time
from types import SimpleNamespace

from pymavlink.dialects.v20.ardupilotmega import MAVLink_message

from navpy.args.conn.network_args import NetworkArgs, NetworkType
from navpy.args.conn_args import ConnArgs
from navpy.modules.comm.listener_abc import ListenerAbc
from navpy.modules.comm.messages.available_task_msg import (
    TaskConfirmRequestMsg, TaskMsgData, AvailableTaskRequestMsg, AvailableTaskResponseMsg,
    TaskHandleMsgData, TaskAssignRequestMsg, TaskAssignResponseMsg, TaskAssignMsgData, TaskConfirmResponseMsg
)
from navpy.modules.comm.messages.check_msg import CheckInMsg, CheckOutMsg
from navpy.modules.comm.messages.location_msg import LocationMsgData
from navpy.modules.comm.messages.msg_abc import MsgABC
from navpy.modules.comm.messages.types import TaskTypeMsgData, MsgType
from navpy.modules.comm.network_factory import create_network
from navpy.modules.vehicle.vehicle_factory import create_vehicle
from navpy.logger.cache_logger import ConsoleLogger


class MsgListener(ListenerAbc):
    def __init__(self, name: str, id: int, network: 'NetworkAbc'):
        self.name = name
        self.id = id
        self.network = network

    def on_message(self, message: MsgABC):
        print(f"[{self.name}] ← {str(message.msg_type())}: {message.to_dict()}")

        if message.msg_type() == MsgType.TASK_CONFIRM_RESPONSE:
            msg: TaskConfirmResponseMsg = message
            if msg.is_confirmed:
                print(f"[{self.name}] Task {msg.task_id} accepted by Vehicle {msg.sender_id}.")
            else:
                print(f"[{self.name}] Task {msg.task_id} rejected by Vehicle {msg.sender_id}.")
        elif message.msg_type() == MsgType.AVAILABLE_TASK_REQUEST:
            msg: AvailableTaskRequestMsg = message
            task = msg.tasks[0]
            self.network.broadcast(
                AvailableTaskResponseMsg(self.id, msg.sender_id, [TaskHandleMsgData(task.task_id, 15)]))
        elif message.msg_type() == MsgType.AVAILABLE_TASK_RESPONSE:
            msg: AvailableTaskResponseMsg = message
            loc = LocationMsgData(44.0, 45.0, 1300)
            task = TaskMsgData(43, TaskTypeMsgData.SMALL, loc)
            self.network.broadcast(TaskAssignRequestMsg(self.id, msg.sender_id,
                                                        TaskAssignMsgData(task.task_id, task.task_type, task.location)))
        elif message.msg_type() == MsgType.TASK_ASSIGN_REQUEST:
            msg: TaskAssignRequestMsg = message
            self.network.broadcast(TaskAssignResponseMsg(self.id, msg.sender_id, msg.task.task_id, True))
            self.network.broadcast(
                TaskConfirmRequestMsg(self.id, TaskMsgData(msg.task.task_id, msg.task.task_type, msg.task.location)))
        elif message.msg_type() == MsgType.TASK_ASSIGN_RESPONSE:
            msg: TaskAssignResponseMsg = message
            status = "accepted" if msg.is_accepted else "rejected"
            print(f"[{self.name}] Task {msg.task_id} {status} by Vehicle {msg.sender_id}.")
        elif message.msg_type() == MsgType.CHECK_IN:
            msg: CheckInMsg = message
            print(f"[{self.name}] Vehicle {msg.sender_id} checked in.")
        elif message.msg_type() == MsgType.CHECK_OUT:
            msg: CheckOutMsg = message
            print(f"[{self.name}] Vehicle {msg.sender_id} checked out at location {msg.location.to_dict()}.")


class HeartbeatListener(ListenerAbc):
    def __init__(self, vehicle_id: int):
        self.vehicle_id = vehicle_id

    def on_message(self, msg: MAVLink_message):
        if msg.get_type() == 'HEARTBEAT' and msg.get_srcSystem() != self.vehicle_id:
            print(f"DRONE {self.vehicle_id} -> OTHER HEARTBIT received {msg.get_srcSystem()} : {msg.to_dict()}")


def make_node(node_id: int, conn_str: str):
    # 1) “Fake” a companion link just to satisfy create_vehicle
    conn = SimpleNamespace(connection=conn_str,
                           baud=115200,
                           source_system=node_id)
    vehicle = create_vehicle(ConnArgs(conn), skip_mission_download=True, wait_for_heartbeat=False)

    heartbeat = HeartbeatListener(node_id)
    vehicle.on_message("HEARTBEAT", heartbeat.on_message)

    # 2) Network on its own UDP port, pointing at peer(s)
    net_cfg = SimpleNamespace(
        network_type=NetworkType.MAV,
        serial_conn='', serial_baud='',
        serial_conn_multi='',
        wifi_self_port='', wifi_peers_ports=''
    )
    network = create_network(NetworkArgs(net_cfg), node_id, ConsoleLogger(), vehicle)
    network.set_listener(MsgListener(f"Vehicle{node_id}", node_id, network))
    network.broadcast(CheckInMsg(0))
    return vehicle, network


def run_simulation():
    # IDs and ports
    ID1, ID2 = 1, 2
    # C1, C2 = 'tcp:127.0.0.1:5760', 'tcp:127.0.0.1:5770'
    # C1, C2 = 'tcp:127.0.0.1:5762', 'tcp:127.0.0.1:5772'
    # C1, C2 = 'tcp: 172.21.176.1:5762', 'tcp: 172.21.176.1:5772'
    # C1, C2 = 'udp:0.0.0.0:15000', 'udp:0.0.0.0:14570'
    # C1, C2 = 'udp:0.0.0.0:15000', 'udp:0.0.0.0:16000'
    C1, C2 = 'udp:0.0.0.0:14560', 'udp:0.0.0.0:14570'

    # bring up both nodes
    d2, n2 = make_node(ID2, C2)
    d1, n1 = make_node(ID1, C1)


    d2.wait_heartbeat_from(ID1, 10.0)  # wait
    d1.wait_heartbeat_from(ID2, 10.0)  # wait for heartbeat from Drone2

    try:
        # 1) both check in
        # d1.send_mavlink_message(MAVLink_statustext_message(MAV_SEVERITY_INFO, b"Drone1 checking in"))
        # d2.send_mavlink_message(MAVLink_statustext_message(MAV_SEVERITY_INFO, b"Drone2 checking in"))
        # print("STATUSTEXT sent")
        #
        # n1.broadcast(CheckInMsg(ID1))
        # n2.broadcast(CheckInMsg(ID2))
        # time.sleep(0.1)
        # print("CHECKIN sent")
        #
        # # 2) share locations
        loc1 = LocationMsgData(44.0, 45.0, 1300)
        # loc2 = LocationMsgData(44.1, 45.1, 1400)
        # n1.broadcast(CheckOutMsg(ID1, loc1))
        # n2.broadcast(CheckOutMsg(ID2, loc2))
        # time.sleep(0.1)
        # print("CHECKOUT sent")
        #
        # # 3) Drone1 → GCS: task confirm request
        task1 = TaskMsgData(42, TaskTypeMsgData.MEDIUM, loc1)
        n1.broadcast(TaskConfirmRequestMsg(ID1, task1))
        # time.sleep(0.1)
        # print("TASK_CONFIRM_REQUEST sent")

        # # 5) Drone1 → Drone2: available task request
        task2 = TaskMsgData(43, TaskTypeMsgData.SMALL, loc1)
        n1.broadcast(AvailableTaskRequestMsg(ID1, [task2]))
        # d1.send_mavlink_message(
        #     MAVLink_heartbeat_message(MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE))
        time.sleep(0.1)
        print("AVAILABLE_TASK_REQUEST sent")

        # 6) Drone2 → Drone1: available task response
        # n2.broadcast(AvailableTaskResponseMsg(ID2, ID1, [TaskHandleMsgData(task2.task_id, 15)]))

        # 7) Drone1 → Drone2: task assign request
        # n1.broadcast(TaskAssignRequestMsg(ID1, ID2, TaskAssignMsgData(task2.task_id, task2.task_type, task2.location)))
        # time.sleep(0.1)

        # 8) Drone2 → Drone1: task assign response
        # n2.broadcast(TaskAssignResponseMsg(ID2, ID1, task2.task_id, True))
        # time.sleep(0.1)

        # 9) Drone2 → GCS: task confirm request
        # n2.broadcast(TaskConfirmRequestMsg(ID2, task2))

        print("Waiting for messages...")
        while True:
            time.sleep(0.1)
    except Exception as e:
        print(f"Simulation error: {e}")
        raise
    finally:
        for net, dr in [(n1, d1), (n2, d2)]:
            dr.close()
            net.close()


if __name__ == '__main__':
    run_simulation()
