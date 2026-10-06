import time
from typing import Dict, Any

from pymavlink.dialects.v20.ardupilotmega import MAVLink_check_in_message, \
    MAVLink_message, MAVLink_check_out_message, MAVLink_slot_heartbeat_message, MAVLink_slot_claim_message, \
    MAVLink_vote_phase_message, MAVLink_search_status_message

from navpy.modules.comm.messages.msg_abc import MsgRegistry, MsgABC
from navpy.modules.comm.messages.types import MsgType
from navpy.modules.vehicle.vehicle_mav import VehicleMav


def create_mock_msg_class(mav_msg: MAVLink_message):
    """
    Dynamically create a mock MsgABC class for a MAVLink message
    that doesn't have a registered wrapper.
    """
    captured_mav_id = mav_msg.get_msgId()
    captured_msg_name = mav_msg.get_type()

    class MockMsg(MsgABC):
        _mav_id = None  # Set after class definition
        _msg_name = None  # Set after class definition

        def __init__(self, sender_id: int = 0):
            super().__init__(sender_id)
            self._mav_msg = None

        def to_dict(self) -> Dict[str, Any]:
            data = super().to_dict()
            if self._mav_msg:
                data.update(self._mav_msg.to_dict())
            return data

        @classmethod
        def from_dict(cls, data: Dict[str, Any]) -> 'MockMsg':
            return cls(data.get('sid', 0))

        @classmethod
        def msg_type(cls) -> MsgType:
            return MsgType.UNKNOWN

        @classmethod
        def mav_id(cls) -> int:
            return cls._mav_id

        def to_mavlink(self):
            return self._mav_msg

        @classmethod
        def from_mavlink(cls, mav_msg: MAVLink_message) -> 'MockMsg':
            instance = cls(sender_id=mav_msg.get_srcSystem())
            instance._mav_msg = mav_msg
            return instance

    # Set class attributes after definition to capture closure variables
    MockMsg._mav_id = captured_mav_id
    MockMsg._msg_name = captured_msg_name

    # Give class a unique name
    MockMsg.__name__ = f"Mock_{captured_msg_name}"
    MockMsg.__qualname__ = f"Mock_{captured_msg_name}"

    return MockMsg


def register_mock_if_needed(mav_msg: MAVLink_message):
    """Register a mock class for MAVLink messages without a wrapper."""
    mav_id = mav_msg.get_msgId()
    if not MsgRegistry.has_mav_id(mav_id):
        mock_class = create_mock_msg_class(mav_msg)
        MsgRegistry.register(mock_class)
        print(f"Registered mock for {mav_msg.get_type()} (ID: {mav_id})")


messages_to_test = [
    MAVLink_check_in_message(12345, 1, 1000, 1500),
    MAVLink_check_out_message(12345, 1, 1000, 1500, 40, 40, 1500),
    MAVLink_slot_heartbeat_message(12345, 1, 1000, 1500, slot_id=1, state=0),
    MAVLink_slot_claim_message(12345, 1, 1000, 1500, slot_id=1, priority=10),
    MAVLink_vote_phase_message(12345, 1, 1000, 1500, phase=1, round_id=1, proposal_id=1, vote=1),
    MAVLink_search_status_message(12345, 1, 1000, 1500, area_id=1, status=0, coverage_pct=50, detections=3),
]

message_ids = []
for msg in messages_to_test:
    message_ids.append(msg.get_msgId())
    register_mock_if_needed(msg)


def on_mavlink_message(sender, msg: MAVLink_message):
    if msg.get_msgId() not in message_ids:
        # print(f"Got message {msg.msgname}")
        return
    print(f"[DRONE {sender}] ← [DRONE {msg.get_srcSystem()}] - {str(msg.get_type())}: {msg.to_dict()}")


vehicle1 = VehicleMav("udp:0.0.0.0:14560",
                      1,
                      baud=115200,
                      wait_heartbeat=True,
                      skip_mission_download=True)
vehicle1.on_message("*", lambda msg: on_mavlink_message(1, msg))

vehicle2 = VehicleMav("udp:0.0.0.0:14570",
                      2,
                      baud=115200,
                      wait_heartbeat=True,
                      skip_mission_download=True)
vehicle2.on_message("*", lambda msg: on_mavlink_message(2, msg))

vehicle2.wait_heartbeat_from(1, 10.0)
vehicle1.wait_heartbeat_from(2, 10.0)

for msg in messages_to_test:
    print(f"Sending [DRONE {2}] -> [ALL] {msg.msgname}")
    vehicle2.send_mavlink_message(msg)

time.sleep(5)
print("\nWaiting for messages...")

try:
    while True:
        time.sleep(3)

finally:
    vehicle1.close()
    vehicle2.close()
    print("Connection closed.")
