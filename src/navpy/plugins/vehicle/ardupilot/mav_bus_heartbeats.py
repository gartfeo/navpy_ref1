"""Thread-safe heartbeat observation for a shared MAVLink bus."""
from __future__ import annotations


from pymavlink.dialects.v20.ardupilotmega import (
    MAVLink_message,
    MAV_TYPE_CAMERA,
    MAV_TYPE_GCS,
    MAV_TYPE_ONBOARD_CONTROLLER,
)

from navpy.core.kernel import runtime_time as rt

_IGNORED_SYSTEM_IDS = frozenset({0, 254, 255})
# Not the vehicle: a companion camera heartbeats under the vehicle's sysid
# (camera protocol, docs/design/gcs_video_streams.md) and must not keep a
# silent autopilot looking alive.
_NON_VEHICLE_TYPES = frozenset({MAV_TYPE_GCS, MAV_TYPE_CAMERA})


class MavBusHeartbeats:
    def __init__(self) -> None:
        self._condition = rt.Condition()
        self._vehicle: dict[int, float] = {}
        self._companion: dict[int, float] = {}

    def observe(self, message: MAVLink_message) -> None:
        system_id = message.get_srcSystem()
        if message.get_type() != "HEARTBEAT" or system_id in _IGNORED_SYSTEM_IDS:
            return
        mav_type = message.type
        now_s = rt.time()
        with self._condition:
            if mav_type == MAV_TYPE_ONBOARD_CONTROLLER:
                self._companion[system_id] = now_s
            elif mav_type not in _NON_VEHICLE_TYPES:
                self._vehicle[system_id] = now_s
            self._condition.notify_all()

    def wait_for(self, system_id: int, timeout_s: float) -> bool:
        deadline_s = rt.monotonic() + timeout_s
        with self._condition:
            while system_id not in self._vehicle:
                remaining_s = deadline_s - rt.monotonic()
                if remaining_s <= 0.0:
                    return False
                self._condition.wait(remaining_s)
            return True

    @property
    def vehicle(self) -> dict[int, float]:
        with self._condition:
            return dict(self._vehicle)

    @property
    def companion(self) -> dict[int, float]:
        with self._condition:
            return dict(self._companion)
