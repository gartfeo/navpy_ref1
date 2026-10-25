"""The heartbeat count that dates a mode reading (todo 117, ``radio_budget.md`` round 7)."""
from unittest.mock import MagicMock

from pymavlink.dialects.v20.ardupilotmega import (
    MAV_TYPE_FIXED_WING,
    MAV_TYPE_GCS,
    MAV_TYPE_ONBOARD_CONTROLLER,
    MAVLink_heartbeat_message,
)

from gcs.backend.vehicle_snapshot import snapshot
from navpy.core.ports.flight_mode import FlightMode
from navpy.plugins.vehicle.ardupilot.link_state import HeartbeatState
from navpy.plugins.vehicle.ardupilot.mode_control import ModeControl


def _heartbeat(mav_type: int) -> MAVLink_heartbeat_message:
    return MAVLink_heartbeat_message(mav_type, 3, 0, 0, 0, 3)


def test_each_autopilot_heartbeat_counts_and_others_do_not():
    state = HeartbeatState(target_system=1, timeout_s=3.0)
    mode = ModeControl(MagicMock(), MagicMock(), state, MagicMock())
    assert state.count == mode.heartbeat_count == 0
    state.observe(_heartbeat(MAV_TYPE_FIXED_WING))
    state.observe(_heartbeat(MAV_TYPE_GCS))
    state.observe(_heartbeat(MAV_TYPE_ONBOARD_CONTROLLER))
    state.touch()
    assert state.count == mode.heartbeat_count == 1
    state.observe(_heartbeat(MAV_TYPE_FIXED_WING))
    assert mode.heartbeat_count == 2


class _Vehicle(MagicMock):
    """Records whether the mode was read after the heartbeat count."""

    def _record(self, name):
        self.__dict__.setdefault("reads", []).append(name)

    @property
    def heartbeat_count(self):
        self._record("count")
        return 7

    @property
    def get_mode(self):
        self._record("mode")
        return FlightMode.AUTO


def test_the_snapshot_reads_the_count_before_the_mode():
    entry = MagicMock()
    entry.vehicle = _Vehicle()
    entry._status_texts = []
    entry._accel_cal_events = []
    snap = snapshot(entry)
    assert snap["heartbeat_count"] == 7 and snap["mode"] == "AUTO"
    assert entry.vehicle.reads == ["count", "mode"]
