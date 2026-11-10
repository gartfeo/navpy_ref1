"""The reference autopilot keeps one table per mission type, as ArduPilot does.

The GCS upload sets the rally table after the flight mission (the rally point
at the default delivery hub). A double that answered a rally transfer in the
flight mission's namespace left the uploader waiting for a reply it never
accepts, until the request deadline, which pushed the lockstep runs past their
finalization budget; and a rally clear wiped the flight mission.
"""
from __future__ import annotations

from pymavlink.dialects.v20 import ardupilotmega as mavlink

from navpy.sim.lockstep.mavlink_wire import StreamDecoder, encoder
from tests.modules.lockstep.lockstep_autopilot import MISSION, ReferenceAutopilot

RALLY = mavlink.MAV_MISSION_TYPE_RALLY


def _exchange(autopilot: ReferenceAutopilot, send) -> list[mavlink.MAVLink_message]:
    link, wire = encoder(255, 190)
    send(link)
    replies, _other = autopilot.step(0, [wire.drain(), b""])
    return [m for m in StreamDecoder().feed(replies) if m.get_type() != "HEARTBEAT"]


def _rally_point(seq: int) -> tuple:
    return (1, 1, seq, mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavlink.MAV_CMD_NAV_RALLY_POINT,
            0, 0, 0.0, 0.0, 0.0, 0.0, 401_100_000, 444_000_000, 100.0, RALLY)


def test_a_rally_upload_is_answered_in_the_rally_namespace_and_keeps_the_mission():
    autopilot = ReferenceAutopilot(heartbeats=frozenset())
    (request,) = _exchange(autopilot, lambda l: l.mission_count_send(1, 1, 1, RALLY))
    assert (request.get_type(), request.seq, request.mission_type) == ("MISSION_REQUEST_INT", 0, RALLY)
    (ack,) = _exchange(autopilot, lambda l: l.mission_item_int_send(*_rally_point(0)))
    assert (ack.get_type(), ack.type, ack.mission_type) == (
        "MISSION_ACK", mavlink.MAV_MISSION_ACCEPTED, RALLY)
    assert len(autopilot.mission) == len(MISSION)
    assert len(autopilot.tables[RALLY]) == 1 and autopilot.parameters["RALLY_TOTAL"] == 1.0
    (count,) = _exchange(autopilot, lambda l: l.mission_request_list_send(1, 1, RALLY))
    assert (count.count, count.mission_type) == (1, RALLY)


def test_a_rally_clear_empties_only_the_rally_table():
    autopilot = ReferenceAutopilot(heartbeats=frozenset())
    _exchange(autopilot, lambda l: l.mission_count_send(1, 1, 1, RALLY))
    _exchange(autopilot, lambda l: l.mission_item_int_send(*_rally_point(0)))
    (ack,) = _exchange(autopilot, lambda l: l.mission_clear_all_send(1, 1, RALLY))
    assert ack.mission_type == RALLY
    assert autopilot.tables[RALLY] == [] and autopilot.parameters["RALLY_TOTAL"] == 0.0
    assert len(autopilot.mission) == len(MISSION)
