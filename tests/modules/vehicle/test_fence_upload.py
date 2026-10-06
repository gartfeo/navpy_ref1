"""Fence mission-table transport remains isolated from the flight mission."""
from __future__ import annotations

from types import SimpleNamespace
import threading
import time
import unittest
from unittest.mock import MagicMock

from pymavlink.dialects.v20.ardupilotmega import (
    MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
    MAV_MISSION_ACCEPTED,
    MAV_MISSION_TYPE_FENCE,
    MAV_MISSION_TYPE_MISSION,
)
from pymavlink.mavwp import MAVWPLoader

from gcs.backend.planner.fence_builder import build_fence_items
from navpy.modules.vehicle.logger_ref import LoggerRef
from navpy.modules.vehicle.mav_mission import MavMission
from navpy.modules.vehicle.mav_transport import MavTransport
from navpy.modules.vehicle.message_store import MessageStore
from navpy.modules.vehicle.mission_inbox import MissionInbox


def _protocol_message(message_type, *, sequence=0, ack_type=0, mission_type=0):
    message = MagicMock(seq=sequence, type=ack_type, mission_type=mission_type)
    message.get_type.return_value = message_type
    message.get_srcSystem.return_value = 3
    return message


def _seed_loader(*locations):
    loader = MAVWPLoader(3, 0)
    for latitude, longitude, altitude in locations:
        loader.add_latlonalt(latitude, longitude, altitude)
    return loader


class MissionHarness:
    def __init__(self) -> None:
        self.messages = MessageStore()
        self.inbox = MissionInbox()
        self.mav = MagicMock()
        self.mission = MavMission(
            3,
            MavTransport(SimpleNamespace(mav=self.mav), threading.RLock()),
            LoggerRef(MagicMock()),
            self.messages,
            self.inbox,
        )

    def install_upload_handshake(self, total: int, recorded: dict) -> None:
        def count_send(_target, _component, count, mission_type=0):
            recorded["count"] = (count, mission_type)
            self.inbox.publish(
                _protocol_message(
                    "MISSION_REQUEST_INT",
                    sequence=0,
                    mission_type=mission_type,
                ),
                time.time(),
            )

        def item_send(
            _target,
            _component,
            sequence,
            _frame,
            command,
            _current,
            _autocontinue,
            _p1,
            _p2,
            _p3,
            _p4,
            _x,
            _y,
            _z,
            mission_type=0,
        ):
            recorded["items"].append((sequence, command, mission_type))
            if sequence < total - 1:
                response = _protocol_message(
                    "MISSION_REQUEST_INT",
                    sequence=sequence + 1,
                    mission_type=mission_type,
                )
            else:
                response = _protocol_message(
                    "MISSION_ACK",
                    ack_type=MAV_MISSION_ACCEPTED,
                    mission_type=mission_type,
                )
            self.inbox.publish(response, time.time())

        self.mav.mission_count_send.side_effect = count_send
        self.mav.mission_item_int_send.side_effect = item_send

    def install_download_handshake(self, items: list, mission_type: int) -> None:
        def publish_count(*_args):
            count = _protocol_message(
                "MISSION_COUNT",
                mission_type=mission_type,
            )
            count.count = len(items)
            self.messages.publish(
                "MISSION_COUNT",
                count,
                receipt_time_s=time.time(),
            )

        def publish_item(_target, _component, sequence, _mission_type):
            item = items[sequence]
            item._header.srcSystem = 3
            self.messages.publish(
                "MISSION_ITEM_INT",
                item,
                receipt_time_s=time.time(),
            )

        self.mav.mission_request_list_send.side_effect = publish_count
        self.mav.mission_request_int_send.side_effect = publish_item


class TestFenceClear(unittest.TestCase):
    def test_clear_fence_does_not_wipe_mission(self):
        harness = MissionHarness()
        harness.mission.load_items(_seed_loader((1.0, 2.0, 3.0)))

        harness.mission.clear(mission_type=MAV_MISSION_TYPE_FENCE)

        harness.mav.mission_clear_all_send.assert_called_once_with(
            3,
            0,
            MAV_MISSION_TYPE_FENCE,
        )
        self.assertEqual(harness.mission.count, 1)

    def test_clear_mission_wipes_mission(self):
        harness = MissionHarness()
        harness.mission.load_items(_seed_loader((1.0, 2.0, 3.0)))

        harness.mission.clear()

        harness.mav.mission_clear_all_send.assert_called_once_with(
            3,
            0,
            MAV_MISSION_TYPE_MISSION,
        )
        self.assertEqual(harness.mission.count, 0)


class TestFenceUpload(unittest.TestCase):
    def test_upload_items_tags_fence_mission_type(self):
        vertices = [
            {"lat": 32.0, "lon": 34.0},
            {"lat": 32.01, "lon": 34.0},
            {"lat": 32.0, "lon": 34.01},
        ]
        items = build_fence_items(vertices, target_system=3)
        harness = MissionHarness()
        harness.mission.load_items(_seed_loader((1.0, 2.0, 3.0)))
        recorded = {"count": None, "items": []}
        harness.install_upload_handshake(len(items), recorded)

        result = harness.mission.upload(
            items=items,
            mission_type=MAV_MISSION_TYPE_FENCE,
        )

        self.assertTrue(result)
        self.assertEqual(recorded["count"], (3, MAV_MISSION_TYPE_FENCE))
        self.assertEqual(len(recorded["items"]), 3)
        self.assertTrue(all(
            mission_type == MAV_MISSION_TYPE_FENCE
            and command == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION
            for _sequence, command, mission_type in recorded["items"]
        ))
        self.assertEqual(harness.mission.count, 1)

    def test_upload_default_still_uses_mission_table(self):
        harness = MissionHarness()
        harness.mission.load_items(_seed_loader(
            (10.0, 20.0, 30.0),
            (11.0, 21.0, 31.0),
        ))
        recorded = {"count": None, "items": []}
        harness.install_upload_handshake(2, recorded)

        result = harness.mission.upload()

        self.assertTrue(result)
        self.assertEqual(recorded["count"], (2, MAV_MISSION_TYPE_MISSION))
        self.assertEqual(len(recorded["items"]), 2)


class TestFenceDownload(unittest.TestCase):
    def test_download_items_returns_fence_and_restores_mission(self):
        items = build_fence_items([
            {"lat": 32.0, "lon": 34.0},
            {"lat": 32.01, "lon": 34.0},
            {"lat": 32.0, "lon": 34.01},
        ], target_system=3)
        harness = MissionHarness()
        harness.mission.load_items(_seed_loader(
            (10.0, 20.0, 30.0),
            (11.0, 21.0, 31.0),
        ))
        harness.install_download_handshake(items, MAV_MISSION_TYPE_FENCE)

        received = harness.mission.download_items(
            mission_type=MAV_MISSION_TYPE_FENCE,
        )

        self.assertEqual(len(received), 3)
        self.assertTrue(all(
            item.command == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION
            for item in received
        ))
        self.assertEqual(harness.mission.count, 2)
        self.assertEqual(harness.mission.get_item(0).x, 10.0)
        self.assertEqual(harness.mission.get_item(1).x, 11.0)

    def test_empty_fence_returns_empty_and_restores_mission(self):
        harness = MissionHarness()
        harness.mission.load_items(_seed_loader((10.0, 20.0, 30.0)))
        harness.install_download_handshake([], MAV_MISSION_TYPE_FENCE)

        received = harness.mission.download_items(
            mission_type=MAV_MISSION_TYPE_FENCE,
        )

        self.assertEqual(received, [])
        self.assertEqual(harness.mission.count, 1)
        self.assertEqual(harness.mission.get_item(0).x, 10.0)


if __name__ == "__main__":
    unittest.main()
