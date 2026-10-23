"""Bounded mission transfers (``docs/design/radio_budget.md``, todo 105): the item
and ACK waits, the attempt budget, one deadline per transfer, the link-silence
end, the stop cancel, and the upload's answers to the autopilot's re-requests."""
from __future__ import annotations

import threading
import time
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

from pymavlink.dialects.v20.ardupilotmega import (
    MAV_MISSION_ACCEPTED,
    MAV_MISSION_ERROR,
    MAV_MISSION_INVALID_SEQUENCE,
    MAVLink_mission_item_int_message,
)

from navpy.core.kernel.logging.cache_logger import ConsoleLogger
from navpy.core.ports.parameter_exchange import ExchangePolicy
from navpy.plugins.vehicle.ardupilot.link_state import HeartbeatState
from navpy.plugins.vehicle.ardupilot.mav_transport import MavTransport
from navpy.plugins.vehicle.ardupilot.message_store import MessageStore
from navpy.plugins.vehicle.ardupilot.mission_download import MissionDownloader
from navpy.plugins.vehicle.ardupilot.mission_inbox import MissionInbox
from navpy.plugins.vehicle.ardupilot.mission_store import MissionStore
from navpy.plugins.vehicle.ardupilot.mission_upload import MissionUploader
from navpy.plugins.vehicle.ardupilot.vehicle_foundation_composition import MESSAGE_HISTORY_POLICY
from navpy.plugins.vehicle.ardupilot.vehicle_mav import VehicleMav

TARGET = 7


def _message(message_type: str, **fields) -> MagicMock:
    message = MagicMock(mission_type=0, **fields)
    message.get_type.return_value = message_type
    message.get_srcSystem.return_value = TARGET
    return message


class Clock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now


class Autopilot:
    """Answers a download: ``drop`` holds item seqs whose next reply is lost."""

    def __init__(self, total: int, *, count_replies: bool = True) -> None:
        self.messages = MessageStore(MESSAGE_HISTORY_POLICY)
        self.connection = SimpleNamespace(mav=MagicMock())
        self.total = total
        self.drop: list[int] = []
        self.answer_items = True
        self.on_item_request = lambda _seq: None
        mav = self.connection.mav
        if count_replies:
            mav.mission_request_list_send.side_effect = lambda *_args: self.messages.publish(
                "MISSION_COUNT", _message("MISSION_COUNT", count=total), receipt_time_s=1.0)
        mav.mission_request_int_send.side_effect = self._item

    def _item(self, _target, _component, seq, _mission_type) -> None:
        self.on_item_request(seq)
        if seq in self.drop:
            self.drop.remove(seq)
        elif self.answer_items:
            self.messages.publish("MISSION_ITEM_INT", _message("MISSION_ITEM_INT", seq=seq),
                                  receipt_time_s=1.0)

    def item_requests(self, seq: int) -> int:
        return sum(1 for call in self.connection.mav.mission_request_int_send.call_args_list
                   if call.args[2] == seq)

    def downloader(self, link=None, cancelled=None) -> MissionDownloader:
        return MissionDownloader(TARGET, MavTransport(self.connection, threading.RLock()),
                                 MissionStore(TARGET, self.messages), self.messages,
                                 SimpleNamespace(value=MagicMock()),
                                 link, cancelled)


def test_a_lost_item_reply_is_requested_again_after_a_quarter_second():
    autopilot = Autopilot(3)
    autopilot.drop = [1]
    started = time.monotonic()
    assert autopilot.downloader().download() == 3
    assert time.monotonic() - started < 1.0  # one 0.25 s item wait, not a 10 s one
    assert [autopilot.item_requests(seq) for seq in range(3)] == [1, 2, 1]


def test_a_silent_autopilot_gets_six_count_requests_by_default():
    autopilot = Autopilot(3, count_replies=False)
    assert autopilot.downloader().download(ack_timeout=0.001) == 0
    assert autopilot.connection.mav.mission_request_list_send.call_count == 6


def test_one_deadline_spans_every_step_of_a_download():
    autopilot = Autopilot(4)
    autopilot.on_item_request = lambda seq: time.sleep(0.04) if seq < 3 else None
    autopilot.drop = [3] * 100
    started = time.monotonic()
    count = autopilot.downloader().download(
        policy=ExchangePolicy(attempts=100, deadline_s=time.monotonic() + 0.3), item_timeout=0.05)
    assert count == 0
    assert time.monotonic() - started < 0.6
    # Items 0-2 used up part of the budget; item 3 got only what was left of it.
    assert autopilot.item_requests(3) < 6


def test_link_silence_ends_a_gcs_download():
    clock = Clock()
    link = HeartbeatState(TARGET, 5.0, clock)
    autopilot = Autopilot(3)
    autopilot.answer_items = False

    def go_silent(_seq):
        if autopilot.item_requests(0) == 2:
            clock.now += 5.1

    autopilot.on_item_request = go_silent
    count = autopilot.downloader(link).download(
        policy=ExchangePolicy(attempts=12, stop_on_silence=True), item_timeout=0.001)
    assert count == 0
    assert autopilot.item_requests(0) == 2


def test_a_stop_ends_a_download_within_one_wait():
    stopped = threading.Event()
    autopilot = Autopilot(3)
    autopilot.answer_items = False
    autopilot.on_item_request = lambda _seq: stopped.set()
    started = time.monotonic()
    assert autopilot.downloader(cancelled=stopped.is_set).download() == 0
    assert time.monotonic() - started < 0.5  # one 0.25 s item wait, then the cancel
    assert autopilot.item_requests(0) == 1


class Receiver:
    """The autopilot side of an upload: ``script`` maps each item it gets to replies."""

    def __init__(self, total: int) -> None:
        self.inbox = MissionInbox()
        self.connection = SimpleNamespace(mav=MagicMock())
        self.store = MagicMock()
        self.store.snapshot.return_value = [MagicMock() for _ in range(total)]
        self.script: dict[tuple[int, int], list[MagicMock]] = {}
        self.items: list[int] = []
        mav = self.connection.mav
        mav.mission_count_send.side_effect = lambda *_args: self.publish(
            _message("MISSION_REQUEST_INT", seq=0))
        mav.mission_item_int_send.side_effect = self._item

    def publish(self, message: MagicMock) -> None:
        self.inbox.publish(message, time.time())

    def _item(self, _target, _component, seq, *_args) -> None:
        self.items.append(seq)
        for reply in self.script.get((seq, self.items.count(seq)), []):
            self.publish(reply)

    def uploader(self, cancelled=None, link=None) -> MissionUploader:
        return MissionUploader(TARGET, MavTransport(self.connection, threading.RLock()),
                               self.store, self.inbox, SimpleNamespace(value=MagicMock()),
                               link, cancelled)


def _request(seq: int) -> MagicMock:
    return _message("MISSION_REQUEST_INT", seq=seq)


def _ack(kind: int) -> MagicMock:
    return _message("MISSION_ACK", type=kind)


def test_the_upload_answers_a_re_request_of_the_last_item_during_the_ack_wait():
    receiver = Receiver(2)
    receiver.script = {
        (0, 1): [_request(1)],
        (1, 1): [_request(1)],  # the item was lost: ArduPilot asks again
        (1, 2): [_ack(MAV_MISSION_ACCEPTED)],
    }
    assert receiver.uploader().upload(ack_timeout=0.1, abandon_s=0.5)
    assert receiver.items == [0, 1, 1]


def test_the_upload_ignores_an_invalid_sequence_ack_from_a_late_duplicate():
    receiver = Receiver(2)
    receiver.script = {
        (0, 1): [_ack(MAV_MISSION_INVALID_SEQUENCE), _request(1)],
        (1, 1): [_ack(MAV_MISSION_ACCEPTED)],
    }
    assert receiver.uploader().upload(ack_timeout=0.1, abandon_s=0.5)


def test_any_other_refusal_ends_the_upload_at_once():
    receiver = Receiver(2)
    receiver.script = {(0, 1): [_ack(MAV_MISSION_ERROR)]}
    started = time.monotonic()
    assert not receiver.uploader().upload(ack_timeout=0.1, abandon_s=5.0)
    assert time.monotonic() - started < 0.5
    assert receiver.items == [0]


def test_a_refusal_of_the_count_ends_the_upload_after_one_send():
    receiver = Receiver(2)
    receiver.connection.mav.mission_count_send.side_effect = lambda *_args: receiver.publish(
        _ack(MAV_MISSION_ERROR))
    started = time.monotonic()
    assert not receiver.uploader().upload(ack_timeout=0.1, abandon_s=5.0)
    assert time.monotonic() - started < 0.5
    assert receiver.connection.mav.mission_count_send.call_count == 1
    assert receiver.items == []


def test_an_upload_the_autopilot_dropped_ends_at_the_abandon_bound():
    receiver = Receiver(2)
    started = time.monotonic()
    assert not receiver.uploader().upload(ack_timeout=0.05, abandon_s=0.3)
    assert 0.3 <= time.monotonic() - started < 1.0
    assert receiver.items == [0]


def test_a_stop_ends_an_upload_within_one_ack_wait():
    stopped = threading.Event()
    receiver = Receiver(2)
    receiver.connection.mav.mission_item_int_send.side_effect = lambda *_args: stopped.set()
    started = time.monotonic()
    assert not receiver.uploader(stopped.is_set).upload(ack_timeout=0.2)  # abandon stays 8 s
    assert time.monotonic() - started < 1.0


def test_after_a_stop_the_vehicle_ends_reads_and_transfers_at_once_but_still_writes():
    connection = MagicMock()
    connection.recv_match.return_value = None
    with patch("navpy.plugins.vehicle.ardupilot.mav_bus.mavutil.mavlink_connection",
               return_value=connection),          patch("navpy.plugins.vehicle.ardupilot.vehicle_mav.mavutil.mavlink.MAVLink",
               return_value=MagicMock()):
        stopped = threading.Event()
        vehicle = VehicleMav(device="dev_stop_cancels_reads", target_system=TARGET,
                             logger=ConsoleLogger(), skip_mission_download=True,
                             wait_heartbeat=False, stop_requested=stopped.is_set)
        try:
            stopped.set()
            started = time.monotonic()
            assert vehicle.get_parameter_fresh("WP_LOITER_RAD", timeout=5.0) is None
            assert vehicle.download_mission() == 0
            vehicle.load_mission_items(_loader(2))
            assert vehicle.upload_mission() is False
            assert time.monotonic() - started < 0.5
            connection.mav.param_request_read_send.assert_not_called()
            connection.mav.mission_request_list_send.assert_not_called()
            connection.mav.mission_count_send.assert_not_called()

            vehicle.set_parameter("WP_LOITER_RAD", 80.0, timeout=0.01)  # a restore
            assert connection.mav.param_set_send.called
        finally:
            vehicle.close()


def _loader(count: int):
    from pymavlink import mavwp
    loader = mavwp.MAVWPLoader()
    for seq in range(count):
        loader.add(MAVLink_mission_item_int_message(
            TARGET, 0, seq, 0, 16, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0))
    return loader
