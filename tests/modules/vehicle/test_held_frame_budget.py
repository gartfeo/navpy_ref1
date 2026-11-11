"""A held frame on the uplink (``docs/design/radio_budget.md``, todo 103/105 budgets).

A lost uplink byte can break a frame header so that the receiver's parser waits for
up to ``MAVLINK2_MAX_FRAME_BYTES`` bytes and swallows every request in them. Those
sends fail together, so they must not use up a budget sized for independent losses
(``stress`` seed 40: item 10's twelve sends, eight of them inside one held frame).

The requests here are real MAVLink 2 frames from a pymavlink encoder; the radio
swallows the bytes of a held frame, then parses the rest as the autopilot would.
"""
from __future__ import annotations

import math
import threading
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
from pymavlink.dialects.v20.ardupilotmega import (
    MAVLink,
    MAVLink_mission_count_message,
    MAVLink_mission_item_int_message,
    MAVLink_param_value_message,
)

from navpy.core.ports.parameter_exchange import (
    MAVLINK2_MAX_FRAME_BYTES,
    ExchangeOutcome,
    ExchangePolicy,
    ExchangeReason,
)
from navpy.plugins.vehicle.ardupilot.mav_transport import MavTransport
from navpy.plugins.vehicle.ardupilot.message_store import MessageStore
from navpy.plugins.vehicle.ardupilot.mission_download import MissionDownloader
from navpy.plugins.vehicle.ardupilot.mission_store import MissionStore
from navpy.plugins.vehicle.ardupilot.parameter_exchange_loop import ExchangeIo, SendBudget, run_exchange
from navpy.plugins.vehicle.ardupilot.parameter_reader import ParameterReader
from navpy.plugins.vehicle.ardupilot.parameter_repository import ParameterRepository
from navpy.plugins.vehicle.ardupilot.vehicle_foundation_composition import MESSAGE_HISTORY_POLICY

TARGET = 7
#: The GCS budget (``gcs.backend.exchange_budget``): 12 sends, held frames not counted.
GCS = ExchangePolicy(attempts=12, held_frame_bytes=MAVLINK2_MAX_FRAME_BYTES)
#: The same budget counting every send, as before the fix.
EVERY_SEND_COUNTS = ExchangePolicy(attempts=12)


class HeldFrameRadio:
    """The uplink, byte by byte: after :meth:`lose_a_byte` the receiver swallows the
    next ``MAVLINK2_MAX_FRAME_BYTES`` bytes (the frame its broken header announced),
    then parses again. Frames that arrive are answered at once on a clean downlink."""

    def __init__(self, messages: MessageStore, total_items: int = 3) -> None:
        self.messages = messages
        self.total_items = total_items
        self.swallowing = 0
        self.arrived: list[str] = []
        self.on_arrival = lambda _message: None
        self._parser = MAVLink(None)
        self._parser.robust_parsing = True
        self.mav = MAVLink(self, srcSystem=255, srcComponent=190)

    def lose_a_byte(self) -> None:
        self.swallowing = MAVLINK2_MAX_FRAME_BYTES

    def write(self, buf: bytes) -> None:
        data = bytes(buf)
        cut = min(self.swallowing, len(data))
        self.swallowing -= cut
        for message in self._parser.parse_buffer(data[cut:]) or []:
            self.arrived.append(message.get_type())
            self.on_arrival(message)
            self._answer(message)

    def _answer(self, message) -> None:
        kind = message.get_type()
        if kind == "MISSION_REQUEST_LIST":
            reply = MAVLink_mission_count_message(255, 190, self.total_items, 0)
            name = "MISSION_COUNT"
        elif kind == "MISSION_REQUEST_INT":
            reply = MAVLink_mission_item_int_message(255, 190, message.seq, 0, 16, 0, 1,
                                                     0, 0, 0, 0, 0, 0, 0.0, 0)
            name = "MISSION_ITEM_INT"
        elif kind == "PARAM_REQUEST_READ":
            reply = MAVLink_param_value_message(message.param_id.encode(), 2.0, 9, 1, 0)
            name = "PARAM_VALUE"
        else:
            return
        reply.get_srcSystem = lambda: TARGET
        self.messages.publish(name, reply, receipt_time_s=1.0)


def _radio(total_items: int = 3) -> tuple[HeldFrameRadio, MavTransport]:
    radio = HeldFrameRadio(MessageStore(MESSAGE_HISTORY_POLICY), total_items)
    return radio, MavTransport(SimpleNamespace(mav=radio.mav), threading.RLock())


def _downloader(radio: HeldFrameRadio, transport: MavTransport) -> MissionDownloader:
    return MissionDownloader(TARGET, transport, MissionStore(TARGET, radio.messages), radio.messages,
                             SimpleNamespace(value=MagicMock()))


def _request_frame_bytes(transport: MavTransport) -> int:
    before = transport.uplink_bytes()
    transport.call(lambda connection: connection.mav.mission_request_int_send(TARGET, 0, 1, 0))
    return transport.uplink_bytes() - before


def _hold_after_item(radio: HeldFrameRadio, seq: int) -> None:
    """A lost byte right after item ``seq``'s request arrived: the next requests vanish."""
    def arrival(message) -> None:
        if message.get_type() == "MISSION_REQUEST_INT" and message.seq == seq:
            radio.lose_a_byte()
            radio.on_arrival = lambda _message: None
    radio.on_arrival = arrival


def test_the_transport_counts_the_bytes_its_encoder_wrote():
    radio, transport = _radio()
    assert transport.uplink_bytes() == 0
    size = _request_frame_bytes(transport)
    assert 12 <= size < MAVLINK2_MAX_FRAME_BYTES // 12  # 12 requests cannot fill a held frame
    assert transport.uplink_bytes() == size


def test_a_transport_without_a_byte_count_reports_none():
    transport = MavTransport(SimpleNamespace(mav=MagicMock()), threading.RLock())
    assert transport.uplink_bytes() is None


def test_a_held_frame_swallows_twelve_counted_requests():
    """The mechanism of the ``stress`` seed 40 failure: with every send counted, one lost
    byte fails the item, though every request after the held frame would arrive."""
    radio, transport = _radio()
    _hold_after_item(radio, 1)
    result = _downloader(radio, transport).download_outcome(policy=EVERY_SEND_COUNTS,
                                                            item_timeout=0.001)
    assert result.failed
    assert radio.arrived.count("MISSION_REQUEST_INT") == 2  # items 0 and 1; item 2's twelve vanished
    assert radio.swallowing > 0  # the held frame was still incomplete when the budget ran out


def test_sends_inside_a_held_frame_do_not_use_up_the_budget():
    radio, transport = _radio()
    _hold_after_item(radio, 1)
    result = _downloader(radio, transport).download_outcome(policy=GCS, item_timeout=0.001)
    assert result.count == 3
    assert radio.arrived.count("MISSION_REQUEST_INT") == 3  # item 2's first request past the held frame


def test_free_sends_are_bounded_by_one_held_frame():
    """An autopilot that never answers still ends the exchange: 12 counted sends after at
    most one held frame's worth of free ones."""
    radio, transport = _radio()
    radio._answer = lambda _message: None
    result = _downloader(radio, transport).download_outcome(policy=GCS, ack_timeout=0.001)
    assert result.failed
    sends = radio.arrived.count("MISSION_REQUEST_LIST")
    size = transport.uplink_bytes() // sends
    assert transport.uplink_bytes() == sends * size  # every send one count request
    assert sends == 12 + 1 + math.ceil(MAVLINK2_MAX_FRAME_BYTES / size)  # the first send, then one held frame


def test_a_parameter_read_outlasts_a_held_frame():
    radio, transport = _radio()
    radio.lose_a_byte()
    reader = ParameterReader(SimpleNamespace(target_system=TARGET), transport, ParameterRepository(),
                             radio.messages, SimpleNamespace(value=MagicMock()), None)
    exchange = reader.read("TEST", GCS, timeout=0.001)
    assert exchange.outcome is ExchangeOutcome.CONFIRMED
    assert radio.arrived == ["PARAM_REQUEST_READ"]
    assert exchange.attempts > 12 // 2  # several requests vanished before it arrived

    radio.lose_a_byte()
    exchange = reader.read("TEST", EVERY_SEND_COUNTS, timeout=0.001)
    assert (exchange.outcome, exchange.reason) == (ExchangeOutcome.UNCONFIRMED, ExchangeReason.NO_ECHO)


class Uplink:
    """A byte counter for :class:`SendBudget`: each send adds its frame, others' traffic adds more."""

    def __init__(self) -> None:
        self.written = 0

    def __call__(self) -> int:
        return self.written

    def send(self, budget: SendBudget, size: int, *, others_before: int = 0) -> None:
        """One send as the loop makes it: the reading, others' bytes in the gap, the frame, the record."""
        reading = budget.reading()
        self.written += others_before + size
        budget.record(reading)


def test_sends_count_once_the_uplink_carried_one_held_frame_after_the_first_send():
    uplink = Uplink()
    uplink.written = 50  # traffic before the exchange is not inside its allowance
    budget = SendBudget(GCS, uplink)
    uplink.send(budget, 15)                  # the first send: free; the allowance starts at 65
    while uplink.written - 65 < MAVLINK2_MAX_FRAME_BYTES:
        uplink.send(budget, 15)
        assert budget.counted == 0
    uplink.send(budget, 15)                  # starts 285 bytes after the first send ended
    assert (budget.sent, budget.counted) == (21, 1)


def test_other_senders_bytes_before_the_first_send_do_not_use_up_the_allowance():
    """Codex review: the reading precedes the send lock, so another sender can write in the
    gap. 280 bytes there must not count as uplink carried after the first send."""
    uplink = Uplink()
    budget = SendBudget(GCS, uplink)
    uplink.send(budget, 15, others_before=MAVLINK2_MAX_FRAME_BYTES)
    uplink.send(budget, 15)
    assert (budget.sent, budget.counted) == (2, 0)  # still inside one held frame after the first send


def test_without_a_byte_count_every_send_counts():
    budget = SendBudget(GCS, lambda: None)
    for _ in range(3):
        budget.record(budget.reading())
    assert (budget.sent, budget.counted) == (3, 3)


def test_a_policy_without_held_frame_bytes_counts_every_send():
    uplink = Uplink()
    budget = SendBudget(EVERY_SEND_COUNTS, uplink)
    for _ in range(12):
        uplink.send(budget, 15)
    assert budget.spent and budget.label() == "12/12"


def test_the_log_names_free_sends_only_once_they_extend_the_budget():
    uplink = Uplink()
    budget = SendBudget(ExchangePolicy(attempts=2, held_frame_bytes=MAVLINK2_MAX_FRAME_BYTES), uplink)
    labels = []
    for size in (15, 15, 265, 15, 15):
        uplink.send(budget, size)
        labels.append(budget.label())
    # The allowance starts at 15 (after send 1); readings before sends 2-5: 15, 30, 295, 310.
    assert labels == ["1/2", "2/2", "3/2 (+3 free)", "4/2 (+3 free)", "5/2 (+3 free)"]
    assert budget.spent


def test_a_send_the_deadline_refused_is_not_recorded():
    """``send()`` returns False when the deadline passed before the send lock came:
    nothing went out, so the budget and the outcome count only the sends made."""
    radio, transport = _radio()
    radio._answer = lambda _message: None
    outcomes = iter([True, True, False])
    io = ExchangeIo(
        "PARAM_REQUEST_READ TEST",
        lambda: next(outcomes) and transport.call(
            lambda connection: connection.mav.param_request_read_send(TARGET, 0, b"TEST", -1)) is None,
        lambda _cursor, _deadline: None,
        transport.uplink_bytes,
    )
    exchange, sample = run_exchange(GCS, None, radio.messages, io, 0.001)
    assert sample is None
    assert (exchange.outcome, exchange.reason, exchange.attempts) == (
        ExchangeOutcome.UNCONFIRMED, ExchangeReason.DEADLINE, 2)
    assert radio.arrived == ["PARAM_REQUEST_READ"] * 2
