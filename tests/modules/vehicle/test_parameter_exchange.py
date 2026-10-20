"""Bounded parameter exchanges (``docs/design/radio_budget.md``, todo 103): the attempt
budget, the link-silence end and the request deadline."""
from __future__ import annotations

import threading
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

from pymavlink.dialects.v20.ardupilotmega import (
    MAV_COMP_ID_ONBOARD_COMPUTER,
    MAV_TYPE_GCS,
)

from navpy.core.ports.parameter_exchange import ExchangeOutcome, ExchangePolicy, ExchangeReason
from navpy.plugins.vehicle.ardupilot.inbound_router import InboundMessageRouter
from navpy.plugins.vehicle.ardupilot.link_state import HeartbeatState, PacketLossTracker
from navpy.plugins.vehicle.ardupilot.mav_transport import MavTransport
from navpy.plugins.vehicle.ardupilot.message_store import MessageStore
from navpy.plugins.vehicle.ardupilot.message_subscriptions import CallbackRegistry
from navpy.plugins.vehicle.ardupilot.mission_inbox import MissionInbox
from navpy.plugins.vehicle.ardupilot.parameter_reader import ParameterReader
from navpy.plugins.vehicle.ardupilot.parameter_repository import ParameterRepository
from navpy.plugins.vehicle.ardupilot.parameter_writer import ParameterWriter
from navpy.plugins.vehicle.ardupilot.vehicle_foundation_composition import MESSAGE_HISTORY_POLICY
from navpy.plugins.vehicle.ardupilot.vehicle_identity import VehicleIdentity

GCS = ExchangePolicy(attempts=12, stop_on_silence=True)


class Clock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now


def _param_value(name: str, value: float) -> MagicMock:
    message = MagicMock(param_id=name, param_value=value, param_type=9)
    message.get_srcSystem.return_value = 7
    return message


def _writer(link: HeartbeatState, on_send=None):
    messages = MessageStore(MESSAGE_HISTORY_POLICY)
    connection = SimpleNamespace(mav=MagicMock())
    if on_send is not None:
        connection.mav.param_set_send.side_effect = lambda *args: on_send(messages, args)
    writer = ParameterWriter(SimpleNamespace(target_system=7), MavTransport(connection, threading.RLock()),
                             ParameterRepository(), messages, SimpleNamespace(value=MagicMock()),
                             MagicMock(), link)
    return writer, connection


def test_frames_without_echoes_use_the_whole_budget_and_end_unconfirmed():
    link = HeartbeatState(7, 5.0)
    writer, connection = _writer(link)
    exchange = writer.write("TEST", 2.0, GCS, timeout=0.001)
    assert (exchange.outcome, exchange.reason, exchange.attempts) == (
        ExchangeOutcome.UNCONFIRMED, ExchangeReason.NO_ECHO, 12)
    assert connection.mav.param_set_send.call_count == 12


def test_a_stale_heartbeat_with_other_autopilot_frames_does_not_end_the_exchange():
    """Liveness is any autopilot frame: a lost heartbeat run alone is no disconnection."""
    clock = Clock()
    link = HeartbeatState(7, 5.0, clock)
    clock.now = 60.0  # no heartbeat for 60 s ...
    link.observe_frame()  # ... but the autopilot still talks
    assert not link.link_ok and not link.autopilot_silent

    def echo_on_the_ninth(messages, args):
        if connection.mav.param_set_send.call_count == 9:
            messages.publish("PARAM_VALUE", _param_value("TEST", 2.0), receipt_time_s=1.0)

    writer, connection = _writer(link, echo_on_the_ninth)
    exchange = writer.write("TEST", 2.0, GCS, timeout=0.001)
    assert exchange.confirmed and exchange.attempts == 9


def test_silence_ends_the_exchange_before_and_after_a_send():
    clock = Clock()
    link = HeartbeatState(7, 5.0, clock)
    clock.now = 5.1
    writer, connection = _writer(link)
    exchange = writer.write("TEST", 2.0, GCS, timeout=0.001)
    assert (exchange.outcome, exchange.reason, exchange.attempts) == (
        ExchangeOutcome.UNATTEMPTED, ExchangeReason.LINK_LOST, 0)
    assert connection.mav.param_set_send.call_count == 0

    link.observe_frame()

    def go_silent_after_two(_messages, _args):
        if connection.mav.param_set_send.call_count == 2:
            clock.now += 5.1

    writer, connection = _writer(link, go_silent_after_two)
    exchange = writer.write("TEST", 2.0, GCS, timeout=0.001)
    assert (exchange.outcome, exchange.reason, exchange.attempts) == (
        ExchangeOutcome.UNCONFIRMED, ExchangeReason.LINK_LOST, 2)


def test_the_default_policy_ignores_silence():
    clock = Clock()
    link = HeartbeatState(7, 5.0, clock)
    clock.now = 100.0
    writer, connection = _writer(link)
    assert writer.set("TEST", 2.0, timeout=0.001) is False
    assert connection.mav.param_set_send.call_count == 3


def test_the_deadline_bounds_the_lock_wait_and_every_echo_wait():
    link = HeartbeatState(7, 5.0)
    writer, connection = _writer(link)
    writer._write_lock.acquire()  # another write holds the vehicle
    try:
        started = time.monotonic()
        exchange = writer.write("TEST", 2.0, ExchangePolicy(12, time.monotonic() + 0.05, True))
    finally:
        writer._write_lock.release()
    assert (exchange.outcome, exchange.reason, exchange.attempts) == (
        ExchangeOutcome.UNATTEMPTED, ExchangeReason.DEADLINE, 0)
    assert time.monotonic() - started < 1.0

    started = time.monotonic()
    exchange = writer.write("TEST", 2.0, ExchangePolicy(12, time.monotonic() + 0.05, True), timeout=2.0)
    assert (exchange.outcome, exchange.reason, exchange.attempts) == (
        ExchangeOutcome.UNCONFIRMED, ExchangeReason.DEADLINE, 1)
    assert time.monotonic() - started < 1.0  # the 2 s echo wait was capped


def test_a_read_returns_its_value_and_marks_a_name_missing_only_after_the_whole_budget():
    link = HeartbeatState(7, 5.0)
    messages = MessageStore(MESSAGE_HISTORY_POLICY)
    connection = SimpleNamespace(mav=MagicMock())
    repository = ParameterRepository()
    reader = ParameterReader(SimpleNamespace(target_system=7), MavTransport(connection, threading.RLock()),
                             repository, messages, SimpleNamespace(value=MagicMock()), link)
    exchange = reader.read("NOPE", ExchangePolicy(12, time.monotonic() + 0.03, True), timeout=0.01)
    assert exchange.reason is ExchangeReason.DEADLINE and not repository.missing_is_fresh("NOPE")
    exchange = reader.read("NOPE", ExchangePolicy(2, None, True), timeout=0.001)
    assert exchange.reason is ExchangeReason.NO_ECHO and repository.missing_is_fresh("NOPE")
    messages.publish("PARAM_VALUE", _param_value("HERE", 3.0), receipt_time_s=1.0)  # before: stale
    connection.mav.param_request_read_send.side_effect = lambda *_args: messages.publish(
        "PARAM_VALUE", _param_value("HERE", 4.0), receipt_time_s=2.0)
    exchange = reader.read("HERE", GCS, timeout=0.001)
    assert exchange.confirmed and exchange.value == 4.0


def test_every_autopilot_frame_keeps_the_link_alive_but_peers_and_companions_do_not():
    clock = Clock()
    link = HeartbeatState(42, 5.0, clock)
    router = InboundMessageRouter(
        VehicleIdentity(42, 42, MAV_COMP_ID_ONBOARD_COMPUTER, MAV_TYPE_GCS),
        MessageStore(MESSAGE_HISTORY_POLICY), CallbackRegistry(), ParameterRepository(), link,
        PacketLossTracker(clock), MissionInbox(), SimpleNamespace(value=MagicMock()))

    def frame(system: int, component: int) -> MagicMock:
        message = MagicMock()
        message.get_msgId.return_value = 74
        message.get_srcSystem.return_value = system
        message.get_srcComponent.return_value = component
        message.get_seq.return_value = 0
        message.get_type.return_value = "VFR_HUD"
        return message

    clock.now = 5.1
    router.ingest(frame(43, 1))  # a peer's autopilot
    router.ingest(frame(42, MAV_COMP_ID_ONBOARD_COMPUTER))  # this vehicle's companion
    assert link.autopilot_silent
    router.ingest(frame(42, 1))
    assert not link.autopilot_silent and not link.link_ok  # a frame is not a heartbeat


def test_a_queued_write_never_confirms_on_an_echo_from_before_it_took_the_lock():
    """An earlier exchange's echo of the same value is not this write's answer."""
    writer, _connection = _writer(HeartbeatState(7, 5.0))
    lock, waiting = threading.Lock(), threading.Event()

    class Announcing:
        """The write lock, telling the test when the queued write asks for it."""

        def acquire(self, *args, **kwargs) -> bool:
            waiting.set()
            return lock.acquire(*args, **kwargs)

        def release(self) -> None:
            lock.release()

    writer._write_lock = Announcing()
    results = []
    lock.acquire()  # another write holds the vehicle
    queued = threading.Thread(target=lambda: results.append(
        writer.write("TEST", 2.0, ExchangePolicy(2, None, True), timeout=0.01)))
    queued.start()
    assert waiting.wait(timeout=5.0)  # the queued write has taken any early cursor
    writer._messages.publish("PARAM_VALUE", _param_value("TEST", 2.0), receipt_time_s=1.0)
    lock.release()
    queued.join(timeout=5.0)
    assert (results[0].outcome, results[0].reason) == (ExchangeOutcome.UNCONFIRMED, ExchangeReason.NO_ECHO)


def test_a_deadline_that_cuts_the_last_wait_is_a_deadline_not_a_missing_parameter():
    link = HeartbeatState(7, 5.0)
    writer, _connection = _writer(link)
    exchange = writer.write("TEST", 2.0, ExchangePolicy(1, time.monotonic() + 0.03, True), timeout=2.0)
    assert (exchange.outcome, exchange.reason, exchange.attempts) == (
        ExchangeOutcome.UNCONFIRMED, ExchangeReason.DEADLINE, 1)
    repository = ParameterRepository()
    reader = ParameterReader(SimpleNamespace(target_system=7),
                             MavTransport(SimpleNamespace(mav=MagicMock()), threading.RLock()),
                             repository, MessageStore(MESSAGE_HISTORY_POLICY),
                             SimpleNamespace(value=MagicMock()), link)
    exchange = reader.read("NOPE", ExchangePolicy(1, time.monotonic() + 0.03, True), timeout=2.0)
    assert exchange.reason is ExchangeReason.DEADLINE and not repository.missing_is_fresh("NOPE")


def test_a_gcs_exchange_logs_every_send_and_its_outcome():
    """The runs' attempt distribution comes from these lines (todo 103)."""
    link = HeartbeatState(7, 5.0)

    def echo_on_the_third(messages, _args):
        if connection.mav.param_set_send.call_count == 3:
            messages.publish("PARAM_VALUE", _param_value("TEST", 2.0), receipt_time_s=1.0)

    writer, connection = _writer(link, echo_on_the_third)
    writer.write("TEST", 2.0, GCS, timeout=0.001)
    lines = [c.args[0] for c in writer._logger_ref.value.info.call_args_list]
    assert lines == ["PARAM_SET TEST send 1/12: no echo", "PARAM_SET TEST send 2/12: no echo",
                     "PARAM_SET TEST send 3/12: echo", "PARAM_SET TEST: confirmed after 3/12 sends"]
