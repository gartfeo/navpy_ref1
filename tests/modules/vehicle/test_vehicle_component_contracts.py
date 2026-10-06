"""Characterization tests for the VehicleMav ownership boundaries.

These tests pin synchronization and compatibility behavior before the
monolithic VehicleMav implementation is replaced by focused collaborators.
"""
from __future__ import annotations

import inspect
import threading
import time
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import pytest

from navpy.exception_groups import BaseExceptionGroup, ExceptionGroup
from navpy.modules.common.models.location import Location
from navpy.modules.navigation.mission_approach_writer import MissionApproachWriter
from navpy.modules.navigation.peer_poi_mission_writer import PeerPoiMissionWriter
from navpy.modules.vehicle.mav_mission import MavMission
from navpy.modules.vehicle.mav_bus import MavBus
from navpy.modules.vehicle.message_store import (
    MESSAGE_HISTORY_LIMIT,
    MessageHistoryOverrun,
    MessageStore,
)
from navpy.modules.vehicle.message_subscriptions import CallbackRegistry
from navpy.modules.vehicle.mav_transport import MavTransport
from navpy.modules.vehicle.mission_store import MissionStore
from navpy.modules.vehicle.mission_inbox import MissionInbox
from navpy.modules.vehicle.mission_protocol import MissionDownloader, MissionUploader
from navpy.modules.vehicle.parameter_client import ParameterReader
from navpy.modules.vehicle.parameter_repository import ParameterRepository
from navpy.modules.vehicle.vehicle_mav import VehicleMav
from navpy.modules.vehicle.vehicle_composition import build_vehicle_parts
from navpy.modules.vehicle.heartbeat_runtime import HeartbeatRuntime
from navpy.modules.vehicle.link_state import HeartbeatState
from navpy.modules.vehicle.vehicle_lifecycle import VehicleLifecycle


def test_message_store_wait_after_rejects_pre_request_sample():
    store = MessageStore()
    stale = object()
    store.publish("MISSION_COUNT", stale, receipt_time_s=10.0)
    cursor = store.cursor("MISSION_COUNT")

    assert store.wait_after(
        "MISSION_COUNT",
        cursor,
        lambda _message: True,
        deadline=time.monotonic() + 0.01,
    ) is None

    fresh = object()
    store.publish("MISSION_COUNT", fresh, receipt_time_s=11.0)
    sample = store.wait_after(
        "MISSION_COUNT",
        cursor,
        lambda _message: True,
        deadline=time.monotonic() + 0.1,
    )
    assert sample is not None
    assert sample.message is fresh


def test_message_store_retains_matching_same_type_burst_for_concurrent_waiters():
    store = MessageStore()
    cursor = store.cursor("PARAM_VALUE")
    requested_a = SimpleNamespace(param_id="A")
    requested_b = SimpleNamespace(param_id="B")
    later_noise = SimpleNamespace(param_id="NOISE")
    store.publish("PARAM_VALUE", requested_a, receipt_time_s=1.0)
    store.publish("PARAM_VALUE", requested_b, receipt_time_s=2.0)
    store.publish("PARAM_VALUE", later_noise, receipt_time_s=3.0)
    results = {}

    def wait_for(name):
        results[name] = store.wait_after(
            "PARAM_VALUE",
            cursor,
            lambda message: message.param_id == name,
            deadline=time.monotonic() + 0.5,
        )

    waiters = [
        threading.Thread(target=wait_for, args=(name,), daemon=True)
        for name in ("A", "B")
    ]
    for waiter in waiters:
        waiter.start()
    for waiter in waiters:
        waiter.join(timeout=0.5)

    assert all(not waiter.is_alive() for waiter in waiters)
    assert results["A"].message is requested_a
    assert results["B"].message is requested_b


def test_message_store_reports_cursor_overrun_instead_of_false_timeout():
    store = MessageStore()
    cursor = store.cursor("PARAM_VALUE")
    wanted = SimpleNamespace(param_id="WANTED")
    store.publish("PARAM_VALUE", wanted, receipt_time_s=1.0)
    for index in range(MESSAGE_HISTORY_LIMIT):
        store.publish(
            "PARAM_VALUE",
            SimpleNamespace(param_id=f"NOISE_{index}"),
            receipt_time_s=2.0 + index,
        )

    with pytest.raises(MessageHistoryOverrun) as raised:
        store.wait_after(
            "PARAM_VALUE",
            cursor,
            lambda message: message.param_id == "WANTED",
            deadline=time.monotonic() + 0.01,
        )

    assert raised.value.cursor == cursor
    assert raised.value.message_types == ("PARAM_VALUE",)


def test_message_store_retains_match_at_exact_history_capacity():
    store = MessageStore()
    cursor = store.cursor("PARAM_VALUE")
    wanted = SimpleNamespace(param_id="WANTED")
    store.publish("PARAM_VALUE", wanted, receipt_time_s=1.0)
    for index in range(MESSAGE_HISTORY_LIMIT - 1):
        store.publish(
            "PARAM_VALUE",
            SimpleNamespace(param_id=f"NOISE_{index}"),
            receipt_time_s=2.0 + index,
        )

    sample = store.wait_after(
        "PARAM_VALUE",
        cursor,
        lambda message: message.param_id == "WANTED",
        deadline=time.monotonic() + 0.01,
    )

    assert sample is not None
    assert sample.message is wanted


def test_message_store_does_not_report_overrun_before_first_eviction():
    store = MessageStore()
    cursor = store.cursor("PARAM_VALUE")
    for index in range(MESSAGE_HISTORY_LIMIT):
        store.publish(
            "PARAM_VALUE",
            SimpleNamespace(param_id=f"NOISE_{index}"),
            receipt_time_s=1.0 + index,
        )

    assert store.wait_after(
        "PARAM_VALUE",
        cursor,
        lambda message: message.param_id == "WANTED",
        deadline=time.monotonic() + 0.01,
    ) is None


def test_mission_store_replacement_swaps_only_after_full_build():
    class Loader:
        def __init__(self, items=(), fail_on=None):
            self.items = list(items)
            self.fail_on = fail_on

        def add(self, item):
            if item == self.fail_on:
                raise RuntimeError("replacement add failed")
            self.items.append(item)

        def clear(self):
            self.items.clear()

        def count(self):
            return len(self.items)

        def wp(self, index):
            return self.items[index]

    original = Loader(["old"])
    store = MissionStore(
        7,
        MessageStore(),
        loader=original,
        loader_factory=lambda: Loader(fail_on="bad"),
    )

    with pytest.raises(RuntimeError, match="replacement add failed"):
        store.replace_items(["new", "bad"])

    assert store.snapshot() == ["old"]


def test_composition_is_fully_initialized_before_bus_attach_dispatch():
    connection = SimpleNamespace(mav=MagicMock())
    bus = MagicMock(
        conn=connection,
        send_lock=threading.RLock(),
        heartbeats=set(),
    )
    lease = MagicMock()
    bus.reserve.return_value = lease
    battery = MagicMock(battery_remaining=55)
    battery.get_type.return_value = "BATTERY_STATUS"
    battery.get_srcSystem.return_value = 7
    battery.get_srcComponent.return_value = 1
    battery.get_seq.return_value = 1
    battery.get_msgId.return_value = 147
    lease.attach.side_effect = lambda receiver: receiver.feed_message(battery)

    parts = build_vehicle_parts(
        "unused",
        7,
        115200,
        MagicMock(),
        True,
        False,
        False,
        1.0,
        3.0,
        18,
        191,
        bus,
    )
    try:
        assert parts.power_gps.battery_level == 55
    finally:
        parts.runtime.lifetime.close()


def test_composition_rolls_back_base_exception_without_masking_it():
    bus = MagicMock(conn=SimpleNamespace(mav=MagicMock()))
    bus.send_lock = threading.RLock()
    lease = MagicMock()
    lease.release.side_effect = RuntimeError("release failed")
    bus.reserve.return_value = lease

    with patch(
        "navpy.modules.vehicle.vehicle_build_transaction.build_protocol_parts",
        side_effect=KeyboardInterrupt("build interrupted"),
    ):
        with pytest.raises(BaseExceptionGroup) as caught:
            build_vehicle_parts(
                "unused", 7, 115200, MagicMock(), True, False, False,
                1.0, 3.0, 18, 191, bus,
            )

    lease.release.assert_called_once_with()
    assert [type(error) for error in caught.value.exceptions] == [
        KeyboardInterrupt,
        RuntimeError,
    ]
    assert [str(error) for error in caught.value.exceptions] == [
        "build interrupted",
        "release failed",
    ]


def test_bus_reservation_survives_first_builder_failure_until_second_attaches():
    connection = MagicMock()
    connection.recv_match.side_effect = lambda **_kwargs: (
        time.sleep(0.005) or None
    )
    bus = MavBus(connection, "reservation-test")
    first_builder = bus.reserve(7)
    second_builder = bus.reserve(8)

    first_builder.release()
    assert not bus.is_closed
    connection.close.assert_not_called()

    receiver = MagicMock()
    second_builder.attach(receiver)
    assert not bus.is_closed
    second_builder.release()
    assert bus.is_closed
    connection.close.assert_called_once()


def test_heartbeat_touch_preserves_explicit_zero_receipt():
    heartbeat = HeartbeatState(target_system=7, timeout_s=1.0)

    heartbeat.touch(0.0)

    assert not heartbeat.link_ok


def test_heartbeat_close_is_bounded_retained_and_retryable():
    entered = threading.Event()
    release = threading.Event()

    class BlockingTransport:
        def call(self, action):
            entered.set()
            release.wait(timeout=1.0)
            return action(SimpleNamespace(mav=MagicMock()))

    runtime = HeartbeatRuntime(
        BlockingTransport(),
        SimpleNamespace(mav_type=18, target_system=7),
        HeartbeatState(7, 3.0),
        SimpleNamespace(value=MagicMock()),
        1.0,
    )
    runtime.start()
    assert entered.wait(timeout=0.5)
    with patch(
        "navpy.modules.vehicle.heartbeat_runtime.HEARTBEAT_JOIN_TIMEOUT_S",
        0.01,
    ):
        started_s = time.perf_counter()
        with pytest.raises(TimeoutError, match="vehicle remains owned"):
            runtime.close()
        assert time.perf_counter() - started_s < 0.2
    release.set()
    runtime.close()


def test_lifecycle_retries_only_failed_cleanup_steps():
    events: list[str] = []

    attempts: dict[str, int] = {}

    def fails_once(name):
        def _raise(*_args):
            events.append(name)
            attempts[name] = attempts.get(name, 0) + 1
            if attempts[name] == 1:
                raise RuntimeError(name)
        return _raise

    heartbeat_runtime = MagicMock()
    heartbeat_runtime.close.side_effect = fails_once("heartbeat")
    first = MagicMock()
    first.close.side_effect = fails_once("first")
    second = MagicMock()
    second.close.side_effect = lambda: events.append("second")
    bus = MagicMock(heartbeats={})
    lease = MagicMock()
    lease.release.side_effect = fails_once("lease")
    lifecycle = VehicleLifecycle(
        bus,
        lease,
        7,
        heartbeat_runtime,
        HeartbeatState(7, 3.0),
        SimpleNamespace(value=MagicMock()),
        closers=(first, second),
    )
    lifecycle.attach(object())

    with pytest.raises(RuntimeError, match="heartbeat"):
        lifecycle.close()
    assert events == ["heartbeat"]

    with pytest.raises(ExceptionGroup) as caught:
        lifecycle.close()
    assert [str(error) for error in caught.value.exceptions] == [
        "first",
        "lease",
    ]
    assert events == [
        "heartbeat", "heartbeat", "first", "second", "lease",
    ]

    lifecycle.close()
    assert events == [
        "heartbeat", "heartbeat", "first", "second", "lease",
        "first", "lease",
    ]

    lifecycle.close()
    assert len(events) == 7


def test_lifecycle_base_exception_does_not_skip_remaining_cleanup():
    heartbeat_runtime = MagicMock()
    heartbeat_runtime.close.side_effect = KeyboardInterrupt("cancelled")
    closer = MagicMock()
    lease = MagicMock()
    lifecycle = VehicleLifecycle(
        MagicMock(heartbeats={}),
        lease,
        7,
        heartbeat_runtime,
        HeartbeatState(7, 3.0),
        SimpleNamespace(value=MagicMock()),
        closers=(closer,),
    )

    with pytest.raises(KeyboardInterrupt, match="cancelled"):
        lifecycle.close()

    closer.close.assert_not_called()
    lease.release.assert_not_called()


def test_lifecycle_serializes_heartbeat_start_before_close():
    start_entered = threading.Event()
    release_start = threading.Event()
    events: list[str] = []
    heartbeat_runtime = MagicMock()

    def blocking_start():
        events.append("start")
        start_entered.set()
        release_start.wait(timeout=1.0)

    heartbeat_runtime.start.side_effect = blocking_start
    heartbeat_runtime.close.side_effect = lambda: events.append("close")
    lease = MagicMock()
    lease.release.side_effect = lambda: events.append("release")
    lifecycle = VehicleLifecycle(
        MagicMock(heartbeats={}),
        lease,
        7,
        heartbeat_runtime,
        HeartbeatState(7, 3.0),
        SimpleNamespace(value=MagicMock()),
    )

    starter = threading.Thread(target=lifecycle.start_heartbeat, daemon=True)
    closer = threading.Thread(target=lifecycle.close, daemon=True)
    starter.start()
    assert start_entered.wait(timeout=0.5)
    closer.start()
    closer.join(timeout=0.05)

    assert closer.is_alive()
    heartbeat_runtime.close.assert_not_called()
    lease.release.assert_not_called()

    release_start.set()
    starter.join(timeout=0.5)
    closer.join(timeout=0.5)
    assert not starter.is_alive()
    assert not closer.is_alive()
    assert events == ["start", "close", "release"]


def test_lifecycle_serializes_bus_attachment_before_close():
    attach_entered = threading.Event()
    release_attach = threading.Event()
    events: list[str] = []
    lease = MagicMock()

    def blocking_attach(_receiver):
        events.append("attach")
        attach_entered.set()
        release_attach.wait(timeout=1.0)

    lease.attach.side_effect = blocking_attach
    lease.release.side_effect = lambda: events.append("release")
    lifecycle = VehicleLifecycle(
        MagicMock(heartbeats={}),
        lease,
        7,
        MagicMock(),
        HeartbeatState(7, 3.0),
        SimpleNamespace(value=MagicMock()),
    )

    attacher = threading.Thread(
        target=lambda: lifecycle.attach(object()),
        daemon=True,
    )
    closer = threading.Thread(target=lifecycle.close, daemon=True)
    attacher.start()
    assert attach_entered.wait(timeout=0.5)
    closer.start()
    closer.join(timeout=0.05)

    assert closer.is_alive()
    lease.release.assert_not_called()

    release_attach.set()
    attacher.join(timeout=0.5)
    closer.join(timeout=0.5)
    assert not attacher.is_alive()
    assert not closer.is_alive()
    assert events == ["attach", "release"]


def test_message_store_publishes_one_atomic_sample():
    store = MessageStore()
    message = object()
    published = store.publish(
        "SIM_STATE",
        message,
        receipt_time_s=123.5,
        boot_time_ms=456,
    )

    sample = store.latest("SIM_STATE")
    assert sample is published
    assert (
        sample.message,
        sample.receipt_time_s,
        sample.boot_time_ms,
        sample.generation,
    ) == (message, 123.5, 456, 1)


def test_mission_download_rejects_count_older_than_request():
    messages = MessageStore()
    stale = MagicMock(count=9)
    stale.get_srcSystem.return_value = 7
    stale.mission_type = 0
    messages.publish("MISSION_COUNT", stale, receipt_time_s=1.0)
    fresh = MagicMock(count=0)
    fresh.get_srcSystem.return_value = 7
    fresh.mission_type = 0
    connection = SimpleNamespace(mav=MagicMock())

    def request_list(*_args):
        messages.publish("MISSION_COUNT", fresh, receipt_time_s=2.0)

    connection.mav.mission_request_list_send.side_effect = request_list
    store = MagicMock()
    logger_ref = SimpleNamespace(value=MagicMock())
    downloader = MissionDownloader(
        7,
        MavTransport(connection, threading.RLock()),
        store,
        messages,
        logger_ref,
    )

    assert downloader.download(timeout=0.1, retries=1) == 0
    store.replace_items.assert_called_once_with([])


def test_mission_download_partial_timeout_preserves_prior_store():
    class RecordingStore:
        def __init__(self, items):
            self.items = list(items)

        def replace_items(self, items):
            self.items = list(items)

    messages = MessageStore()
    old_item = object()
    store = RecordingStore([old_item])
    count = MagicMock(count=2, mission_type=0)
    count.get_srcSystem.return_value = 7
    first = MagicMock(seq=0, mission_type=0)
    first.get_srcSystem.return_value = 7
    connection = SimpleNamespace(mav=MagicMock())
    connection.mav.mission_request_list_send.side_effect = lambda *_args: (
        messages.publish("MISSION_COUNT", count, receipt_time_s=1.0)
    )

    def request_item(_target, _component, sequence, _mission_type):
        if sequence == 0:
            messages.publish("MISSION_ITEM_INT", first, receipt_time_s=2.0)

    connection.mav.mission_request_int_send.side_effect = request_item
    downloader = MissionDownloader(
        7,
        MavTransport(connection, threading.RLock()),
        store,
        messages,
        SimpleNamespace(value=MagicMock()),
    )

    assert downloader.download(timeout=0.01, retries=1) == 0
    assert store.items == [old_item]


def test_mission_download_valid_empty_count_clears_prior_store():
    class RecordingStore:
        def __init__(self, items):
            self.items = list(items)

        def replace_items(self, items):
            self.items = list(items)

    messages = MessageStore()
    store = RecordingStore([object()])
    count = MagicMock(count=0, mission_type=0)
    count.get_srcSystem.return_value = 7
    connection = SimpleNamespace(mav=MagicMock())
    connection.mav.mission_request_list_send.side_effect = lambda *_args: (
        messages.publish("MISSION_COUNT", count, receipt_time_s=1.0)
    )
    downloader = MissionDownloader(
        7,
        MavTransport(connection, threading.RLock()),
        store,
        messages,
        SimpleNamespace(value=MagicMock()),
    )

    assert downloader.download(timeout=0.1, retries=1) == 0
    assert store.items == []


def test_parameter_read_rejects_value_older_than_request():
    messages = MessageStore()
    stale = MagicMock(param_id="TEST", param_value=1.0)
    stale.get_srcSystem.return_value = 7
    messages.publish("PARAM_VALUE", stale, receipt_time_s=1.0)
    fresh = MagicMock(param_id="TEST", param_value=2.0)
    fresh.get_srcSystem.return_value = 7
    connection = SimpleNamespace(mav=MagicMock())
    connection.mav.param_request_read_send.side_effect = lambda *_args: (
        messages.publish("PARAM_VALUE", fresh, receipt_time_s=2.0)
    )
    reader = ParameterReader(
        SimpleNamespace(target_system=7),
        MavTransport(connection, threading.RLock()),
        ParameterRepository(),
        messages,
        SimpleNamespace(value=MagicMock()),
    )

    assert reader.get("TEST", timeout=0.1, retries=1) == 2.0


def test_mission_upload_accepts_next_request_published_during_item_send():
    inbox = MissionInbox()
    connection = SimpleNamespace(mav=MagicMock())
    waypoints = [MagicMock(), MagicMock()]
    store = MagicMock()
    store.snapshot.return_value = waypoints

    def message(message_type, **fields):
        item = MagicMock(**fields)
        item.get_type.return_value = message_type
        item.get_srcSystem.return_value = 7
        item.mission_type = 0
        return item

    connection.mav.mission_count_send.side_effect = lambda *_args: inbox.publish(
        message("MISSION_REQUEST_INT", seq=0), time.time(),
    )

    def send_item(_target, _component, sequence, *_args):
        if sequence == 0:
            inbox.publish(
                message("MISSION_REQUEST_INT", seq=1), time.time(),
            )
        else:
            inbox.publish(
                message("MISSION_ACK", type=0), time.time(),
            )

    connection.mav.mission_item_int_send.side_effect = send_item
    uploader = MissionUploader(
        7,
        MavTransport(connection, threading.RLock()),
        store,
        inbox,
        SimpleNamespace(value=MagicMock()),
    )

    assert uploader.upload(timeout=0.1, retries=1)
    assert connection.mav.mission_item_int_send.call_count == 2


def test_callback_registry_order_and_idempotent_self_cancellation():
    registry = CallbackRegistry()
    seen: list[str] = []
    subscription = None

    def exact(_message):
        seen.append("exact")
        subscription.cancel()
        subscription.cancel()

    subscription = registry.subscribe("ATTITUDE", exact)
    registry.subscribe("*", lambda _message: seen.append("wildcard"))
    registry.subscribe("NAVLINK", lambda _message: seen.append("navlink"))

    worker = threading.Thread(
        target=lambda: registry.dispatch(
            "ATTITUDE", object(), is_navlink=True,
        ),
        daemon=True,
    )
    worker.start()
    worker.join(timeout=0.5)

    assert not worker.is_alive(), "callback ran while registry lock was held"
    assert seen == ["exact", "wildcard", "navlink"]

    registry.dispatch("ATTITUDE", object(), is_navlink=False)
    assert seen == ["exact", "wildcard", "navlink", "wildcard"]


def test_mav_transport_restores_exact_source_component_state():
    lock = threading.RLock()
    observed: list[tuple[object, bool]] = []

    class Encoder:
        srcComponent = None

        def send(self, message):
            observed.append((self.srcComponent, lock._is_owned()))

    connection = SimpleNamespace(mav=Encoder())
    transport = MavTransport(connection, lock)

    transport.send(object(), source_component=100)

    assert observed == [(100, True)]
    assert connection.mav.srcComponent is None


def test_mav_transport_removes_temporary_source_component_when_originally_absent():
    lock = threading.RLock()

    class Encoder:
        def send(self, _message):
            assert self.srcComponent == 101
            assert lock._is_owned()

    connection = SimpleNamespace(mav=Encoder())
    transport = MavTransport(connection, lock)

    transport.send(object(), source_component=101)

    assert not hasattr(connection.mav, "srcComponent")


@pytest.mark.parametrize("originally_present", [False, True])
def test_mav_transport_restores_source_component_when_send_raises(
    originally_present,
):
    lock = threading.RLock()

    class Encoder:
        def send(self, _message):
            assert self.srcComponent == 102
            assert lock._is_owned()
            raise RuntimeError("encode failed")

    encoder = Encoder()
    if originally_present:
        encoder.srcComponent = None
    connection = SimpleNamespace(mav=encoder)
    transport = MavTransport(connection, lock)

    with pytest.raises(RuntimeError, match="encode failed"):
        transport.send(object(), source_component=102)

    if originally_present:
        assert connection.mav.srcComponent is None
    else:
        assert not hasattr(connection.mav, "srcComponent")


def test_vehicle_and_mission_update_contract_is_sequence_first():
    vehicle_parameters = tuple(
        inspect.signature(VehicleMav.update_mission_item).parameters
    )
    mission_parameters = tuple(
        inspect.signature(MavMission.update_item).parameters
    )

    assert vehicle_parameters == ("self", "sequence", "command")
    assert mission_parameters == ("self", "sequence", "command")

    mission_store = MagicMock()
    mission = MavMission(
        7,
        MagicMock(),
        SimpleNamespace(value=MagicMock()),
        MessageStore(),
        MissionInbox(),
        store=mission_store,
        downloader=MagicMock(),
        uploader=MagicMock(),
    )
    command = MagicMock()
    mission.update_item(7, command)
    mission_store.update.assert_called_once_with(7, command)

    loader = MagicMock()
    store = MissionStore(7, MessageStore(), loader=loader)
    store.update(7, command)
    loader.set.assert_called_once_with(command, 7)


def test_vehicle_adapter_delegates_sequence_first_without_reordering():
    mission_gateway = MagicMock()
    with patch(
        "navpy.modules.vehicle.vehicle_mav.build_vehicle_parts",
        return_value=SimpleNamespace(mission=mission_gateway),
    ):
        vehicle = VehicleMav(
            "unused",
            8,
            wait_heartbeat=False,
            send_heartbeat=False,
            skip_mission_download=True,
        )
    command = MagicMock()

    vehicle.update_mission_item(8, command)

    mission_gateway.update_item.assert_called_once_with(8, command)


def test_mission_approach_writer_calls_sequence_first():
    vehicle = MagicMock()
    commands = [MagicMock() for _ in range(4)]

    MissionApproachWriter._upload(vehicle, commands, restart_index=2)

    assert vehicle.update_mission_item.call_args_list == [
        ((index, command), {}) for index, command in enumerate(commands)
    ]


def test_peer_poi_writer_calls_sequence_first():
    vehicle = MagicMock()
    vehicle.target_system = 7
    vehicle.location.return_value = Location(40.0, 44.0, 100.0, False)
    vehicle.home_location = Location(40.0, 44.0, 900.0, True)
    poi = Location(40.1, 44.1, 1000.0, True)

    PeerPoiMissionWriter().plan(vehicle, poi)

    calls = vehicle.update_mission_item.call_args_list
    assert [call.args[0] for call in calls] == [0, 1, 2, 3]
    assert all(call.args[1] is not None for call in calls)
