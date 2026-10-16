"""Reader batch budget and pymavlink batching (docs/design/reader_batch.md)."""
from __future__ import annotations

import socket
import threading
import time
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import pytest
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink

from navpy.app.wall_scheduler_cadence import speedup_scheduler_cadence
from navpy.plugins.vehicle.ardupilot.mav_bus import MavBus
from navpy.plugins.vehicle.ardupilot.mav_bus_reader_budget import (
    DEFAULT_READER_BATCH_BUDGET_S,
    ReaderBudgetRegistry,
)


# -- budget registry ----------------------------------------------------------

def test_no_request_keeps_the_default_budget():
    registry = ReaderBudgetRegistry()
    assert registry.current_s() == DEFAULT_READER_BATCH_BUDGET_S
    registry.request("unset")  # a request that has not set a slot yet
    assert registry.current_s() == DEFAULT_READER_BATCH_BUDGET_S


def test_shared_bus_minimum_wins_and_release_restores_the_next():
    registry = ReaderBudgetRegistry()
    slow = registry.request("1x vehicle")
    fast = registry.request("10x vehicle")
    slow.set(0.020)
    fast.set(0.002)
    assert registry.current_s() == pytest.approx(0.002)

    fast.release()
    assert registry.current_s() == pytest.approx(0.020)
    slow.release()
    assert registry.current_s() == DEFAULT_READER_BATCH_BUDGET_S


def test_a_released_budget_cannot_be_set_and_bad_slots_are_rejected():
    registry = ReaderBudgetRegistry()
    handle = registry.request("cadence")
    for bad in (0.0, -1.0, float("nan"), float("inf")):
        with pytest.raises(ValueError):
            handle.set(bad)
    handle.release()
    handle.release()  # idempotent
    with pytest.raises(RuntimeError, match="after release"):
        handle.set(0.002)
    assert registry.current_s() == DEFAULT_READER_BATCH_BUDGET_S


def test_mav_bus_reader_is_bounded_by_its_budget_requests():
    captured = {}

    class _Reader:
        def __init__(self, *args, **kwargs):
            captured.update(kwargs)

        def start(self):
            pass

    with patch("navpy.plugins.vehicle.ardupilot.mav_bus.MavBusReader", _Reader):
        bus = MavBus(MagicMock(), "test:reader-budget")
    budget = captured["batch_budget_s"]
    assert budget() == DEFAULT_READER_BATCH_BUDGET_S
    first = bus.reader_budgets.request("a")
    second = bus.reader_budgets.request("b")
    first.set(0.004)
    second.set(0.002)
    assert budget() == pytest.approx(0.002)
    second.release()
    assert budget() == pytest.approx(0.004)


# -- wall composition: the cadence feeds the vehicle's bus budget ---------------

def _vehicle(speedup, loop_rate_hz):
    """Only what ``speedup_scheduler_cadence`` reads of a vehicle."""
    return SimpleNamespace(
        sim_speedup=speedup,
        get_param_or_default=lambda name, default: (
            loop_rate_hz if name == "SCHED_LOOP_RATE" else default
        ),
    )


def test_wall_cadence_sets_one_scheduler_slot_and_releases_on_close():
    registry = ReaderBudgetRegistry()
    speed = {"value": 10.0}
    vehicle = _vehicle(lambda: speed["value"], 50.0)

    cadence = speedup_scheduler_cadence(vehicle, SimpleNamespace(reader_budgets=registry))
    try:
        # Period resolved before the first sync: 20 ms / 10x = 2 ms at once.
        assert registry.current_s() == pytest.approx(0.002)

        speed["value"] = 5.0
        cadence.wall_period_for_scheduler_period(0.02)  # non-blocking sync
        assert registry.current_s() == pytest.approx(0.004)

        speed["value"] = 2.0
        cadence.sync_speed()  # polled sync
        assert registry.current_s() == pytest.approx(0.010)
    finally:
        cadence.close()
    assert registry.current_s() == DEFAULT_READER_BATCH_BUDGET_S
    cadence.sync_speed()  # a sync after close no longer touches the bus
    assert registry.current_s() == DEFAULT_READER_BATCH_BUDGET_S


def test_hardware_at_1x_keeps_a_20ms_budget():
    registry = ReaderBudgetRegistry()
    cadence = speedup_scheduler_cadence(
        _vehicle(lambda: 1.0, 50.0), SimpleNamespace(reader_budgets=registry),
    )
    try:
        assert registry.current_s() == pytest.approx(0.020)
    finally:
        cadence.close()


# -- pymavlink batching on a real UDP socket ----------------------------------

def _packets(count):
    encoder = mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
    return [
        mavlink.MAVLink_attitude_message(i, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).pack(encoder)
        for i in range(count)
    ]


def _free_udp_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


def _dispatched_boot_times(datagrams, expected):
    port = _free_udp_port()
    connection = mavutil.mavlink_connection(f"udpin:127.0.0.1:{port}", source_system=255)
    bus = MavBus(connection, f"udpin:127.0.0.1:{port}")
    received = []
    done = threading.Event()

    def feed(message):
        received.append(message.time_boot_ms)
        if len(received) >= expected:
            done.set()

    bus.attach(1, SimpleNamespace(feed_message=feed))
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        for datagram in datagrams:
            sender.sendto(datagram, ("127.0.0.1", port))
            time.sleep(0.05)  # let the reader see each datagram on its own
        assert done.wait(5.0)
        time.sleep(0.1)  # nothing dispatched twice afterwards
        return received
    finally:
        sender.close()
        bus.close()


def test_a_datagram_with_several_packets_dispatches_each_once():
    packets = _packets(4)
    assert _dispatched_boot_times([b"".join(packets)], 4) == [0, 1, 2, 3]


def test_a_frame_split_across_datagrams_dispatches_once():
    first, second, third = _packets(3)
    half = len(second) // 2
    datagrams = [first + second[:half], second[half:] + third]
    assert _dispatched_boot_times(datagrams, 3) == [0, 1, 2]
