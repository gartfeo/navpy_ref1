"""Shared-system-id admission, heartbeat, and packet-loss boundaries."""
from __future__ import annotations

from types import SimpleNamespace
import threading
from unittest.mock import MagicMock, patch

from pymavlink.dialects.v20.ardupilotmega import (
    MAV_COMP_ID_ONBOARD_COMPUTER,
    MAV_MODE_FLAG_SAFETY_ARMED,
    MAV_TYPE_FIXED_WING,
    MAV_TYPE_ONBOARD_CONTROLLER,
)

from navpy.modules.vehicle.inbound_router import InboundMessageRouter
from navpy.modules.vehicle.link_state import HeartbeatState, PacketLossTracker
from navpy.modules.vehicle.message_store import MessageStore
from navpy.modules.vehicle.message_subscriptions import CallbackRegistry
from navpy.modules.vehicle.mission_inbox import MissionInbox
from navpy.modules.vehicle.parameter_repository import ParameterRepository
from navpy.modules.vehicle.preflight_health import PreflightHealth
from navpy.modules.vehicle.vehicle_identity import VehicleIdentity
from navpy.modules.vehicle.vehicle_mav import VehicleMav


def _router_and_health(target_system: int = 42):
    messages = MessageStore()
    heartbeat = HeartbeatState(target_system, 3.0)
    packet_loss = PacketLossTracker()
    router = InboundMessageRouter(
        VehicleIdentity(
            target_system,
            target_system,
            MAV_COMP_ID_ONBOARD_COMPUTER,
            MAV_TYPE_ONBOARD_CONTROLLER,
        ),
        messages,
        CallbackRegistry(),
        ParameterRepository(),
        heartbeat,
        packet_loss,
        MissionInbox(),
        SimpleNamespace(value=MagicMock()),
    )
    return router, messages, PreflightHealth(messages, heartbeat, packet_loss)


def _message(message_type: str, source_system: int, source_component: int = 1):
    message = MagicMock()
    message.get_msgId.return_value = 0
    message.get_srcSystem.return_value = source_system
    message.get_srcComponent.return_value = source_component
    message.get_seq.return_value = 0
    message.get_type.return_value = message_type
    return message


def _heartbeat(mav_type: int, base_mode: int = 0, source_system: int = 42):
    message = _message("HEARTBEAT", source_system)
    message.type = mav_type
    message.base_mode = base_mode
    return message


def _construction_bus():
    connection = SimpleNamespace(mav=MagicMock())
    bus = MagicMock(
        conn=connection,
        send_lock=threading.RLock(),
        heartbeats={},
    )
    return bus


def test_transmits_with_vehicle_sys_id_and_companion_component():
    bus = _construction_bus()
    with patch(
        "navpy.modules.vehicle.mav_bus.MavBus.get_or_create",
        return_value=bus,
    ) as get_or_create:
        vehicle = VehicleMav(
            "udp:0.0.0.0:5760",
            target_system=121,
            logger=MagicMock(),
            skip_mission_download=True,
            wait_heartbeat=False,
            send_heartbeat=False,
        )
    try:
        assert get_or_create.call_args.args[2:4] == (
            121,
            MAV_COMP_ID_ONBOARD_COMPUTER,
        )
        assert vehicle.target_system == 121
        assert vehicle.source_system == 121
    finally:
        vehicle.close()


def test_reports_actual_transport_encoder_source_identity():
    bus = _construction_bus()
    bus.conn.mav.srcSystem = 255
    bus.conn.mav.srcComponent = 190
    vehicle = VehicleMav(
        "unused",
        target_system=7,
        logger=MagicMock(),
        skip_mission_download=True,
        wait_heartbeat=False,
        send_heartbeat=False,
        bus=bus,
    )
    try:
        assert vehicle.source_system == 7
        assert vehicle.transport_source_system == 255
        assert vehicle.transport_source_component == 190
    finally:
        vehicle.close()


def test_custom_component_id_is_honoured():
    bus = _construction_bus()
    with patch(
        "navpy.modules.vehicle.mav_bus.MavBus.get_or_create",
        return_value=bus,
    ) as get_or_create:
        vehicle = VehicleMav(
            "udp:0.0.0.0:5760",
            target_system=7,
            logger=MagicMock(),
            skip_mission_download=True,
            wait_heartbeat=False,
            send_heartbeat=False,
            mav_comp_id=154,
        )
    try:
        assert get_or_create.call_args.args[2:4] == (7, 154)
    finally:
        vehicle.close()


def test_accepts_only_its_own_target_system():
    router, messages, _health = _router_and_health(42)

    router.ingest(_message("ATTITUDE", 42))
    router.ingest(_message("VFR_HUD", 142))

    assert messages.message("ATTITUDE") is not None
    assert messages.message("VFR_HUD") is None


def test_companion_heartbeat_does_not_replace_aircraft_armed_state():
    router, _messages, health = _router_and_health()

    router.ingest(_heartbeat(MAV_TYPE_ONBOARD_CONTROLLER))

    assert not health.is_armed


def test_aircraft_heartbeat_updates_armed_state():
    router, _messages, health = _router_and_health()

    router.ingest(_heartbeat(MAV_TYPE_FIXED_WING, MAV_MODE_FLAG_SAFETY_ARMED))

    assert health.is_armed


def _sequenced(message_type, source_system, source_component, sequence):
    message = _message(message_type, source_system, source_component)
    message.get_seq.return_value = sequence
    return message


def test_companion_components_do_not_create_phantom_packet_loss():
    router, _messages, health = _router_and_health(42)
    pattern = [MAV_COMP_ID_ONBOARD_COMPUTER, 100, 100]

    for sequence in range(30):
        router.ingest(_sequenced(
            "CAMERA_SETTINGS",
            42,
            pattern[sequence % len(pattern)],
            sequence % 256,
        ))

    assert health.link_quality == 100


def test_autopilot_gaps_still_count_as_packet_loss():
    router, _messages, health = _router_and_health(42)

    for sequence in (0, 1, 5):
        router.ingest(_sequenced("VFR_HUD", 42, 1, sequence))

    assert health.link_quality == 57
