"""The router admits camera-protocol requests from any system, and nothing else.

docs/design/gcs_video_streams.md, section 1: a ground station on another
system id must reach NavPy's camera components with REQUEST_MESSAGE for
CAMERA_INFORMATION (259) or VIDEO_STREAM_INFORMATION (269); every other
message from a foreign system stays dropped.
"""

from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
from pymavlink.dialects.v20 import ardupilotmega as mav

from navpy.plugins.vehicle.ardupilot.inbound_router import InboundMessageRouter
from navpy.plugins.vehicle.ardupilot.link_state import HeartbeatState, PacketLossTracker
from navpy.plugins.vehicle.ardupilot.message_store import MessageStore
from navpy.plugins.vehicle.ardupilot.message_subscriptions import CallbackRegistry
from navpy.plugins.vehicle.ardupilot.mission_inbox import MissionInbox
from navpy.plugins.vehicle.ardupilot.parameter_repository import ParameterRepository
from navpy.plugins.vehicle.ardupilot.vehicle_identity import VehicleIdentity

OWN = 42
GCS_SYSTEM = 255
GCS_COMPONENT = mav.MAV_COMP_ID_MISSIONPLANNER


def _router(
    heartbeat: HeartbeatState | None = None,
) -> tuple[InboundMessageRouter, list[mav.MAVLink_message]]:
    callbacks = CallbackRegistry()
    router = InboundMessageRouter(
        VehicleIdentity(
            OWN, OWN, mav.MAV_COMP_ID_ONBOARD_COMPUTER, mav.MAV_TYPE_ONBOARD_CONTROLLER
        ),
        MessageStore({}),
        callbacks,
        ParameterRepository(),
        heartbeat or HeartbeatState(OWN, 3.0),
        PacketLossTracker(),
        MissionInbox(),
        SimpleNamespace(value=MagicMock()),
    )
    seen: list[mav.MAVLink_message] = []
    callbacks.subscribe("*", seen.append)
    return router, seen


def _wire(message: mav.MAVLink_message, system: int, component: int) -> mav.MAVLink_message:
    """``message`` as received from (system, component)."""
    sender = mav.MAVLink(None, srcSystem=system, srcComponent=component)
    return mav.MAVLink(None).decode(bytearray(message.pack(sender)))


def _request(
    message_id: float,
    *,
    target_system: int = OWN,
    target_component: int = mav.MAV_COMP_ID_CAMERA,
    command: int = mav.MAV_CMD_REQUEST_MESSAGE,
    source_system: int = GCS_SYSTEM,
) -> mav.MAVLink_message:
    return _wire(
        mav.MAVLink_command_long_message(
            target_system, target_component, command, 0,
            message_id, 0, 0, 0, 0, 0, 0,
        ),
        source_system,
        GCS_COMPONENT,
    )


@pytest.mark.parametrize("message_id", [259, 269])
@pytest.mark.parametrize(
    "camera_component",
    [mav.MAV_COMP_ID_CAMERA, mav.MAV_COMP_ID_CAMERA6],
)
def test_foreign_camera_requests_are_admitted(message_id, camera_component):
    router, seen = _router()
    request = _request(message_id, target_component=camera_component)

    router.ingest(request)

    assert seen == [request]


@pytest.mark.parametrize(
    "request_kwargs",
    [
        pytest.param({"message_id": 0}, id="other-message-heartbeat"),
        pytest.param({"message_id": 260}, id="other-message-camera-settings"),
        pytest.param({"message_id": 270}, id="other-message-stream-status"),
        pytest.param({"message_id": 259.5}, id="non-integer-message-id"),
        pytest.param({"message_id": 259, "target_system": OWN + 1}, id="other-system"),
        pytest.param({"message_id": 259, "target_system": 0}, id="broadcast-system"),
        pytest.param(
            {"message_id": 269, "target_component": mav.MAV_COMP_ID_AUTOPILOT1},
            id="autopilot-component",
        ),
        pytest.param(
            {"message_id": 269, "target_component": mav.MAV_COMP_ID_ONBOARD_COMPUTER},
            id="companion-component",
        ),
        pytest.param({"message_id": 269, "target_component": 0}, id="broadcast-component"),
        pytest.param(
            {"message_id": 259, "command": mav.MAV_CMD_COMPONENT_ARM_DISARM},
            id="other-command",
        ),
    ],
)
def test_other_foreign_commands_stay_dropped(request_kwargs):
    router, seen = _router()

    router.ingest(_request(**request_kwargs))

    assert seen == []


def test_other_foreign_traffic_stays_dropped():
    router, seen = _router()
    foreign = [
        _wire(
            mav.MAVLink_heartbeat_message(mav.MAV_TYPE_GCS, 8, 0, 0, 0, 3),
            GCS_SYSTEM,
            GCS_COMPONENT,
        ),
        _wire(
            mav.MAVLink_command_int_message(
                OWN, mav.MAV_COMP_ID_CAMERA, 0, mav.MAV_CMD_REQUEST_MESSAGE,
                0, 0, 259, 0, 0, 0, 0, 0, 0,
            ),
            GCS_SYSTEM,
            GCS_COMPONENT,
        ),
        _wire(
            mav.MAVLink_attitude_message(1, 0, 0, 0, 0, 0, 0),
            OWN + 1,
            mav.MAV_COMP_ID_AUTOPILOT1,
        ),
    ]

    for message in foreign:
        router.ingest(message)

    assert seen == []


@pytest.mark.parametrize("status", [mav.MAV_STATE_ACTIVE, mav.MAV_STATE_CRITICAL])
def test_own_camera_heartbeat_with_boot_id_leaves_autopilot_heartbeat_alone(status):
    """The camera HEARTBEAT carries NavPy's boot id in custom_mode and the
    detector readiness in system_status (docs/design/gcs_preflight_camera_row.md);
    neither may reach the autopilot heartbeat that flight mode is read from."""
    heartbeat = HeartbeatState(OWN, 3.0)
    router, _ = _router(heartbeat)
    autopilot = _wire(
        mav.MAVLink_heartbeat_message(
            mav.MAV_TYPE_FIXED_WING, mav.MAV_AUTOPILOT_ARDUPILOTMEGA,
            mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 10, mav.MAV_STATE_ACTIVE, 3,
        ),
        OWN,
        mav.MAV_COMP_ID_AUTOPILOT1,
    )
    camera = _wire(
        mav.MAVLink_heartbeat_message(
            mav.MAV_TYPE_CAMERA, mav.MAV_AUTOPILOT_INVALID, 0, 0xDEADBEEF, status, 3,
        ),
        OWN,
        mav.MAV_COMP_ID_CAMERA,
    )

    router.ingest(autopilot)
    router.ingest(camera)

    assert heartbeat.message is autopilot
    assert heartbeat.count == 1


def test_own_system_commands_still_pass_unchanged():
    router, seen = _router()
    own = _request(0, source_system=OWN, target_component=mav.MAV_COMP_ID_AUTOPILOT1)

    router.ingest(own)

    assert seen == [own]
