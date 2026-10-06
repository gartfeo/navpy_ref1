"""Companion (NavPy onboard computer) MAVLink identity.

A NavPy companion shares its aircraft's MAVLink SYSTEM id and is told apart by
COMPONENT id: the autopilot transmits as component 1, the companion as
``MAV_COMP_ID_ONBOARD_COMPUTER`` (191). The component is therefore the sole
companion discriminator — filtering on system id alone would let the
autopilot's own traffic alias as companion traffic.

Camera/gimbal optics the companion publishes ride the per-camera component ids
instead of 191 (see ``navpy.modules.vision.gimbal_telemetry_publisher``), so
telemetry filters must accept that wider set.
"""
from __future__ import annotations

from typing import Iterable, Optional

from pymavlink.dialects.v20.ardupilotmega import MAV_COMP_ID_ONBOARD_COMPUTER

from navpy.modules.vision.mavlink_camera_components import (
    GIMBAL_DEVICE_ID_BY_CAMERA_COMPONENT,
)

# The component id every NavPy companion transmits with by default
# (VehicleMav -> MavBus source_component).
COMPANION_COMPONENT_ID: int = MAV_COMP_ID_ONBOARD_COMPUTER

# Every component id a companion process transmits from: its onboard-computer
# id plus the per-camera ids carrying CAMERA_FOV_STATUS / CAMERA_SETTINGS.
# Deliberately excludes the autopilot's own component (1) and any
# autopilot-side gimbal manager component.
COMPANION_TELEMETRY_COMPONENT_IDS: frozenset[int] = frozenset(
    {COMPANION_COMPONENT_ID, *GIMBAL_DEVICE_ID_BY_CAMERA_COMPONENT}
)


def source_ids(msg) -> Optional[tuple[int, int]]:
    """``(srcSystem, srcComponent)`` of *msg*, or None if it carries neither."""
    try:
        return int(msg.get_srcSystem()), int(msg.get_srcComponent())
    except (AttributeError, TypeError, ValueError):
        return None


def is_from_companion(
    msg,
    sys_id: int,
    components: Iterable[int] = (COMPANION_COMPONENT_ID,),
) -> bool:
    """True when *msg* was transmitted by the NavPy companion of *sys_id*.

    Matches on system id AND component id, so a packet from the aircraft's own
    autopilot (component 1) is never attributed to its companion.
    """
    ids = source_ids(msg)
    if ids is None:
        return False
    source_system, source_component = ids
    return source_system == int(sys_id) and source_component in components
