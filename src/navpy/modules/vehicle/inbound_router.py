"""Ordered MAVLink message admission, publication, and callback dispatch."""
from __future__ import annotations

import time

from pymavlink.dialects.v20.ardupilotmega import (
    MAV_COMP_ID_AUTOPILOT1,
    MAVLink_message,
    MAV_TYPE_GCS,
)

from navpy.modules.comm.messages.msg_abc import MsgRegistry
import navpy.modules.vehicle.pose_cadence_debug as pose_cadence_debug
from navpy.modules.vehicle.admission_tap import (
    AdmissionSubscription,
    AdmissionTap,
    RulingCallback,
)
from navpy.modules.vehicle.link_state import HeartbeatState, PacketLossTracker
from navpy.modules.vehicle.logger_ref import LoggerRef
from navpy.modules.vehicle.message_store import MessageStore
from navpy.modules.vehicle.message_subscriptions import CallbackRegistry
from navpy.modules.vehicle.mission_inbox import MissionInbox
from navpy.modules.vehicle.parameter_repository import ParameterRepository
from navpy.modules.vehicle.vehicle_identity import VehicleIdentity


AUTOPILOT_TELEMETRY_TYPES = frozenset({
    "ATTITUDE",
    "GLOBAL_POSITION_INT",
    "GPS_RAW_INT",
    "LOCAL_POSITION_NED",
    "NAV_CONTROLLER_OUTPUT",
    "SIM_STATE",
    "VFR_HUD",
    "WIND",
})

BOOT_TIME_REBOOT_JUMP_MS = 60_000
FRESH_BOOT_WINDOW_MS = 3_000

# Why the router rejected a message it rules on, as the tap reports it.
REJECT_FOREIGN_COMPONENT = "foreign_component"
REJECT_STALE_BOOT = "stale_boot"


def message_component_id(message: MAVLink_message) -> int | None:
    try:
        component_id = message.get_srcComponent()
    except Exception:
        return None
    return component_id if isinstance(component_id, int) else None


def message_boot_time_ms(message: MAVLink_message) -> int | None:
    value = getattr(message, "time_boot_ms", None)
    if isinstance(value, bool) or not isinstance(value, int):
        return None
    return value & 0xFFFFFFFF


def boot_time_newer_or_equal(candidate_ms: int, previous_ms: int) -> bool:
    if candidate_ms == previous_ms:
        return True
    delta = (candidate_ms - previous_ms) & 0xFFFFFFFF
    if delta < 0x80000000:
        return True
    backward_ms = 0x100000000 - delta
    if backward_ms > BOOT_TIME_REBOOT_JUMP_MS:
        return True
    return candidate_ms < FRESH_BOOT_WINDOW_MS <= previous_ms


class InboundMessageRouter:
    def __init__(
        self,
        identity: VehicleIdentity,
        message_store: MessageStore,
        callbacks: CallbackRegistry,
        parameters: ParameterRepository,
        heartbeat_state: HeartbeatState,
        packet_loss: PacketLossTracker,
        mission_inbox: MissionInbox,
        logger_ref: LoggerRef,
    ) -> None:
        self._identity = identity
        self._store = message_store
        self._callbacks = callbacks
        self._parameters = parameters
        self._heartbeat = heartbeat_state
        self._packet_loss = packet_loss
        self._mission_inbox = mission_inbox
        self._logger_ref = logger_ref
        # Every ruling, numbered, for a recorder (``admission_tap``).
        self._tap = AdmissionTap(message_boot_time_ms, AUTOPILOT_TELEMETRY_TYPES)

    def ingest(self, message: MAVLink_message) -> None:
        is_navlink = MsgRegistry.has_mav_id(message.get_msgId())
        source_system = message.get_srcSystem()
        if (
            source_system != self._identity.target_system
            and self._identity.mav_type != MAV_TYPE_GCS
            and not is_navlink
        ):
            return
        message_type = message.get_type()
        receipt_time_s = time.time()
        component_id = message_component_id(message)
        if (
            message_type == "HEARTBEAT"
            and source_system == self._identity.target_system
            and component_id == MAV_COMP_ID_AUTOPILOT1
        ):
            self._heartbeat.observe(message)
        elif message_type == "PARAM_VALUE":
            self._parameters.update(message.param_id, message.param_value)
        if source_system == self._identity.target_system and component_id == MAV_COMP_ID_AUTOPILOT1:
            self._packet_loss.observe(component_id, message.get_seq())
        verdict = self._accept_telemetry(message_type, message, component_id)
        if verdict is not None:
            accepted, reason = verdict
            # Ruled on: the tap hears it before anything is published.
            self._tap.rule(message_type, message, accepted, reason)
            if not accepted:
                return
        boot_time_ms = message_boot_time_ms(message)
        self._store.publish(
            message_type,
            message,
            receipt_time_s=receipt_time_s,
            boot_time_ms=boot_time_ms,
        )
        self._record_pose_arrival(message_type, receipt_time_s, boot_time_ms)
        self._mission_inbox.publish(message, receipt_time_s)
        self._callbacks.dispatch(message_type, message, is_navlink=is_navlink)

    def feed_message(self, message: MAVLink_message) -> None:
        self.ingest(message)

    def on_admission(
        self,
        message_name: str,
        callback: RulingCallback,
    ) -> AdmissionSubscription:
        """Every later ruling on ``message_name``, numbered: see
        ``admission_tap``."""
        return self._tap.subscribe(message_name, callback)

    def _accept_telemetry(
        self,
        message_type: str,
        message: MAVLink_message,
        component_id: int | None,
    ) -> tuple[bool, str | None] | None:
        """The verdict on a message this router rules on, with the reason for
        a rejection; None for a message it does not rule on, which passes."""
        if (
            message.get_srcSystem() != self._identity.target_system
            or message_type not in AUTOPILOT_TELEMETRY_TYPES
        ):
            return None
        if component_id is not None and component_id != MAV_COMP_ID_AUTOPILOT1:
            self._record_rejection(message_type, REJECT_FOREIGN_COMPONENT)
            return False, REJECT_FOREIGN_COMPONENT
        boot_time_ms = message_boot_time_ms(message)
        previous = self._store.latest(message_type)
        if boot_time_ms is None or previous is None or previous.boot_time_ms is None:
            return True, None
        if boot_time_newer_or_equal(boot_time_ms, previous.boot_time_ms):
            return True, None
        self._record_rejection(message_type, REJECT_STALE_BOOT)
        return False, REJECT_STALE_BOOT

    def _record_pose_arrival(
        self,
        message_type: str,
        receipt_time_s: float,
        boot_time_ms: int | None,
    ) -> None:
        if not pose_cadence_debug.ENABLED:
            return
        if message_type == "ATTITUDE":
            pose_cadence_debug.record_attitude_arrival(
                self._identity.target_system, receipt_time_s, boot_time_ms or 0,
            )
        elif message_type == "GLOBAL_POSITION_INT":
            pose_cadence_debug.record_position_arrival(
                self._identity.target_system, receipt_time_s, boot_time_ms or 0,
            )

    def _record_rejection(self, message_type: str, reason: str) -> None:
        if pose_cadence_debug.ENABLED:
            pose_cadence_debug.record_telemetry_reject(
                self._identity.target_system, time.time(), message_type, reason,
            )
