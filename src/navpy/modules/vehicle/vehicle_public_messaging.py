"""Stateless messaging facet for :class:`VehicleMav`."""

from __future__ import annotations

from collections.abc import Callable

from pymavlink.dialects.v20.ardupilotmega import MAVLink_message

from navpy.modules.vehicle.admission_tap import (
    AdmissionSubscription,
    RulingCallback,
)
from navpy.modules.vehicle.message_subscriptions import Subscription
from navpy.modules.vehicle.vehicle_public_ports import MessagingParts


class VehicleMessagingFacet:
    """Delegate inbound subscriptions and outbound MAVLink messages."""

    _parts: MessagingParts

    def feed_message(self, message: MAVLink_message) -> None:
        return self._parts.messaging.feed_message(message)

    def wait_heartbeat_from(
        self,
        target_sysid: int,
        timeout: float = 30.0,
    ) -> bool:
        return self._parts.lifetime.wait_heartbeat_from(
            target_sysid,
            timeout,
        )

    def on_message(
        self,
        message_name: str,
        callback: Callable[[MAVLink_message], None],
    ) -> Subscription:
        return self._parts.messaging.on_message(message_name, callback)

    def on_admission(
        self,
        message_name: str,
        callback: RulingCallback,
    ) -> AdmissionSubscription:
        # VehicleMav's one method for the admission tap, D1 of
        # the LANDING2 step-1 plan, under the
        # guard's allowance of 83 that the owner approved on 2026-09-10.
        return self._parts.messaging.on_admission(message_name, callback)

    def register_rc_channel(
        self,
        channel: int,
        receiver: Callable[[int], None],
    ) -> Subscription:
        return self._parts.messaging.register_rc_channel(
            channel,
            receiver,
        )

    def send_mavlink_message(
        self,
        message: MAVLink_message,
        *,
        source_component: int | None = None,
    ) -> None:
        return self._parts.messaging.send_mavlink_message(
            message,
            source_component=source_component,
        )

    def send_status_text(self, message: str) -> None:
        return self._parts.messaging.send_status_text(message)
