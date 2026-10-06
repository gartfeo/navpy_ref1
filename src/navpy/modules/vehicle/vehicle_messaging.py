"""Inbound subscriptions and generic MAVLink message transmission."""
from __future__ import annotations

from collections.abc import Callable

from pymavlink import mavutil
from pymavlink.dialects.v20.ardupilotmega import MAVLink_message

from navpy.modules.vehicle.admission_tap import (
    AdmissionSubscription,
    RulingCallback,
)
from navpy.modules.vehicle.inbound_router import InboundMessageRouter
from navpy.modules.vehicle.mav_transport import MavTransport
from navpy.modules.vehicle.message_subscriptions import CallbackRegistry, Subscription


class VehicleMessaging:
    def __init__(
        self,
        transport: MavTransport,
        callbacks: CallbackRegistry,
        router: InboundMessageRouter,
    ) -> None:
        self._transport = transport
        self._callbacks = callbacks
        self._router = router

    def feed_message(self, message: MAVLink_message) -> None:
        self._router.ingest(message)

    def on_message(
        self,
        message_name: str,
        callback: Callable[[MAVLink_message], None],
    ) -> Subscription:
        return self._callbacks.subscribe(message_name, callback)

    def on_admission(
        self,
        message_name: str,
        callback: RulingCallback,
    ) -> AdmissionSubscription:
        return self._router.on_admission(message_name, callback)

    def register_rc_channel(
        self,
        channel: int,
        receiver: Callable[[int], None],
    ) -> Subscription:
        def receive(message: MAVLink_message) -> None:
            value = getattr(message, f"chan{channel}_raw", None)
            if value is not None:
                receiver(value)

        return self.on_message("RC_CHANNELS", receive)

    def send_mavlink_message(
        self,
        message: MAVLink_message,
        *,
        source_component: int | None = None,
    ) -> None:
        self._transport.send(message, source_component=source_component)

    def send_status_text(self, message: str) -> None:
        payload = message.encode("utf-8")[:50]
        self._transport.try_call(
            lambda connection: connection.mav.statustext_send(
                mavutil.mavlink.MAV_SEVERITY_INFO, payload,
            ),
            blocking=False,
        )
