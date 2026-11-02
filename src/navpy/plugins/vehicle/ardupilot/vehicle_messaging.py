"""Inbound subscriptions and generic MAVLink message transmission."""
from __future__ import annotations

from collections.abc import Callable

from pymavlink import mavutil
from pymavlink.dialects.v20.ardupilotmega import MAVLink_message

from navpy.core.ports.admission_tap import (
    AdmissionSubscription,
    RulingCallback,
)
from navpy.plugins.vehicle.ardupilot.inbound_router import InboundMessageRouter
from navpy.plugins.vehicle.ardupilot.logger_ref import LoggerRef
from navpy.plugins.vehicle.ardupilot.mav_transport import MavTransport
from navpy.plugins.vehicle.ardupilot.message_subscriptions import CallbackRegistry
from navpy.core.ports.subscription import Subscription

# Longest a status text waits for the bus send lock before it is dropped and
# logged. Every holder of that lock encodes and writes one frame to a
# non-blocking socket (bench waits: max 14-21 ms), so a text normally waits
# for one send. The bound keeps an unexpected long holder from stalling the
# logger worker that sends texts, and through its status lock a navigation
# thread's defer or flush call (nav_entry, nav_exit_cleanup).
STATUS_TEXT_SEND_WAIT_S = 0.1

class VehicleMessaging:
    def __init__(
        self,
        transport: MavTransport,
        callbacks: CallbackRegistry,
        router: InboundMessageRouter,
        logger_ref: LoggerRef,
    ) -> None:
        self._transport = transport
        self._callbacks = callbacks
        self._router = router
        self._logger_ref = logger_ref

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

        def _send(connection: mavutil.mavfile) -> bool:
            connection.mav.statustext_send(
                mavutil.mavlink.MAV_SEVERITY_INFO, payload,
            )
            return True

        sent = self._transport.try_call(
            _send,
            blocking=True,
            timeout=STATUS_TEXT_SEND_WAIT_S,
        )
        if sent is not True:
            # Text log only: a status-bound line here would queue another text.
            self._logger_ref.value.warning(
                f"status text dropped: bus send lock busy for "
                f"{STATUS_TEXT_SEND_WAIT_S:.3f} s: {message!r}",
                key="vehicle",
            )
