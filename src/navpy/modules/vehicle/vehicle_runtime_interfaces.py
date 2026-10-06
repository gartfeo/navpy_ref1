"""Vehicle messaging, logging, and lifetime contracts."""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Callable
from typing import TYPE_CHECKING

from pymavlink.dialects.v20.ardupilotmega import MAVLink_message

from navpy.logger.cache_logger import ILogger

if TYPE_CHECKING:
    from navpy.modules.vehicle.admission_tap import (
        AdmissionSubscription,
        RulingCallback,
    )
    from navpy.modules.vehicle.message_subscriptions import Subscription

RcChannelReceiver = Callable[[int], None]


class VehicleMessaging(ABC):
    @abstractmethod
    def register_rc_channel(
        self,
        rc_channel: int,
        rc_channel_receiver: RcChannelReceiver,
    ) -> "Subscription": ...

    @abstractmethod
    def on_message(
        self,
        message_name: str,
        callback: Callable[[MAVLink_message], None],
    ) -> "Subscription": ...

    @abstractmethod
    def send_mavlink_message(
        self,
        mav_msg: MAVLink_message,
        *,
        source_component: int | None = None,
    ) -> None: ...

    @abstractmethod
    def send_status_text(self, msg: str) -> None: ...

    def on_admission(
        self,
        message_name: str,
        callback: "RulingCallback",
    ) -> "AdmissionSubscription | None":
        """Every later ruling on ``message_name``, numbered, for a recorder
        (``admission_tap``). None: this vehicle has no admission tap."""
        return None


class VehicleLifetime(ABC):
    @abstractmethod
    def close(self) -> None: ...


class VehicleLogging(ABC):
    @abstractmethod
    def set_logger(self, logger: ILogger) -> None: ...


__all__ = [
    "RcChannelReceiver",
    "VehicleLifetime",
    "VehicleLogging",
    "VehicleMessaging",
]
