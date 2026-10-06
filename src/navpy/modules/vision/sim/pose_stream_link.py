"""Own a vehicle's pose-stream request and message subscriptions."""

from __future__ import annotations

from collections.abc import Callable, Iterable

from navpy.modules.vehicle.message_subscriptions import Subscription
from navpy.modules.vehicle.pose_streams import (
    pose_stream_rate_for_scheduler_rate_hz,
    request_pose_streams,
    request_simulator_truth_pose_stream,
    resolve_ardupilot_scheduler_rate_hz,
)
from navpy.modules.vehicle.vehicle_interface import IVehicle

MessageCallback = Callable[[object], None]


class PoseStreamLink:
    """Request pose streams once and fan messages into one callback."""

    def __init__(
        self,
        vehicle: IVehicle,
        *,
        guard: Callable[[MessageCallback], MessageCallback] | None = None,
    ) -> None:
        self._vehicle = vehicle
        # Wraps the callback at start(), once for every type: the decision
        # trace counts what it raises (DeterminismTrace.guard_callback).
        self._guard = guard
        # ONE parameter read for BOTH rates. The stream rate is derived from
        # the scheduler rate, and a second read can answer differently, which
        # would put the association skew window and any scheduler-slot grid
        # built beside it on grids that disagree.
        self._scheduler_rate_hz = resolve_ardupilot_scheduler_rate_hz(vehicle)
        self._rate_hz = pose_stream_rate_for_scheduler_rate_hz(
            self._scheduler_rate_hz
        )
        self._subscriptions: tuple[Subscription, ...] = ()

    @property
    def rate_hz(self) -> float:
        return self._rate_hz

    @property
    def scheduler_rate_hz(self) -> float:
        """The scheduler rate ``rate_hz`` was derived from -- same read."""
        return self._scheduler_rate_hz

    def start(
        self,
        on_message: MessageCallback,
        message_types: Iterable[str],
        *,
        request_truth: bool = False,
    ) -> None:
        """Raise the pose streams and subscribe ``on_message`` to each type.

        ``request_truth`` additionally asks for SIM_STATE, which has no default
        Plane stream. It is opt-in because a caller that renders from telemetry
        pose would silently pay for a stream it never reads. With a ``guard``,
        what each type is subscribed with is ``guard(on_message)``, built once.
        """
        request_pose_streams(self._vehicle, rate_hz=self._rate_hz)
        if request_truth:
            request_simulator_truth_pose_stream(
                self._vehicle,
                rate_hz=self._rate_hz,
            )
        callback = (
            on_message if self._guard is None else self._guard(on_message)
        )
        self._subscriptions = tuple(
            self._vehicle.on_message(name, callback)
            for name in message_types
        )

    def close(self) -> None:
        subscriptions, self._subscriptions = self._subscriptions, ()
        for subscription in subscriptions:
            subscription.cancel()


__all__ = ["MessageCallback", "PoseStreamLink"]
