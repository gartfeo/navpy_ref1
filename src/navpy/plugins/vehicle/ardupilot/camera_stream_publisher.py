"""Periodic lifecycle for NavPy's camera components: heartbeats and answers.

Requests arrive on the bus reader thread and are queued; the worker sends
every reply and heartbeat, so nothing is sent from a stopped publisher.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Callable, Iterable
from typing import Protocol

from pymavlink.dialects.v20.ardupilotmega import MAVLink_message

from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.thread_launch import ThreadLaunchGate
from navpy.core.ports.subscription import Subscription
from navpy.core.ports.worker_failure import WorkerFailureLatch
from navpy.plugins.vehicle.ardupilot.camera_protocol_timing import (
    CAMERA_HEARTBEAT_PERIOD_S,
)
from navpy.plugins.vehicle.ardupilot.camera_stream_messages import (
    CameraStreamResponder,
    StreamingCamera,
    camera_heartbeat,
    new_camera_boot_id,
)

# Requests held between worker passes. A ground station asks one camera one
# thing at a time (design section 1), so this only bounds memory under a
# flood: the oldest request is dropped and its requester retries
# (CAMERA_REQUEST_MAX_SENDS).
CAMERA_REQUEST_BACKLOG = 32
_STOP_JOIN_TIMEOUT_S = 2.0


class CameraStreamVehicle(Protocol):
    def send_mavlink_message(
        self,
        message: object,
        *,
        source_component: int | None = None,
    ) -> None: ...

    def on_message(
        self,
        message_name: str,
        callback: Callable[[MAVLink_message], None],
    ) -> Subscription: ...


class CameraStreamLogger(Protocol):
    def warning(self, message: str) -> None: ...


class CameraStreamPublisher:
    """Own one retryable worker for every camera component."""

    def __init__(
        self,
        vehicle: CameraStreamVehicle,
        own_system: int,
        cameras: Iterable[StreamingCamera],
        logger: CameraStreamLogger,
    ) -> None:
        self._vehicle = vehicle
        self._responder = CameraStreamResponder(own_system, cameras)
        self._boot_id = new_camera_boot_id()
        self._logger = logger
        self._requests: deque[MAVLink_message] = deque(maxlen=CAMERA_REQUEST_BACKLOG)
        self._requests_ready = rt.Condition(rt.Lock())
        self._subscription: Subscription | None = None
        self._stop = rt.Event()
        self._thread: rt.Thread | None = None
        self._launch: ThreadLaunchGate | None = None
        self._lifecycle_lock = rt.Lock()
        self._failures = WorkerFailureLatch()

    def start(self) -> None:
        if not self._responder.component_ids:
            return  # no camera component: nothing is ever sent
        with self._lifecycle_lock:
            if self._thread is not None:
                if (
                    self._launch is not None and not self._launch.committed
                ) or self._thread.is_alive():
                    return
                self._thread = None
                self._launch = None
            stop_event = rt.Event()
            launch = ThreadLaunchGate()
            thread = rt.Thread(
                target=self._run_generation,
                args=(stop_event, launch),
                name="camera-stream-publisher",
                daemon=True,
            )
            self._stop = stop_event
            self._thread = thread
            self._launch = launch
            if self._subscription is None:
                self._subscription = self._vehicle.on_message(
                    "COMMAND_LONG", self._enqueue
                )
            try:
                with rt.detached():  # owned by this component, not its creating request
                    thread.start()
            except BaseException:
                stop_event.set()
                if launch.cancel_before_commit() and self._thread is thread:
                    self._thread = None
                    self._launch = None
                raise

    def stop(self) -> bool:
        with self._lifecycle_lock:
            self._stop.set()
            subscription, self._subscription = self._subscription, None
            thread = self._thread
            launch = self._launch
            if subscription is not None:
                subscription.cancel()
            self._wake()
            if (
                thread is not None
                and launch is not None
                and launch.cancel_before_commit()
            ):
                self._thread = None
                self._launch = None
                return True
        if thread is not None:
            if thread is rt.current_thread():
                return False
            if thread.is_alive():
                thread.join(timeout=_STOP_JOIN_TIMEOUT_S)
            if thread.is_alive():
                self._logger.warning(
                    "Camera stream publisher did not stop within "
                    f"{_STOP_JOIN_TIMEOUT_S} seconds"
                )
                return False
        with self._lifecycle_lock:
            if self._thread is thread:
                self._thread = None
                self._launch = None
        return True

    @property
    def is_quiescent(self) -> bool:
        thread = self._thread
        if thread is None:
            return True
        launch = self._launch
        if launch is not None and not launch.committed:
            return False
        try:
            return not thread.is_alive()
        except BaseException:
            return False

    def raise_if_failed(self) -> None:
        self._failures.raise_if_failed()

    def send_heartbeats(self) -> None:
        for camera in self._responder.cameras:
            self._vehicle.send_mavlink_message(
                camera_heartbeat(camera.camera_health.ready(), self._boot_id),
                source_component=camera.component_id,
            )

    def answer(self, request: MAVLink_message) -> None:
        replies = self._responder.answer(request)
        for reply in replies:
            self._vehicle.send_mavlink_message(
                reply, source_component=request.target_component
            )

    def _enqueue(self, request: MAVLink_message) -> None:
        with self._requests_ready:
            self._requests.append(request)
            self._requests_ready.notify_all()

    def _wake(self) -> None:
        with self._requests_ready:
            self._requests_ready.notify_all()

    def _take_requests(
        self,
        stop_event: rt.Event,
        timeout_s: float,
    ) -> tuple[MAVLink_message, ...]:
        with self._requests_ready:
            if not self._requests and not stop_event.is_set():
                self._requests_ready.wait(timeout_s)
            requests = tuple(self._requests)
            self._requests.clear()
        return requests

    def _run_generation(
        self,
        stop_event: rt.Event,
        launch: ThreadLaunchGate,
    ) -> None:
        if not launch.enter():
            return
        self._failures.run(lambda: self._run(stop_event))

    def _run(self, stop_event: rt.Event) -> None:
        next_heartbeat_s = rt.monotonic()
        while not stop_event.is_set():
            now_s = rt.monotonic()
            if now_s >= next_heartbeat_s:
                self.send_heartbeats()
                next_heartbeat_s = now_s + CAMERA_HEARTBEAT_PERIOD_S
            requests = self._take_requests(
                stop_event, max(0.0, next_heartbeat_s - rt.monotonic())
            )
            for request in requests:
                if stop_event.is_set():
                    return
                self.answer(request)


__all__ = [
    "CAMERA_REQUEST_BACKLOG",
    "CameraStreamPublisher",
]
