"""Companion heartbeat sender lifecycle."""
from __future__ import annotations

import threading

from pymavlink.dialects.v20.ardupilotmega import (
    MAV_AUTOPILOT_INVALID,
    MAV_STATE_ACTIVE,
    MAV_TYPE_GCS,
)

from navpy.modules.vehicle.link_state import HeartbeatState
from navpy.modules.vehicle.logger_ref import LoggerRef
from navpy.modules.vehicle.mav_transport import MavTransport
from navpy.modules.vehicle.vehicle_identity import VehicleIdentity
from navpy.modules.common.thread_launch import ThreadLaunchGate


HEARTBEAT_JOIN_TIMEOUT_S = 2.0


class HeartbeatRuntime:
    def __init__(
        self,
        transport: MavTransport,
        identity: VehicleIdentity,
        heartbeat_state: HeartbeatState,
        logger_ref: LoggerRef,
        heartbeat_hz: float,
    ) -> None:
        self._transport = transport
        self._identity = identity
        self._state = heartbeat_state
        self._logger_ref = logger_ref
        self._period_s = 1.0 / max(heartbeat_hz, 0.1)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._launch: ThreadLaunchGate | None = None
        self._lock = threading.Lock()

    def start(self) -> None:
        with self._lock:
            if self._thread is not None:
                return
            stop = threading.Event()
            launch = ThreadLaunchGate()
            thread = threading.Thread(
                target=self._run_generation,
                args=(stop, launch),
                daemon=True,
            )
            self._stop = stop
            self._thread = thread
            self._launch = launch
            try:
                thread.start()
            except BaseException:
                stop.set()
                if launch.cancel_before_commit():
                    self._thread = None
                    self._launch = None
                raise

    def close(self) -> None:
        with self._lock:
            self._stop.set()
            thread = self._thread
            launch = self._launch
            if (
                thread is not None
                and launch is not None
                and launch.cancel_before_commit()
            ):
                self._thread = None
                self._launch = None
                return
        if thread is None:
            return
        if thread is threading.current_thread():
            raise TimeoutError("heartbeat worker cannot join itself")
        if thread.is_alive():
            thread.join(timeout=HEARTBEAT_JOIN_TIMEOUT_S)
        if thread.is_alive():
            raise TimeoutError(
                "heartbeat worker did not stop; vehicle remains owned"
            )
        with self._lock:
            if self._thread is thread:
                self._thread = None
                self._launch = None

    def _run_generation(
        self,
        stop: threading.Event,
        launch: ThreadLaunchGate,
    ) -> None:
        if not launch.enter():
            return
        self._run(stop)

    def _run(self, stop: threading.Event | None = None) -> None:
        stop = self._stop if stop is None else stop
        while not stop.is_set():
            self._transport.call(
                lambda connection: connection.mav.heartbeat_send(
                    self._identity.mav_type,
                    MAV_AUTOPILOT_INVALID,
                    0,
                    0,
                    MAV_STATE_ACTIVE,
                )
            )
            age_s = self._state.age_s
            if age_s > 0.0 and self._identity.mav_type != MAV_TYPE_GCS:
                if not self._state.link_ok:
                    self._logger_ref.value.warning(
                        f"Vehicle {self._identity.target_system} "
                        f"heartbeat timeout {age_s}."
                    )
            stop.wait(self._period_s)
