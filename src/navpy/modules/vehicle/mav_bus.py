"""Shared MAVLink connection boundary with one reader per device."""
from __future__ import annotations

import logging
import socket
import threading
from _thread import RLock as RLockType

from pymavlink import mavutil

from navpy.modules.vehicle.mav_bus_heartbeats import MavBusHeartbeats
from navpy.modules.vehicle.mav_bus_lifecycle import (
    MavBusClosedError,
    MavBusLifecycle,
)
from navpy.modules.vehicle.mav_bus_membership import (
    MavBusMembership,
    MavMessageReceiver,
)
from navpy.modules.vehicle.mav_bus_reader import MavBusReader
from navpy.modules.vehicle.mav_bus_registry import (
    MavBusRegistry,
    MavBusRegistryPorts,
)

log = logging.getLogger(__name__)

_RCVBUF_BYTES = 4 * 1024 * 1024


def _enlarge_recv_buffer(
    connection: mavutil.mavfile,
    device: str,
) -> None:
    """Best-effort receive-buffer hardening for socket transports."""
    port = getattr(connection, "port", None)
    if port is None or not hasattr(port, "setsockopt"):
        return
    try:
        port.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, _RCVBUF_BYTES)
        actual = port.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
        log.info(
            "MavBus %s: SO_RCVBUF set (requested=%d actual=%d)",
            device,
            _RCVBUF_BYTES,
            actual,
        )
    except OSError as exc:
        log.debug("MavBus %s: could not enlarge SO_RCVBUF: %s", device, exc)


class MavBusLease:
    """One reservation, atomically convertible to a target attachment."""

    def __init__(
        self,
        lifecycle: MavBusLifecycle,
        membership: MavBusMembership,
        target_system: int,
    ) -> None:
        self._lifecycle = lifecycle
        self._membership = membership
        self._target = target_system
        self._state = "reserved"

    def attach(self, receiver: MavMessageReceiver) -> None:
        def activate() -> None:
            if self._state != "reserved":
                raise RuntimeError(f"cannot attach lease in state {self._state}")
            self._membership.activate(self._target, receiver)
            self._state = "attached"

        self._lifecycle.change(activate)

    def release(self) -> None:
        def remove() -> None:
            if self._state == "released":
                return
            self._membership.release(
                self._target,
                attached=self._state == "attached",
            )
            self._state = "released"

        self._lifecycle.release(remove)


class MavBus:
    """Thin boundary over transport, membership, reader, and lifecycle owners."""

    @staticmethod
    def get_or_create(
        device: str,
        baud: int = 115200,
        source_system: int = 1,
        source_component: int = 0,
    ) -> "MavBus":
        def create() -> MavBus:
            connection = mavutil.mavlink_connection(
                device,
                baud=baud,
                source_system=source_system,
                source_component=source_component,
                autoreconnect=True,
            )
            _enlarge_recv_buffer(connection, device)
            bus = MavBus(connection, device)
            log.info("MavBus created for %s (src_sys=%d)", device, source_system)
            return bus

        return _registry.get_or_create(device, create)

    def __init__(
        self,
        connection: mavutil.mavfile,
        device: str,
    ) -> None:
        self._conn = connection
        # Public plain attribute, deliberately not a property: the canonical
        # endpoint this connection was opened on, read by the live-link
        # identity used to bind capture calibration.
        self.device = device
        self._send_lock = threading.RLock()
        self._heartbeats = MavBusHeartbeats()
        membership = MavBusMembership()
        self._membership = membership
        self._lifecycle = MavBusLifecycle(
            connection,
            device,
            membership,
            lambda: _registry.remove(device, membership),
        )
        reader = MavBusReader(
            connection,
            device,
            self._membership,
            self._heartbeats,
            self._lifecycle.stop_event,
        )
        self._lifecycle.bind_reader(reader)
        reader.start()

    @property
    def conn(self) -> mavutil.mavfile:
        return self._conn

    @property
    def send_lock(self) -> RLockType:
        return self._send_lock

    @property
    def heartbeats(self) -> dict[int, float]:
        return self._heartbeats.vehicle

    @property
    def companion_heartbeats(self) -> dict[int, float]:
        return self._heartbeats.companion

    @property
    def has_targets(self) -> bool:
        return self._membership.has_targets

    @property
    def is_closed(self) -> bool:
        return self._lifecycle.is_closed

    @property
    def is_closing(self) -> bool:
        return self._lifecycle.is_closing

    def wait_closed(self, timeout: float | None = None) -> bool:
        return self._lifecycle.wait_closed(timeout)

    def attach(
        self,
        system_id: int,
        receiver: MavMessageReceiver,
    ) -> None:
        self._lifecycle.change(
            lambda: self._membership.attach(system_id, receiver)
        )

    def reserve(self, system_id: int) -> MavBusLease:
        self._lifecycle.change(lambda: self._membership.reserve(system_id))
        return MavBusLease(self._lifecycle, self._membership, system_id)

    def detach(self, system_id: int) -> None:
        self._lifecycle.release(
            lambda: self._membership.release(system_id, attached=True)
        )

    def close(self) -> None:
        self._lifecycle.close()

    def wait_heartbeat(self, target_sysid: int, timeout: float = 30.0) -> bool:
        return self._heartbeats.wait_for(target_sysid, timeout)


_registry: MavBusRegistry[MavBus] = MavBusRegistry(
    MavBusRegistryPorts(
        identity=lambda bus: bus._membership,
        return_if_open=lambda bus, action: bus._lifecycle.return_if_open(
            action
        ),
        wait_closed=lambda bus, timeout: bus.wait_closed(timeout),
    )
)
_buses = _registry.entries
