"""LockstepBus: the ``MavBus`` surface composition uses, on bytes alone.

It replaces ``mavutil.mavfile`` for one participant link under lockstep
(``lockstep_slice.md``, "Bus adapter"; ``lockstep_contracts.md``, "Bus
adapter"). The shared bus parts stay the product's: ``MavBusMembership``
routes, ``MavBusLease`` reserves and attaches, ``MavBusLifecycle`` closes, and
the reader's ``_dispatch`` fans out. Only the edges change:

- **Inbound:** the coordinator delivers each tick's port bytes
  (:meth:`LockstepBus.deliver`); a managed reader decodes them with one
  persistent :class:`StreamDecoder` and dispatches in byte order, yielding
  after each packet (the wall reader yields once per bounded batch,
  ``docs/design/reader_batch.md``; lockstep keeps the per-packet baton point).
- **Outbound:** :class:`LockstepConnection` is the encoder (a ``MAVLink`` on
  the link's byte sink, with the product's source ids) plus ``mode_mapping``;
  the link drains the sink into ``DONE``.
- **Heartbeats** are admitted by the wall ``MavBusHeartbeats``.
"""

from __future__ import annotations

import logging
from collections.abc import Callable

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink

from navpy.core.kernel import runtime_time as rt
from navpy.plugins.vehicle.ardupilot.mav_bus import MavBusLease
from navpy.plugins.vehicle.ardupilot.mav_bus_heartbeats import MavBusHeartbeats
from navpy.plugins.vehicle.ardupilot.mav_bus_lifecycle import MavBusLifecycle
from navpy.plugins.vehicle.ardupilot.mav_bus_membership import MavBusMembership
from navpy.plugins.vehicle.ardupilot.mav_bus_reader import MavBusReader
from navpy.sim.lockstep.baton import BatonRuntime
from navpy.sim.lockstep.baton_queue import BatonQueue
from navpy.sim.lockstep.baton_sync import yield_now
from navpy.sim.lockstep.mavlink_wire import ByteSink, StreamDecoder, encoder

log = logging.getLogger(__name__)

class LockstepConnection:
    """What composition uses of a ``mavfile``: ``mav`` and ``mode_mapping``."""

    def __init__(self, sink: ByteSink, source_system: int, source_component: int) -> None:
        self.mav, _sink = encoder(source_system, source_component, sink)
        # A mavfile's initial vehicle state: fixed-wing, generic autopilot.
        self._vehicle_type = mavlink.MAV_TYPE_FIXED_WING
        self._autopilot = mavlink.MAV_AUTOPILOT_GENERIC

    def observe(self, message: mavlink.MAVLink_message) -> None:
        """Called by the reader for each message, before its dispatch.

        As ``mavfile.post_message``: every probable vehicle heartbeat (and
        ``HIGH_LATENCY2``) sets the type and autopilot; the latest one wins.
        """
        kind = message.get_type()
        if kind == "HIGH_LATENCY2" or (
            kind == "HEARTBEAT" and mavutil.mavfile.probably_vehicle_heartbeat(None, message)
        ):
            self._vehicle_type = message.type
            self._autopilot = message.autopilot

    def mode_mapping(self) -> dict[str, int] | None:
        """``mavfile.mode_mapping``: PX4's map, else the type's static table."""
        if self._autopilot == mavlink.MAV_AUTOPILOT_PX4:
            return mavutil.px4_map
        return mavutil.mode_mapping_byname(self._vehicle_type)

    def close(self) -> None:
        """Nothing to release: the link, not the connection, owns the bytes."""


class LockstepBusReader(MavBusReader):
    """The bus reader on per-tick bytes instead of ``recv_match``."""

    def __init__(
        self,
        inbound: BatonQueue,
        connection: LockstepConnection,
        device: str,
        membership: MavBusMembership,
        heartbeats: MavBusHeartbeats,
        lifecycle: MavBusLifecycle,
        cooperative_yield: Callable[[], None],
    ) -> None:
        super().__init__(
            connection, device, membership, heartbeats, lifecycle.stop_event,
            cooperative_yield=cooperative_yield, on_exit=lifecycle.reader_exited,
        )
        self._inbound = inbound
        self._decoder = StreamDecoder()

    def join(self, timeout_s: float | None = None) -> bool:
        """Close sets the stop, then joins: wake a reader parked on input first."""
        self._inbound.put(b"")
        return super().join(timeout_s)

    def _read_until_stopped(self) -> None:
        while not self._stop.is_set():
            for message in self._decoder.feed(self._inbound.get()):
                if self._stop.is_set():
                    return
                self._connection.observe(message)
                try:
                    self._dispatch(message)
                except Exception:
                    log.exception("MavBus %s reader error", self._device)
                finally:
                    # Per packet, unlike the wall reader's per-batch yield: a
                    # woken consumer runs before the next packet at the same
                    # tick, so a burst cannot evict the answer it waits for.
                    self._cooperative_yield()


class LockstepBus:
    """One open of a lockstep link; :class:`LockstepLink` creates it."""

    def __init__(
        self,
        baton: BatonRuntime,
        device: str,
        connection: LockstepConnection,
        unregister: Callable[[], None],
    ) -> None:
        self.device = device
        self._conn = connection
        self._send_lock = rt.RLock()
        self._heartbeats = MavBusHeartbeats()
        self._membership = MavBusMembership()
        self._inbound = BatonQueue(baton)
        self.lifecycle = MavBusLifecycle(connection, device, self._membership, unregister)
        reader = LockstepBusReader(
            self._inbound, connection, device, self._membership, self._heartbeats, self.lifecycle,
            lambda: yield_now(baton),
        )
        self.lifecycle.bind_reader(reader)
        reader.start()

    @property
    def conn(self) -> LockstepConnection:
        return self._conn

    @property
    def send_lock(self) -> rt.LockPort:
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
        return self.lifecycle.is_closed

    @property
    def is_closing(self) -> bool:
        return self.lifecycle.is_closing

    def wait_closed(self, timeout: float | None = None) -> bool:
        return self.lifecycle.wait_closed(timeout)

    def reserve(self, system_id: int) -> MavBusLease:
        self.lifecycle.change(lambda: self._membership.reserve(system_id))
        return MavBusLease(self.lifecycle, self._membership, system_id)

    def close(self) -> None:
        self.lifecycle.close()

    def wait_heartbeat(self, target_sysid: int, timeout: float = 30.0) -> bool:
        return self._heartbeats.wait_for(target_sysid, timeout)

    def deliver(self, data: bytes) -> None:
        """The coordinator's input path: one tick's bytes, in order."""
        self._inbound.deliver(data)


__all__ = ["LockstepBus", "LockstepBusReader", "LockstepConnection"]
