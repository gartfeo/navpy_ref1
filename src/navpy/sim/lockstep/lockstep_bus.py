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
  With the baton's ``reader_budget`` (dev sim step 6) it dispatches at most
  ``packets_per_tick`` per tick, yields once per ``batch``, and carries the
  rest over, in order, to the next tick.
- **Outbound:** :class:`LockstepConnection` is the encoder (a ``MAVLink`` on
  the link's byte sink, with the product's source ids) plus ``mode_mapping``;
  the link drains the sink into ``DONE``.
- **Heartbeats** are admitted by the wall ``MavBusHeartbeats``.
"""

from __future__ import annotations

import logging
from collections import deque
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
from navpy.sim.lockstep.baton_parts import ReaderBudget
from navpy.sim.lockstep.baton_queue import BatonQueue
from navpy.sim.lockstep.baton_sync import yield_now
from navpy.sim.lockstep.mavlink_wire import ByteSink, StreamDecoder, encoder

log = logging.getLogger(__name__)

#: A wait that is due at the next tick: any positive wait is (ticks are about 20 ms).
NEXT_TICK_S = 1e-6


class ReaderService:
    """When a lockstep reader yields: after each packet, or by its ``ReaderBudget``."""

    def __init__(self, baton: BatonRuntime, budget: ReaderBudget | None) -> None:
        self._baton = baton
        self._budget = budget
        self._tick_us: int | None = None
        self._served = 0

    def before_packet(self, stop: rt.Event) -> None:
        """Wait for the next tick once this tick's packets are served; ``stop`` ends the wait,
        so a close in this tick still joins the reader."""
        if self._budget is None:
            return
        if self._baton.now_us != self._tick_us:
            self._tick_us, self._served = self._baton.now_us, 0
        if self._served == self._budget.packets_per_tick:
            stop.wait(NEXT_TICK_S)
            self._tick_us, self._served = self._baton.now_us, 0

    def after_packet(self) -> None:
        if self._budget is None:
            yield_now(self._baton)
            return
        self._served += 1
        if self._served % self._budget.batch == 0:
            yield_now(self._baton)

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
        service: ReaderService,
    ) -> None:
        super().__init__(
            connection, device, membership, heartbeats, lifecycle.stop_event,
            cooperative_yield=service.after_packet, on_exit=lifecycle.reader_exited,
        )
        self._inbound = inbound
        self._decoder = StreamDecoder()
        self._service = service

    def join(self, timeout_s: float | None = None) -> bool:
        """Close sets the stop, then joins: wake a reader parked on input first."""
        self._inbound.put(b"")
        return super().join(timeout_s)

    def _read_until_stopped(self) -> None:
        pending: deque[mavlink.MAVLink_message] = deque()
        while not self._stop.is_set():
            if not pending:
                pending.extend(self._decoder.feed(self._inbound.get()))
                continue
            self._service.before_packet(self._stop)
            if self._stop.is_set():
                return
            message = pending.popleft()
            self._connection.observe(message)
            try:
                self._dispatch(message)
            except Exception:
                log.exception("MavBus %s reader error", self._device)
            finally:
                # Per packet by default, unlike the wall reader's per-batch
                # yield: with first-in first-out order a woken consumer runs
                # before the next packet at the same tick, so a burst cannot
                # evict the answer it waits for. A reader budget batches.
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
            ReaderService(baton, baton.runnable.scheduling.reader_budget),
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


__all__ = ["LockstepBus", "LockstepBusReader", "LockstepConnection", "NEXT_TICK_S", "ReaderService"]
