"""LockstepLink: one logical device of a participant, both of its sides.

The product opens it through the bus seam (``MavBusOpener``): NavPy's
``create_vehicle(open_bus=link.open)`` and the backend's
``VehicleManager(open_bus=link.open)``. The device string is the logical
device (``lockstep:serial0`` for NavPy, ``lockstep:serial1`` for the backend
monitor), so it reaches every place a device string does.

The participant loop is the other side. Per tick it calls :meth:`deliver` with
the ``TICK``'s port bytes, in the tick's input phase, and :meth:`drain_egress`
for the ``DONE``. :attr:`state` is what it journals: releasing the last
reservation closes the bus (the reader stops first, then the bus counts as
closed), and the link reports ``closed``.

Opens share one bus per device as ``MavBus.get_or_create`` does (the same
registry, owned by the link instead of the process). A reopen after a close
is a new bus: a fresh decoder and fresh encoder sequence numbers, as a
reopened ``mavfile`` has. Bytes delivered while no bus is open have no reader
and are lost, as on a UART; they are counted.
"""

from __future__ import annotations

from navpy.plugins.vehicle.ardupilot.mav_bus_registry import MavBusRegistry, MavBusRegistryPorts
from navpy.sim.lockstep.baton import BatonRuntime
from navpy.sim.lockstep.lockstep_bus import LockstepBus, LockstepConnection
from navpy.sim.lockstep.mavlink_wire import ByteSink

#: A device string names a logical device under this prefix.
LOGICAL_DEVICE_PREFIX = "lockstep:"


class LockstepLink:
    def __init__(self, baton: BatonRuntime, device: str) -> None:
        if not device.startswith(LOGICAL_DEVICE_PREFIX):
            raise ValueError(f"{device!r} is not a logical device ({LOGICAL_DEVICE_PREFIX}...)")
        self.device = device
        self._baton = baton
        self._egress = ByteSink()
        self._bus: LockstepBus | None = None
        self._registry: MavBusRegistry[LockstepBus] = MavBusRegistry(MavBusRegistryPorts(
            identity=lambda bus: bus,
            return_if_open=lambda bus, action: bus.lifecycle.return_if_open(action),
            wait_closed=lambda bus, timeout: bus.wait_closed(timeout),
        ))
        #: Ingress bytes that arrived while no bus was open.
        self.lost_ingress_bytes = 0

    def open(
        self,
        device: str,
        baud: int = 115200,
        source_system: int = 1,
        source_component: int = 0,
    ) -> LockstepBus:
        """The product's opener (``MavBusOpener``); ``baud`` has no meaning here."""
        del baud
        if device != self.device:
            raise ValueError(f"lockstep link {self.device} cannot open {device!r}")

        def create() -> LockstepBus:
            connection = LockstepConnection(self._egress, source_system, source_component)
            bus = LockstepBus(
                self._baton, device, connection, lambda: self._registry.remove(device, bus),
            )
            self._bus = bus
            return bus

        return self._registry.get_or_create(device, create)

    def deliver(self, data: bytes) -> None:
        """Coordinator, in the tick's input phase: the ``TICK``'s port bytes."""
        bus = self._bus
        if bus is None or bus.is_closing or bus.is_closed:
            self.lost_ingress_bytes += len(data)
            return
        bus.deliver(data)

    def drain_egress(self) -> bytes:
        """Coordinator, after quiescence: what the product wrote, for ``DONE``."""
        return self._egress.drain()

    @property
    def state(self) -> str:
        """``unopened``, ``open``, ``closing`` (a close still joining) or ``closed``."""
        if self._bus is None:
            return "unopened"
        if self._bus.is_closed:
            return "closed"
        return "closing" if self._bus.is_closing else "open"


__all__ = ["LOGICAL_DEVICE_PREFIX", "LockstepLink"]
