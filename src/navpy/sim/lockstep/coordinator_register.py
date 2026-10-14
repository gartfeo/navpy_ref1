"""Holding the clock for a spawned participant's ``REGISTER``.

One absolute wall deadline covers the accept and the whole ``REGISTER``
message. ``REGISTER`` must match the recorded spawn, and the recorded child
must still run when it arrives (the host-side identity check).
"""

from __future__ import annotations

import socket
import time

from navpy.sim.lockstep.coordinator_spawn import HostChild, SpawnRecord, validate_register
from navpy.sim.lockstep.coordinator_types import HostFault
from navpy.sim.lockstep.part_socket import PartListener, PartSocket
from navpy.sim.lockstep.part_wire import PartWireError, Register


def await_register(listener: PartListener, record: SpawnRecord, child: HostChild,
                   budget_s: float) -> PartSocket:
    deadline = time.monotonic() + budget_s
    try:
        link = listener.accept(budget_s)
        message, _raw = link.recv(deadline)
    except (socket.timeout, OSError, PartWireError) as error:
        raise HostFault("register_missed", f"spawn {record.spawn_id} did not register: {error}",
                        ordinal=record.ordinal) from error
    if not isinstance(message, Register):
        link.close()
        raise HostFault("register_missed", f"spawn {record.spawn_id} sent "
                        f"{type(message).__name__} first", ordinal=record.ordinal)
    try:
        validate_register(message, record)
    except HostFault:
        link.close()
        raise
    if child.wait(0) is not None:
        link.close()
        raise HostFault("register_exited", f"spawn {record.spawn_id} registered but its "
                        "process exited", ordinal=record.ordinal)
    return link


__all__ = ["await_register"]
