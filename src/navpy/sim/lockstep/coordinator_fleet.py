"""The coordinator's SITL fleet: launch, frames, boot latch, replies, parking, stop.

``lockstep_fleet.md``, "Fleet barrier". The SITLs are data: the manifest's
``sitl_ports`` names each SITL's port devices, in SITL order, and the
links arrive in the same order. Per tick the fleet reads frame k from every
SITL (each checked, then the fleet clock), opens the tick's exchange
deadline, and sends reply k to every SITL; the reply to SITL 1 anchors the
next tick's deadline. The launch (:meth:`SitlFleet.launch`) anchors the
first: the launcher calls ``deadline.anchor()`` at SITL 1's ``go`` and draws
the remaining launches on frame 1's deadline. Parking reads each SITL's
parked frame k_F+1 in order, with every check, and replies to none; then
the deadline closes (drain has no SITL exchange).

Journal shape: a one-SITL run keeps the slice's records (a frame or reply
hash is one string, the parked record one frame) until step 10c
re-baselines it (owner Q1); a fleet journals a list in SITL order
(:meth:`SitlFleet.per_sitl`).
"""

from __future__ import annotations

import logging
from collections.abc import Callable, Sequence
from typing import Any, Protocol

from navpy.sim.lockstep.coordinator_deadline import ExchangeDeadline
from navpy.sim.lockstep.coordinator_frames import FleetFrames, SitlLink
from navpy.sim.lockstep.coordinator_journal import sha256
from navpy.sim.lockstep.coordinator_latches import BootLatch
from navpy.sim.lockstep.coordinator_mavlink import FROM_SITL
from navpy.sim.lockstep.coordinator_types import HostFault, Manifest
from navpy.sim.lockstep.step_wire import Frame

_log = logging.getLogger(__name__)


class SitlHost(Protocol):
    def snapshot(self, parked_tick: int) -> None:
        """Take the SITL artifacts up to and including the parked frame's row."""

    def stop(self) -> None:
        """Stop the SITL (and relay) by ownership."""


class Sitl(SitlLink, SitlHost, Protocol):
    """One serial-step SITL behind its relay: its link and its host side."""


def anchor_now(deadline: ExchangeDeadline) -> None:
    """The default launch: the SITLs already run (launched before the
    coordinator), so frame 1's deadline counts from now."""
    deadline.anchor()


class SitlFleet:
    def __init__(self, manifest: Manifest, sitls: Sequence[Sitl], deadline: ExchangeDeadline, *,
                 launch: Callable[[ExchangeDeadline], None] = anchor_now) -> None:
        self._sitls = tuple(sitls)
        self._deadline = deadline
        self._launch = launch
        self._frames = FleetFrames(self._sitls, manifest.sitl_ports, manifest.max_frame_gap_us,
                                   deadline)
        self.boot = BootLatch(len(self._sitls), manifest.boot_budget_us)
        self.tally = self._frames.tally

    @property
    def size(self) -> int:
        return len(self._sitls)

    def per_sitl(self, values: list[Any]) -> Any:
        """A per-SITL journal value: the slice's single value for one SITL."""
        return values[0] if self.size == 1 else values

    def launch(self) -> None:
        self._launch(self._deadline)

    def read(self, k: int) -> tuple[list[Frame], list[bytes]]:
        frames = self._frames.read(k)
        return [frame for frame, _raw in frames], [raw for _frame, raw in frames]

    def egress(self, frames: list[Frame]) -> dict[str, bytes]:
        return self._frames.egress(frames)

    def reply(self, k: int, ingress: dict[str, bytes]) -> list[bytes]:
        return self._frames.reply(k, ingress)

    def park(self, k: int) -> dict[str, Any]:
        """Read every SITL's parked frame k (k_F+1), checked; never replied to.

        Returns its journal record; its egress is tallied (it completes
        messages too), never routed.
        """
        frames, raws = self.read(k)
        self._deadline.close()
        parked: dict[str, Any] = {"tick": frames[0].tick}
        shapes = [{"length": len(raw), "sha256": sha256(raw)} for raw in raws]
        if self.size == 1:
            parked.update(shapes[0])
        else:
            parked["frames"] = shapes
        self.tally.feed(parked, FROM_SITL, self.egress(frames))
        return parked

    def snapshot(self, parked_tick: int) -> None:
        for sitl in self._sitls:
            sitl.snapshot(parked_tick)

    def stop(self) -> None:
        """Stop every SITL by ownership; each is tried whatever the others do."""
        failures = []
        for index, sitl in enumerate(self._sitls, start=1):
            try:
                sitl.stop()
            except Exception as error:  # noqa: BLE001 - every SITL is still stopped
                _log.exception("stopping SITL %d failed", index)
                failures.append(f"SITL {index}: {type(error).__name__}: {error}")
        if failures:
            raise HostFault("teardown_failed", "; ".join(failures))


__all__ = ["Sitl", "SitlFleet", "SitlHost", "anchor_now"]
