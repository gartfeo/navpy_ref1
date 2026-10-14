"""MAVLink message counts per link and direction, for the run journal.

The slice's assignment gate reads them (``lockstep_slice.md``, Slice checks):
no task-dispatch, advert or peer-relay message may cross a link. The counts
derive from the router path's packet records (``lockstep_fleet.md``, Tally;
:meth:`MavlinkTally.count`): every packet is framed once by the router's
decoder, so a message split across ticks counts once, in the tick it
completes.

Counts are keyed by message id and identity, ``<id>:<src_sys>.<src_comp>><target>``
(``coordinator_packets.packet_key``; ``target`` is the router entry table's
``target_system``, ``-`` if it has none), so a gate can tell the owner's
self-assignment publication (``self_assignment_publisher.py``: a
``TASK_ASSIGN_REQUEST`` from its companion to its own system) from a
dispatch, and the owner's and the ground station's traffic from a relayed
peer's. Each tick record gets the NAVLINK messages counted in that tick
(sparse: only a nonzero count is written), which places the confirmation
response sends on the tick grid; the run's end adds the totals of every key.

The links are each SITL's ports and the router's backend endpoint
(:data:`ROUTER_LINK`). The directions are a frame's egress (``from_sitl``,
including the parked frame k_F+1), a reply's ingress (``to_sitl``, a packet
counted at the reply that admits its last byte), the participants' drain
egress (``drain_to_sitl``: never delivered, still counted so nothing crosses
a link unseen), ``routed_out`` (router copies into a link),
``router_dropped`` (packets from a link no endpoint accepted) and
``router_skipped_bytes`` (bytes from a link that are no packet, or fail
their CRC; the autopilot's boot text).
"""

from __future__ import annotations

from collections import Counter
from collections.abc import Iterable
from dataclasses import dataclass

FROM_SITL = "from_sitl"
TO_SITL = "to_sitl"
DRAIN_TO_SITL = "drain_to_sitl"
ROUTED_OUT = "routed_out"
ROUTER_DROPPED = "router_dropped"
ROUTER_SKIPPED_BYTES = "router_skipped_bytes"
DIRECTIONS = (FROM_SITL, TO_SITL, DRAIN_TO_SITL, ROUTED_OUT, ROUTER_DROPPED, ROUTER_SKIPPED_BYTES)
#: The tally link of the router's backend endpoint.
ROUTER_LINK = "router.backend"
#: The NavPy dialect's NAVLINK message ids (``navlink`` in the MAVLink XML).
NAVLINK_IDS = range(25_000, 26_000)
#: ``target`` of a message without a ``target_system`` field.
NO_TARGET = "-"


@dataclass(frozen=True)
class MessageKey:
    message_id: int
    source_system: int
    source_component: int
    target_system: int | None


def parse_key(key: str) -> MessageKey:
    message_id, identity = key.split(":", 1)
    source, target = identity.split(">", 1)
    system, component = source.split(".", 1)
    return MessageKey(int(message_id), int(system), int(component),
                      None if target == NO_TARGET else int(target))


def message_id(key: str) -> int:
    return int(key.split(":", 1)[0])


class _Counter:
    """One link direction's counts."""

    def __init__(self) -> None:
        self.totals: Counter[str] = Counter()
        self.skipped_bytes = 0


class MavlinkTally:
    def __init__(self, devices: tuple[str, ...]) -> None:
        self._devices = devices
        self._counters = {(device, direction): _Counter()
                          for device in devices for direction in DIRECTIONS}

    def count(self, record: dict, device: str, direction: str, keys: Iterable[str] = (),
              skipped_bytes: int = 0) -> None:
        """Count framed packets (their ``packet_key``) and skipped bytes; note the
        NAVLINK counts in ``record`` (a tick record's data, or the parked frame's record)."""
        counter = self._counters[(device, direction)]
        counts = Counter(keys)
        counter.totals.update(counts)
        counter.skipped_bytes += skipped_bytes
        _note_navlink(record, device, direction, counts)

    def totals(self) -> dict[str, object]:
        """Every message id's count and the skipped bytes, per link and direction."""
        return {
            device: {
                direction: {
                    "messages": dict(sorted(self._counters[(device, direction)].totals.items())),
                    "skipped_bytes": self._counters[(device, direction)].skipped_bytes,
                }
                for direction in DIRECTIONS
            }
            for device in self._devices
        }


def _note_navlink(record: dict, device: str, direction: str, counts: Counter[str]) -> None:
    """Add a tick's NAVLINK counts to ``record`` (sparse: nonzero only)."""
    navlink = {key: n for key, n in sorted(counts.items()) if message_id(key) in NAVLINK_IDS}
    if not navlink:
        return
    noted = record.setdefault("navlink", {}).setdefault(device, {}).setdefault(direction, {})
    for key, n in navlink.items():
        noted[key] = noted.get(key, 0) + n
    record["navlink"][device][direction] = dict(sorted(noted.items()))


__all__ = [
    "DIRECTIONS", "DRAIN_TO_SITL", "FROM_SITL", "MavlinkTally", "MessageKey", "NAVLINK_IDS",
    "NO_TARGET", "ROUTED_OUT", "ROUTER_DROPPED", "ROUTER_LINK",
    "ROUTER_SKIPPED_BYTES", "TO_SITL", "message_id", "parse_key",
]
