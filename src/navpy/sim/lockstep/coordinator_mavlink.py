"""Decoded MAVLink message counts per link and direction, for the run journal.

The slice's assignment gate reads them (``lockstep_slice.md``, Slice checks):
no task-dispatch, advert or peer-relay message may cross either link. The
coordinator sees every byte of both links in tick steps 1 (``from_sitl``,
the frame's egress) and 6 (``to_sitl``, the reply's ingress), so it decodes
them here with one persistent decoder per link and direction (a message may
be split across ticks).

Counts are keyed by message id and identity, ``<id>:<src_sys>.<src_comp>><target>``
(:func:`count_key`; ``target`` is the message's ``target_system``, ``-`` if
it has none; a message outside the dialect is keyed by its raw frame's
id and source), so a gate can tell the owner's self-assignment publication
(``self_assignment_publisher.py``: a ``TASK_ASSIGN_REQUEST`` from its
companion to its own system) from a dispatch, and the owner's and the
ground station's traffic from a relayed peer's. Each tick record gets the
NAVLINK messages decoded in that tick (sparse: only a nonzero count is
written), which places the confirmation response sends on the tick grid;
the run's end adds the totals of every key. Bytes that are no MAVLink frame
(the autopilot's boot text) are counted as skipped.

The directions are the frame's egress (``from_sitl``, including the parked
frame k_F+1), the reply's ingress (``to_sitl``) and the participants' drain
egress (``drain_to_sitl``: written during drain, never delivered, still
counted so nothing crosses a link unseen).
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass

from pymavlink.dialects.v20 import ardupilotmega as mavlink

FROM_SITL = "from_sitl"
TO_SITL = "to_sitl"
DRAIN_TO_SITL = "drain_to_sitl"
DIRECTIONS = (FROM_SITL, TO_SITL, DRAIN_TO_SITL)
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


def _unknown_identity(message: mavlink.MAVLink_unknown) -> tuple[int, int, int]:
    """A message outside the dialect has no decoded header: its id and source
    come from its raw frame (v1: ``FE len seq sys comp id``; v2: ``FD len
    incompat compat seq sys comp id[3]``)."""
    frame = message.data
    if frame[0] == mavlink.PROTOCOL_MARKER_V2:
        return int.from_bytes(frame[7:10], "little"), frame[5], frame[6]
    return frame[5], frame[3], frame[4]


def count_key(message: mavlink.MAVLink_message) -> str:
    if isinstance(message, mavlink.MAVLink_unknown):
        message_id, system, component = _unknown_identity(message)
        return f"{message_id}:{system}.{component}>{NO_TARGET}"
    target = getattr(message, "target_system", None)
    return (f"{message.get_msgId()}:{message.get_srcSystem()}.{message.get_srcComponent()}"
            f">{NO_TARGET if target is None else target}")


def parse_key(key: str) -> MessageKey:
    message_id, identity = key.split(":", 1)
    source, target = identity.split(">", 1)
    system, component = source.split(".", 1)
    return MessageKey(int(message_id), int(system), int(component),
                      None if target == NO_TARGET else int(target))


def message_id(key: str) -> int:
    return int(key.split(":", 1)[0])


class _Counter:
    """One link direction: a persistent parser and its counts."""

    def __init__(self) -> None:
        self._parser = mavlink.MAVLink(None)
        self._parser.robust_parsing = True
        self.totals: Counter[str] = Counter()
        self.skipped_bytes = 0

    def feed(self, data: bytes) -> Counter[str]:
        tick: Counter[str] = Counter()
        if not data:
            return tick
        for message in self._parser.parse_buffer(data) or ():
            if message.get_type() == "BAD_DATA":
                self.skipped_bytes += len(message.get_msgbuf())
                continue
            tick[count_key(message)] += 1
        self.totals.update(tick)
        return tick


class MavlinkTally:
    def __init__(self, devices: tuple[str, ...]) -> None:
        self._devices = devices
        self._counters = {(device, direction): _Counter()
                          for device in devices for direction in DIRECTIONS}

    def feed(self, record: dict, direction: str, blocks: dict[str, bytes]) -> None:
        """Decode one tick's bytes per device; note its NAVLINK counts in ``record``
        (a tick record's data, or the parked frame's record)."""
        for device in self._devices:
            counts = self._counters[(device, direction)].feed(blocks.get(device, b""))
            navlink = {key: n for key, n in sorted(counts.items()) if message_id(key) in NAVLINK_IDS}
            if navlink:
                record.setdefault("navlink", {}).setdefault(device, {})[direction] = navlink

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


__all__ = [
    "DIRECTIONS", "DRAIN_TO_SITL", "FROM_SITL", "MavlinkTally", "MessageKey", "NAVLINK_IDS",
    "NO_TARGET", "TO_SITL", "count_key", "message_id", "parse_key",
]
