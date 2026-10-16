"""The emulated ZR10's unrequested 0x0D attitude frames: the 0x0D push and the 0x25 stream, per requester.

0x0D push (the real unit, gimbal-controller docs/PROTOCOL.md, rig 2026-09): the unit is silent until addressed, and
one 0x0D request starts a push of attitude frames at about 50.5 Hz to the requester. Here: ``attitude_push_hz``
(50, one push per plant step; 0 = request/reply only). A request still gets its own reply; a further request keeps
the push going on its schedule. Unmeasured, so not modelled: how long the push lasts without new requests (here it
lasts until the emulator stops), and whether the real reply differs from a push.

0x25 (data type 1 = attitude, frequency code 0..7 = 0, 2, 4, 5, 10, 20, 50, 100 Hz; 0 stops it): a stream at that
rate to the requester; its effect on the real unit is unmeasured.

At most one attitude frame per requester per step, whichever schedules are due.
"""
from __future__ import annotations

from tools.hitl.zr10_config import STEP_S

Address = tuple[str, int]

STREAM_HZ = {0: 0.0, 1: 2.0, 2: 4.0, 3: 5.0, 4: 10.0, 5: 20.0, 6: 50.0, 7: 100.0}   # 0x25 frequency codes
STREAM_ATTITUDE = 1
MAX_STREAM_HZ = 1.0 / STEP_S      # one frame per plant step at most: a faster rate would repeat states
_EPS_S = 1e-9


class AttitudeStreams:
    def __init__(self, push_hz: float) -> None:
        if not 0.0 <= push_hz <= MAX_STREAM_HZ:
            raise ValueError(f"attitude_push_hz must be in 0..{MAX_STREAM_HZ:g}")
        self.push_hz = push_hz
        self._streams: dict[Address, tuple[float, float]] = {}     # 0x25: requester -> (period_s, next_due_s)
        self._pushes: dict[Address, tuple[float, float]] = {}      # 0x0D push: requester -> (period_s, next_due_s)

    def set_stream(self, source: Address, data_type: int, code: int, now_s: float) -> None:
        """0x25 from ``source``; ValueError for a stream the emulator does not serve."""
        if data_type != STREAM_ATTITUDE or code not in STREAM_HZ:
            raise ValueError(f"stream type {data_type} code {code} is not served")
        hz = STREAM_HZ[code]
        if hz > MAX_STREAM_HZ:
            raise ValueError(f"{hz:g} Hz is faster than the {MAX_STREAM_HZ:g} Hz plant step")
        if hz == 0.0:
            self._streams.pop(source, None)
        else:
            self._streams[source] = (1.0 / hz, now_s + 1.0 / hz)

    def requested(self, source: Address, now_s: float) -> None:
        """A 0x0D request from ``source``: starts its push, or keeps a running one on its schedule."""
        if self.push_hz > 0.0 and source not in self._pushes:
            self._pushes[source] = (1.0 / self.push_hz, now_s + 1.0 / self.push_hz)

    def due(self, now_s: float) -> list[Address]:
        """The requesters owed an attitude frame at ``now_s`` (each once), their schedules moved on."""
        due: dict[Address, None] = {}
        for schedules in (self._pushes, self._streams):
            for source, (period_s, next_due_s) in list(schedules.items()):
                if next_due_s <= now_s + _EPS_S:
                    due[source] = None
                    while next_due_s <= now_s + _EPS_S:
                        next_due_s += period_s
                    schedules[source] = (period_s, next_due_s)
        return list(due)


__all__ = ["MAX_STREAM_HZ", "STREAM_ATTITUDE", "STREAM_HZ", "AttitudeStreams"]
