"""The SITL link: tick step 1 (read and check frame k, the parked one too) and step 6.

EOF, a malformed frame or the wrong tick is a coordinator fault. A nonzero
out-of-loop advance count is a host fault. The grid check: ``source_us``
must rise strictly, by at most the manifest's ``max_frame_gap_us`` (Δ_max),
from the previous frame; today's decoder accepts any header
(``step_wire.py``), so this is the only place time is checked.
"""

from __future__ import annotations

from typing import Protocol

from navpy.sim.lockstep.coordinator_types import CoordinatorFault, HostFault
from navpy.sim.lockstep.step_wire import Frame, StepWireError, pack_reply, unpack_frame


class SitlLink(Protocol):
    def recv_frame(self) -> bytes: ...

    def send_reply(self, reply: bytes) -> None: ...


class FrameReader:
    def __init__(self, link: SitlLink, max_frame_gap_us: int) -> None:
        self._link = link
        self._max_gap_us = max_frame_gap_us
        self._last_us: int | None = None

    def read(self, k: int) -> tuple[Frame, bytes]:
        try:
            raw = self._link.recv_frame()
            frame = unpack_frame(raw, expected_tick=k)
        except (ConnectionError, OSError, StepWireError) as error:
            raise CoordinatorFault("frame_invalid", f"SITL frame {k}: {error}", tick=k) from error
        self.check(frame)
        return frame, raw

    def reply(self, k: int, ingress: tuple[bytes, ...]) -> bytes:
        """Tick step 6: reply k; returns its raw bytes for the journal."""
        try:
            raw = pack_reply(k, ingress)
            self._link.send_reply(raw)
        except (ConnectionError, OSError, StepWireError) as error:
            raise CoordinatorFault("reply_failed", f"reply {k}: {error}", tick=k) from error
        return raw

    def check(self, frame: Frame) -> None:
        if frame.out_of_loop_advances:
            raise HostFault("out_of_loop_advance",
                            f"out-of-loop clock advances: {frame.out_of_loop_advances}", tick=frame.tick)
        if self._last_us is not None:
            gap = frame.source_us - self._last_us
            if gap <= 0 or gap > self._max_gap_us:
                raise CoordinatorFault(
                    "grid", f"grid check at tick {frame.tick}: source_us step {gap} us outside "
                    f"(0, {self._max_gap_us}]", tick=frame.tick)
        self._last_us = frame.source_us


__all__ = ["FrameReader", "SitlLink"]
