"""One link's confirmation images, sent one at a time at the image rate.

``docs/design/radio_budget.md`` round 7 (todos 116, 110). A queued image
returns at once; this sender's own thread sends its frames, each through the
image's ``run_unit`` (the round's exact fence, held for that frame only), and
after each frame waits the frame's wire time at the image rate, outside the
fence. So every image of the link shares one budget, and a resolved round's
image ends at its next frame.
"""

from __future__ import annotations

import logging
from collections.abc import Callable, Sequence
from dataclasses import dataclass

from pymavlink.dialects.v20.ardupilotmega import MAVLink_message

from navpy.core.kernel import runtime_time as rt
from navpy.core.ports.network_abc import ImageSendUnitRunner

#: MAVLink 2 framing around a payload: 10 header bytes and a 2-byte CRC (unsigned).
MAVLINK2_FRAMING_BYTES = 12

log = logging.getLogger(__name__)


def wire_bytes(message: MAVLink_message) -> int:
    """The message's MAVLink 2 frame size with its full payload."""
    return type(message).unpacker.size + MAVLINK2_FRAMING_BYTES


@dataclass(frozen=True)
class _Image:
    frames: Sequence[MAVLink_message]
    run_unit: ImageSendUnitRunner


class PacedImageSender:
    """Send queued images one at a time, each frame followed by its wire time."""

    def __init__(self, send: Callable[[MAVLink_message], None], bytes_per_s: float) -> None:
        if bytes_per_s <= 0:
            raise ValueError(f"image rate {bytes_per_s} B/s is not positive")
        self._send = send
        self._bytes_per_s = bytes_per_s
        self._lock = rt.Lock()
        #: Images waiting, by POI, in queue order; at most one per POI.
        self._waiting: dict[int, _Image] = {}
        self._wake = rt.Event()
        self._closed = rt.Event()
        self._thread = rt.Thread(target=self._run, daemon=True)
        with rt.detached():  # owned by this sender (closed by it)
            self._thread.start()

    def queue(self, poi_id: int, frames: Sequence[MAVLink_message], run_unit: ImageSendUnitRunner) -> None:
        """Queue an image; it replaces one still waiting for the same POI."""
        with self._lock:
            if self._closed.is_set():
                return
            self._waiting[poi_id] = _Image(tuple(frames), run_unit)
            self._wake.set()

    def close(self) -> None:
        with self._lock:
            self._closed.set()
            self._waiting.clear()
            self._wake.set()
        if rt.current_thread() is not self._thread:
            self._thread.join()

    def _run(self) -> None:
        while True:
            with self._lock:
                if self._closed.is_set():
                    return
                if not self._waiting:
                    self._wake.clear()
                    image = None
                else:
                    image = self._waiting.pop(next(iter(self._waiting)))
            if image is None:
                self._wake.wait()
                continue
            try:
                self._send_frames(image)
            except Exception:  # noqa: BLE001 - one image never stops the sender
                log.exception("confirmation image send failed")

    def _send_frames(self, image: _Image) -> None:
        for frame in image.frames:
            if self._closed.is_set() or not image.run_unit(lambda: self._send(frame)):
                return
            if self._closed.wait(wire_bytes(frame) / self._bytes_per_s):
                return


__all__ = ["MAVLINK2_FRAMING_BYTES", "PacedImageSender", "wire_bytes"]
