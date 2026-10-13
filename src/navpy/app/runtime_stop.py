"""Acknowledged clean stop for GCS-managed NavPy processes.

With ``--stop-on-stdin`` the managing GCS writes one ``NAVPY_STOP_V1`` line on
the child's stdin. The run loop then ends, the normal cleanup joins every
producer and drains the log worker, the pose-cadence buffers are flushed, and
``NAVPY_STOPPED_V1`` is printed on stdout as the ack. A cleanup failure prints
no ack, so the GCS treats the stop as missed.
"""

from __future__ import annotations

import threading
from argparse import ArgumentParser
from typing import TextIO


NAVPY_STOP_REQUEST = "NAVPY_STOP_V1"
NAVPY_STOPPED_MARKER = "NAVPY_STOPPED_V1"


class StdinStopRequest:
    """Latches once the stop-request line arrives on *stream*."""

    def __init__(self, stream: TextIO) -> None:
        self._stream = stream
        self._requested = threading.Event()
        self._thread = threading.Thread(
            target=self._watch,
            name="navpy-stop-request",
            daemon=True,
        )

    def start(self) -> None:
        self._thread.start()

    def requested(self) -> bool:
        return self._requested.is_set()

    def _watch(self) -> None:
        # EOF (the GCS went away) is not a request: the GCS's job object owns
        # that case.
        for line in self._stream:
            if line.strip() == NAVPY_STOP_REQUEST:
                self._requested.set()
                return


def acknowledge_stop() -> None:
    """Flush what cleanup does not own, then publish the ack."""
    # Imported here: under its debug env the module starts a flush thread on
    # import, which the GCS backend (it imports the markers) must not get.
    from navpy.core.kernel.logging import pose_cadence_debug

    # stop_flusher joins the periodic flush and drops its atexit hook, so the
    # final dump below is the last and only concurrent-free write.
    pose_cadence_debug.stop_flusher()
    pose_cadence_debug.dump_all(strict=True)
    print(NAVPY_STOPPED_MARKER, flush=True)


def add_args(parser: ArgumentParser) -> None:
    parser.add_argument(
        "--stop-on-stdin",
        action="store_true",
        help=(
            f"stop cleanly when {NAVPY_STOP_REQUEST} arrives on stdin and "
            f"print {NAVPY_STOPPED_MARKER} once everything is flushed"
        ),
    )


__all__ = [
    "NAVPY_STOPPED_MARKER",
    "NAVPY_STOP_REQUEST",
    "StdinStopRequest",
    "acknowledge_stop",
    "add_args",
]
