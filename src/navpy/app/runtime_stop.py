"""Acknowledged clean stop for GCS-managed NavPy processes.

With ``--stop-on-stdin`` the managing GCS writes one ``NAVPY_STOP_V1`` line on
the child's stdin. The run loop then ends, the normal cleanup joins every
producer and drains the log worker, the pose-cadence buffers are flushed, and
``NAVPY_STOPPED_V1`` is printed on stdout as the ack. A cleanup failure prints
no ack, so the GCS treats the stop as missed.

Under lockstep (``--stop-on-lifecycle``) no thread reads stdin: the stop is
the coordinator's lifecycle ``stop{id, generation}`` record, latched by a
:class:`LifecycleStopRequest` with the same ``requested()`` surface, so the
run loop's stop path is unchanged. Its ack is a ``stopped{id}`` event in the
participant's ``DONE`` (``lockstep_slice.md``, "Participant stop").
"""

from __future__ import annotations

from argparse import ArgumentParser
from collections.abc import Callable
from typing import TextIO

from navpy.core.kernel import runtime_time as rt


NAVPY_STOP_REQUEST = "NAVPY_STOP_V1"
NAVPY_STOPPED_MARKER = "NAVPY_STOPPED_V1"


class StdinStopRequest:
    """Latches once the stop-request line arrives on *stream*."""

    def __init__(self, stream: TextIO) -> None:
        self._stream = stream
        self._requested = rt.Event()
        self._thread = rt.Thread(
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


class LifecycleStopRequest:
    """Latches from the lifecycle ``stop`` entry, or the drain ``end``.

    :meth:`deliver_stop` and :meth:`deliver_end` run on the participant
    loop's thread in the tick's input phase, so they only deliver; a stop or
    end that arrived before this request existed is passed in as
    ``held_stop_id`` / ``held_end``. The ack publishes ``stopped{id}``
    through ``publish_stopped`` after the strict flush; a drain end has no
    stop id and so no ack event.
    """

    def __init__(
        self,
        publish_stopped: Callable[[tuple[int, int]], None],
        *,
        held_stop_id: tuple[int, int] | None = None,
        held_end: bool = False,
    ) -> None:
        self._publish_stopped = publish_stopped
        self._requested = rt.Event()
        self._stop_id = held_stop_id
        if held_stop_id is not None or held_end:
            self._requested.set()

    def deliver_stop(self, stop_id: tuple[int, int]) -> None:
        if self._stop_id is None:
            self._stop_id = stop_id
        self._requested.deliver_set()

    def deliver_end(self) -> None:
        self._requested.deliver_set()

    def requested(self) -> bool:
        return self._requested.is_set()

    def acknowledge(self) -> None:
        """``acknowledge_stop`` with the ack as an event: strict flush, then ``stopped``."""
        flush_pose_cadence()
        if self._stop_id is not None:
            self._publish_stopped(self._stop_id)


def flush_pose_cadence() -> None:
    """Strictly flush the pose-cadence buffers, the one output cleanup does not own."""
    # Imported here: under its debug env the module starts a flush thread on
    # import, which the GCS backend (it imports the markers) must not get.
    from navpy.core.kernel.logging import pose_cadence_debug

    # stop_flusher joins the periodic flush and drops its atexit hook, so the
    # final dump below is the last and only concurrent-free write.
    pose_cadence_debug.stop_flusher()
    pose_cadence_debug.dump_all(strict=True)


def acknowledge_stop() -> None:
    """Flush what cleanup does not own, then publish the ack."""
    flush_pose_cadence()
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
    parser.add_argument(
        "--stop-on-lifecycle",
        action="store_true",
        help=(
            "lockstep participant: stop cleanly on the coordinator's lifecycle "
            "stop record and acknowledge it with a stopped event"
        ),
    )


__all__ = [
    "LifecycleStopRequest",
    "NAVPY_STOPPED_MARKER",
    "NAVPY_STOP_REQUEST",
    "StdinStopRequest",
    "acknowledge_stop",
    "add_args",
    "flush_pose_cadence",
]
