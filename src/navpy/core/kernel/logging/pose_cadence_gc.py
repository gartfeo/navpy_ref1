"""Opt-in garbage-collection record for the pose-cadence CSVs (stream ``gc``).

Each collection appends one row to a buffer made at install time, which the
pose-cadence flusher writes and frees like any other stream. Installed only by
the wall entry point (``navpy.main``): host collections and host clocks would
make a lockstep run's files differ run to run.
"""
from __future__ import annotations

import gc
from collections.abc import Callable

from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.logging import pose_cadence_debug


class GcRecord:
    """A ``gc.callbacks`` entry that appends one row per collection.

    It runs inside the collector, on whichever thread allocated, so it only
    reads two clocks and appends to a list made before install: no lock, no
    logging, no I/O. ``dur_ms`` runs from the start to the stop callback, so it
    also holds other callbacks' time: an upper bound on the collection.
    """

    def __init__(
            self,
            rows: list[tuple],
            wall_time: Callable[[], float] = rt.time,
            perf_counter: Callable[[], float] = rt.perf_counter,
    ) -> None:
        self._rows = rows
        self._wall_time = wall_time
        self._perf_counter = perf_counter
        self._wall_s = 0.0
        self._perf_s: float | None = None

    def __call__(self, phase: str, info: dict) -> None:
        if phase == "start":
            self._wall_s = self._wall_time()
            self._perf_s = self._perf_counter()
            return
        if self._perf_s is None:
            return
        dur_ms = (self._perf_counter() - self._perf_s) * 1000.0
        self._rows.append((
            self._wall_s,
            self._perf_s,
            info["generation"],
            dur_ms,
            info["collected"],
            info["uncollectable"],
        ))
        self._perf_s = None


def install_gc_record(sys_id: int) -> None:
    """Record this process's collections under ``sys_id``.

    A no-op unless pose-cadence recording is enabled.
    """
    if not pose_cadence_debug.ENABLED:
        return
    gc.callbacks.append(GcRecord(pose_cadence_debug.gc_rows(sys_id)))


def uninstall_gc_record() -> None:
    """Stop recording, before the final flush, so no row lands after it."""
    for record in [cb for cb in gc.callbacks if isinstance(cb, GcRecord)]:
        gc.callbacks.remove(record)


__all__ = ["GcRecord", "install_gc_record", "uninstall_gc_record"]
