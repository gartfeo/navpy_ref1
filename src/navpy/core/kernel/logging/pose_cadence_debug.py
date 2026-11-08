"""Opt-in source-time observability for the pose->frame->observation->command
pipeline.

Started as a cadence instrument for the pose (ATTITUDE) stream; Phase 2a of the
10x-determinism plan extended it to every hop of the guidance data
path. Each hop records the SOURCE time of the data it consumed (the raw
autopilot boot clock for source-driven simulation frames) next to its receipt/
emission time, plus the drop/duplicate outcomes that were previously silent.
The evaluator aggregates the CSVs into cadence and age distributions
(``scripts/eval_certificate.py.source_time_summary``); nothing in this module
feeds back into control.

Enabled only when ``NAVPY_POSE_CADENCE_DEBUG`` is set to a non-empty value.
When the value looks like a directory path it is used verbatim; the sentinels
``1`` / ``on`` / ``true`` / ``yes`` select a default scratch directory. Disabled
by default with a single module-level boolean check on the hot path.

Event streams, keyed by ``(sys_id, stream)``, and their CSV headers:
``pose_cadence_streams``.

Rows buffer in memory (append-only lists; list.append is atomic in CPython). A
daemon thread flushes every ``_FLUSH_INTERVAL_S``, APPENDING only the rows added
since the last flush (O(new) per flush, so a long run never pays a growing
whole-file rewrite that could perturb the cadence it measures), then deletes
the written rows from memory, so a long run does not keep every row alive for
the garbage collector to walk (todo 171). A daemon flush --
not ``atexit`` -- is used because NavPy subprocesses are hard-terminated on
Windows (``TerminateProcess``), so ``atexit`` cannot be relied on; the flush
interval bounds how much tail a hard kill can lose.
"""
from __future__ import annotations

import os
from typing import Dict, List, Tuple

from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.logging.pose_cadence_flusher import PoseCadenceFlusher
from navpy.core.kernel.logging.pose_cadence_streams import HEADERS as _HEADERS
from navpy.core.kernel.logging.pose_cadence_naming import (
    file_owner,
    resolve_naming,
    starts_fresh,
    validate_producer,
    write_rows,
)


def _resolve_output_dir() -> str | None:
    raw = os.environ.get("NAVPY_POSE_CADENCE_DEBUG")
    if raw is None:
        return None
    value = raw.strip()
    if value == "" or value.lower() in {"0", "off", "false", "no"}:
        return None
    if value.lower() in {"1", "on", "true", "yes"}:
        return os.path.join(
            os.environ.get("TEMP", os.getcwd()), "navpy_pose_cadence"
        )
    return value


_OUTPUT_DIR = _resolve_output_dir()
ENABLED = _OUTPUT_DIR is not None

_NAMING = resolve_naming()
_producer = "navpy"


def set_producer(name: str) -> None:
    """Name this process's files in ``producer`` naming (the GCS backend: ``gcs``)."""
    global _producer
    _producer = validate_producer(name)
# Bounds the rows a Windows hard-terminate (no atexit) can lose to the last
# quarter wall-second -- at SIM_SPEEDUP=10 that is ~2.5 sim-seconds of tail,
# which keeps the end-of-navigation task (SNAP) window in the record.
_FLUSH_INTERVAL_S = 0.25

# key -> list of row tuples. Distinct keys are written by distinct threads
# (reader thread per sys_id for arrivals; detector thread per sys_id for poses),
# so per-key appends never contend. The lock guards only buffer creation.
_buffers: Dict[Tuple[int, str], List[tuple]] = {}
_buffers_lock = rt.Lock()
# Rows written to disk per key so far. A key's presence means its file was
# started; the written rows themselves are deleted from the buffer.
_written: Dict[Tuple[int, str], int] = {}
# Serializes dump_all so the periodic flusher and the atexit hook can't interleave
# their appends / _written updates for the same file.
_dump_lock = rt.Lock()


def _buffer_for(sys_id: int, stream: str) -> List[tuple]:
    key = (sys_id, stream)
    buf = _buffers.get(key)
    if buf is None:
        with _buffers_lock:
            buf = _buffers.get(key)
            if buf is None:
                buf = []
                _buffers[key] = buf
    return buf


def gc_rows(sys_id: int) -> List[tuple]:
    """The ``gc`` stream's buffer, made now so a GC callback only appends to it."""
    return _buffer_for(int(sys_id), "gc")


def _cell(value: object) -> object:
    return value if value is not None else ""


def record_attitude_arrival(sys_id: int, wall_s: float, boot_ms: int) -> None:
    """Record one raw ATTITUDE receipt (reader-thread hot path)."""
    if not ENABLED:
        return
    _buffer_for(int(sys_id), "attitude_arrival").append((wall_s, boot_ms))


def record_position_arrival(sys_id: int, wall_s: float, boot_ms: int) -> None:
    """Record one raw GLOBAL_POSITION_INT receipt (reader-thread hot path)."""
    if not ENABLED:
        return
    _buffer_for(int(sys_id), "position_arrival").append((wall_s, boot_ms))


def record_telemetry_reject(
        sys_id: int,
        wall_s: float,
        mtype: str,
        reason: str,
) -> None:
    """Record one autopilot-telemetry packet the pose cache refused."""
    if not ENABLED:
        return
    _buffer_for(int(sys_id), "telemetry_reject").append((wall_s, mtype, reason))


def record_detector_pose(
        sys_id: int,
        wall_s: float,
        boot_s: float | None,
        outcome: str,
        frame_ts: float | None = None,
        epoch: int | None = None,
) -> None:
    """Record one detector timestamp attempt (detector-thread hot path)."""
    if not ENABLED:
        return
    _buffer_for(int(sys_id), "detector_pose").append(
        (wall_s, _cell(boot_s), outcome, _cell(frame_ts), _cell(epoch))
    )


def record_detector_poll(sys_id: int, row: tuple) -> None:
    """Record one detector poll iteration (``detector_poll_timer`` row)."""
    if not ENABLED:
        return
    _buffer_for(int(sys_id), "detector_poll").append(tuple(_cell(c) for c in row))


def record_observation(
        sys_id: int,
        wall_s: float,
        obs_ts: float | None,
        src_now_s: float | None,
        outcome: str,
        epoch: int | None = None,
) -> None:
    """Record one detection delivered to guidance (nav())."""
    if not ENABLED:
        return
    _buffer_for(int(sys_id), "observation").append(
        (wall_s, _cell(obs_ts), _cell(src_now_s), outcome, _cell(epoch))
    )


def record_worker(
        sys_id: int,
        wall_start_s: float,
        exec_ms: float | None,
        obs_ts: float | None,
        src_now_s: float | None = None,
        source: str = "",
        outcome: str = "fresh",
        work_cpu_ms: float | None = None,
        epoch: int | None = None,
) -> None:
    """Record one guidance command actually sent (execute_guidance_command)."""
    if not ENABLED:
        return
    _buffer_for(int(sys_id), "worker").append(
        (
            wall_start_s,
            _cell(exec_ms),
            _cell(obs_ts),
            _cell(src_now_s),
            source,
            outcome,
            _cell(work_cpu_ms),
            _cell(epoch),
        )
    )


def record_frame_lineage(
        sys_id: int,
        wall_s: float,
        stage: str,
        epoch: int,
        frame_ts: float,
        disposition: str,
) -> None:
    """Record what became of one fresh guidance frame."""
    if not ENABLED:
        return
    _buffer_for(int(sys_id), "frame_lineage").append(
        (wall_s, stage, epoch, frame_ts, disposition)
    )




def dump_all(*, strict: bool = False) -> None:
    """Append newly-buffered rows to ``<dir>/pose_cadence_<stream>_uav_<sys>_pid_<pid>.csv``.

    Incremental: each flush writes the rows buffered so far and then deletes
    them from the buffer, so a flush is O(new) and written rows leave memory.
    The first flush of a key truncates + writes the header; later flushes
    append. Serialized by ``_dump_lock`` so the daemon flusher and atexit can't
    race the same file. PID-keyed filenames by default: an eval that restarts
    NavPy with the same sys_ids keeps each generation apart; ``producer``
    naming (``pose_cadence_naming``) gives stable names instead.
    """
    if not ENABLED:
        return
    with _dump_lock:
        with _buffers_lock:
            keys = list(_buffers.keys())
        if not keys:
            return
        os.makedirs(_OUTPUT_DIR, exist_ok=True)
        owner = file_owner(_NAMING, _producer)
        for key in keys:
            rows = _buffers[key]
            # Recorders only ever append, without a lock: rows appended after
            # this len() read stay at the end for the next flush.
            end = len(rows)
            if end == 0:
                continue
            tail = rows[:end]
            sys_id, stream = key
            path = os.path.join(
                _OUTPUT_DIR, f"pose_cadence_{stream}_uav_{sys_id}_{owner}.csv"
            )
            try:
                fresh = starts_fresh(_NAMING, path, key not in _written)
                write_rows(
                    path,
                    _HEADERS.get(stream, "") if fresh else None,
                    (",".join(str(c) for c in row) + "\n" for row in tail),
                )
            except OSError:
                # The periodic flush retries later; the final (strict) one
                # must not report a complete flush that lost rows.
                if strict:
                    raise
                continue
            # Delete the written prefix in place. Clearing or rebinding the
            # list would lose rows a recorder appended after the len() read.
            del rows[:end]
            _written[key] = _written.get(key, 0) + end


# One periodic-flush daemon for this module generation, wired to this module's
# own dump. Constructed idle; started only under ENABLED (below). A reload
# rebinds this to a fresh instance, so a test that reloads under ENABLED must
# stop_flusher() the current generation first or the old daemon is orphaned.
_flusher = PoseCadenceFlusher(dump_all, _FLUSH_INTERVAL_S)


def _start_flusher() -> None:  # pragma: no cover - diagnostic wiring
    """Start the periodic flush daemon (import-time wiring under ENABLED)."""
    _flusher.start()


def stop_flusher() -> None:
    """Stop and join the periodic flusher (test lifecycle seam).

    Production never calls this: the daemon runs until process exit and the
    atexit hook drains the tail. Tests that reload this module under ENABLED
    call it to join each generation before the next reload orphans its handle.
    Delegates to :meth:`PoseCadenceFlusher.stop`: idempotent, does NOT flush,
    and re-raises a join timeout with the handle retained.
    """
    _flusher.stop()


if ENABLED:  # pragma: no cover - diagnostic wiring
    _start_flusher()
