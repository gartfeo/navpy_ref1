"""Opt-in source-time observability for the pose->frame->observation->command
pipeline.

Started as a cadence instrument for the pose (ATTITUDE) stream; Phase 2a of the
10x-determinism plan extended it to every hop of the final approach data
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

Event streams, keyed by ``(sys_id, stream)``:

- ``attitude_arrival`` -- one row per ATTITUDE that is ACCEPTED and updates the
  pose cache (``vehicle_mav._process_message``, after
  ``_accept_telemetry_sample``). ``boot_ms`` is the autopilot sample time
  (sim-time under SIM_SPEEDUP); the ``boot_ms`` delta approximates ArduPilot's
  accepted emission cadence in SIM time, the ``wall_s`` delta the
  transport/reader cadence.
- ``position_arrival`` -- same, for GLOBAL_POSITION_INT (the position half of
  the pose the sim detector reads without a timestamp).
- ``telemetry_reject`` -- one row per autopilot-telemetry packet that
  ``_accept_telemetry_sample`` refused (foreign component / stale boot time),
  making the drop path countable instead of silent.
- ``detector_pose`` -- source-pose admission and final-approach detector outcomes:
  raw attitude boot time, accepted poses, emitted frames, explicit queue /
  reset / invalidation drops, missing timestamps, duplicates, and reordering.
  Only an actual frame publication is ``emitted``; its unchanged raw source
  timestamp appears in ``frame_ts``.
- ``observation`` -- one row per detection delivered to final approach
  (``runtime.nav``): the frame's source timestamp, the source clock's "now"
  read through the detection's own provider, and the delivery ``outcome``
  (fresh / duplicate / gap_mismatch / invalid).
- ``worker`` -- one row per final-approach command actually SENT
  (``execute_final_approach_command``, every command -- not decimated like the
  FINAL_APPROACH_CMD log event): raw observation source timestamp, matching source-
  clock now, diagnostic execution time, and the compatibility marker
  ``measured``, and whether the 25 Hz tick used a ``fresh`` command or a
  zero-order ``held`` primitive command. The current final-approach runtime does not
  issue predictions.

Rows buffer in memory (append-only lists; list.append is atomic in CPython). A
daemon thread flushes every ``_FLUSH_INTERVAL_S``, APPENDING only the rows added
since the last flush (O(new) per flush, so a long run never pays a growing
whole-file rewrite that could perturb the cadence it measures). A daemon flush --
not ``atexit`` -- is used because NavPy subprocesses are hard-terminated on
Windows (``TerminateProcess``), so ``atexit`` cannot be relied on; the flush
interval bounds how much tail a hard kill can lose.
"""
from __future__ import annotations

import os
import threading
from typing import Dict, List, Tuple

from navpy.modules.vehicle.pose_cadence_flusher import PoseCadenceFlusher


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
# Bounds the rows a Windows hard-terminate (no atexit) can lose to the last
# quarter wall-second -- at SIM_SPEEDUP=10 that is ~2.5 sim-seconds of tail,
# which keeps the end-of-navigation task (SNAP) window in the record.
_FLUSH_INTERVAL_S = 0.25

# key -> list of row tuples. Distinct keys are written by distinct threads
# (reader thread per sys_id for arrivals; detector thread per sys_id for poses),
# so per-key appends never contend. The lock guards only buffer creation.
_buffers: Dict[Tuple[int, str], List[tuple]] = {}
_buffers_lock = threading.Lock()
# Rows already written to disk per key, so each flush appends only new rows.
_written: Dict[Tuple[int, str], int] = {}
# Serializes dump_all so the periodic flusher and the atexit hook can't interleave
# their appends / _written updates for the same file.
_dump_lock = threading.Lock()


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
) -> None:
    """Record one detector timestamp attempt (detector-thread hot path)."""
    if not ENABLED:
        return
    _buffer_for(int(sys_id), "detector_pose").append(
        (wall_s, _cell(boot_s), outcome, _cell(frame_ts))
    )


def record_observation(
        sys_id: int,
        wall_s: float,
        obs_ts: float | None,
        src_now_s: float | None,
        outcome: str,
) -> None:
    """Record one detection delivered to final approach (nav())."""
    if not ENABLED:
        return
    _buffer_for(int(sys_id), "observation").append(
        (wall_s, _cell(obs_ts), _cell(src_now_s), outcome)
    )


def record_worker(
        sys_id: int,
        wall_start_s: float,
        exec_ms: float | None,
        obs_ts: float | None,
        src_now_s: float | None = None,
        source: str = "",
        outcome: str = "fresh",
) -> None:
    """Record one final-approach command actually sent (execute_final_approach_command)."""
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
        )
    )


_HEADERS = {
    "attitude_arrival": "wall_s,boot_ms\n",
    "position_arrival": "wall_s,boot_ms\n",
    "telemetry_reject": "wall_s,mtype,reason\n",
    "detector_pose": "wall_s,boot_s,outcome,frame_ts\n",
    "observation": "wall_s,obs_ts,src_now_s,outcome\n",
    "worker": "wall_start_s,exec_ms,obs_ts,src_now_s,source,outcome\n",
}


def dump_all() -> None:
    """Append newly-buffered rows to ``<dir>/pose_cadence_<stream>_uav_<sys>_pid_<pid>.csv``.

    Incremental: ``_written`` tracks how many rows of each buffer are already on
    disk, so a flush writes only the tail (O(new)) instead of rewriting the whole
    history. The first flush of a key truncates + writes the header; later flushes
    append. Serialized by ``_dump_lock`` so the daemon flusher and atexit can't
    race the same file. PID-keyed filenames: the GCS eval relaunches NavPy with
    the same sys_ids (param-write then mission), so a bare-sysid name would let
    generations overwrite each other.
    """
    if not ENABLED:
        return
    with _dump_lock:
        with _buffers_lock:
            keys = list(_buffers.keys())
        if not keys:
            return
        os.makedirs(_OUTPUT_DIR, exist_ok=True)
        pid = os.getpid()
        for key in keys:
            rows = _buffers[key]
            start = _written.get(key, 0)
            # Snapshot ONLY the unwritten tail. Copying whole buffers every
            # flush is O(total history) and its growing cost would perturb the
            # cadence being measured. Recorders only ever append, so reading
            # len() and slicing up to it without their lock is safe: rows
            # appended after the len() read are simply left for the next
            # flush.
            end = len(rows)
            if start >= end:
                continue
            tail = rows[start:end]
            sys_id, stream = key
            path = os.path.join(
                _OUTPUT_DIR, f"pose_cadence_{stream}_uav_{sys_id}_pid_{pid}.csv"
            )
            try:
                # 'w' on the first write (truncates any stale file + header),
                # 'a' thereafter so earlier rows are preserved.
                with open(path, "w" if start == 0 else "a",
                          encoding="utf-8", newline="") as fh:
                    if start == 0:
                        fh.write(_HEADERS.get(stream, ""))
                    for row in tail:
                        fh.write(",".join(str(c) for c in row) + "\n")
                _written[key] = end
            except OSError:
                pass


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
