"""TEMPORARY diagnostic (nav-reader-lag session): bounded reader/GIL trace.

Not for merge. A ring buffer of timestamped records (perf_counter) from the
MAVLink reader, router, callbacks, detector poll, command worker, GC and the
freshness check. NavPy's main enables it; a NAV exit (command cadence off)
dumps the ring 2 s later to ``reader_trace_pid<PID>_<n>.csv`` in the
pose-cadence output directory. At most three dumps per process.
"""
from __future__ import annotations

import collections
import gc
import os
import threading
import time

from navpy.core.kernel.logging import pose_cadence_debug as _pcd

ENABLED = False
_pc = time.perf_counter
_get_ident = threading.get_ident
_RING: collections.deque = collections.deque(maxlen=600_000)
_append = _RING.append
_dumps = [0]
_gc_start: dict[int, float] = {}
_lock = threading.Lock()


def enable() -> None:
    global ENABLED
    if _pcd._OUTPUT_DIR is None:
        return
    ENABLED = True
    gc.callbacks.append(_on_gc)


def rec(kind: str, t: float, a=None, b=None, c=None, d=None, e=None) -> None:
    _append((t, _get_ident(), kind, a, b, c, d, e))


def _on_gc(phase: str, info: dict) -> None:
    ident = _get_ident()
    if phase == "start":
        _gc_start[ident] = _pc()
        return
    started = _gc_start.pop(ident, None)
    if started is not None:
        _append((started, ident, "gc", _pc() - started, info.get("generation"),
                 info.get("collected"), None, None))


_nav = {"on": False, "on_since": None, "pending": False}


def nav(active: bool) -> None:
    if not ENABLED:
        return
    now = _pc()
    rec("nav", now, 1 if active else 0)
    with _lock:
        _nav["on"] = active
        if active:
            _nav["on_since"] = now
            _nav["pending"] = True
            return
    timer = threading.Timer(2.0, _dump)
    timer.daemon = True
    timer.start()


def _dump() -> None:
    with _lock:
        if _nav["on"] or not _nav["pending"] or _dumps[0] >= 3:
            return
        _nav["pending"] = False
        _dumps[0] += 1
        n = _dumps[0]
    rows = list(_RING)
    names = {t.ident: (t.name, getattr(t, "native_id", None)) for t in threading.enumerate()}
    path = os.path.join(_pcd._OUTPUT_DIR, f"reader_trace_pid{os.getpid()}_{n}.csv")
    epoch_s, perf_s = time.time(), _pc()
    with open(path, "w", newline="", encoding="utf-8") as handle:
        handle.write(f"#epoch_s={epoch_s:.6f},perf_s={perf_s:.6f}\n")
        for ident, (name, native) in names.items():
            handle.write(f"#thread,{ident},{native},{name}\n")
        handle.write("t,ident,kind,a,b,c,d,e\n")
        for row in rows:
            handle.write(",".join("" if v is None else (f"{v:.7f}" if isinstance(v, float) else str(v)) for v in row))
            handle.write("\n")
