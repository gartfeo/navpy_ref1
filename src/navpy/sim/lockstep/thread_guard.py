"""The thread guard: no Python thread starts unless the runtime made it.

Installed at bootstrap step 2. ``threading.Thread.start``, the start alias
``threading`` cached at import (``threading._start_new_thread``) and the raw
``_thread`` starts are wrapped, so starting a thread the runtime did not create
is a :class:`LockstepFault` at creation, latched in the baton, even one that
would finish before quiescence. Only the OS thread
:meth:`~navpy.sim.lockstep.baton.BatonRuntime.spawn` is starting at that moment
passes (``BatonRuntime.starting``): identity, not class or name, so a
look-alike built outside the runtime is refused.
Product threads are ``rt.Thread`` and so run on spawned threads. The per-tick
:func:`~navpy.sim.lockstep.baton.audit` of ``threading.enumerate()`` stays as a
second check.

Boundary: native threads inside extension modules (OpenCV, BLAS, ...) are
invisible to both checks. :func:`apply_native_thread_defaults` sets the
lockstep default for the one the product uses, ``cv2.setNumThreads(0)``;
anything else such a thread does must not reach a byte the run records.
"""

from __future__ import annotations

import _thread
import importlib
import importlib.util
import threading
from collections.abc import Callable
from typing import Any

from navpy.sim.lockstep.baton import BatonRuntime

#: The ``_thread`` entry points that start a raw thread (``start_new`` is the old alias).
_RAW_STARTS = ("start_new_thread", "start_new")


def install_thread_guard(baton: BatonRuntime) -> Callable[[], None]:
    """Wrap the thread starts; returns the uninstall (for tests)."""
    original_start = threading.Thread.start
    original_alias = threading._start_new_thread  # type: ignore[attr-defined]
    originals = {name: getattr(_thread, name) for name in _RAW_STARTS if hasattr(_thread, name)}

    def authorized(thread: object) -> bool:
        return thread is not None and thread is baton.starting

    def guarded_start(thread: threading.Thread) -> None:
        if not authorized(thread):
            baton.fail(f"unmanaged thread started: {thread.name} (use rt.Thread)")
        original_start(thread)

    def guarded_alias(function: Callable[..., object], args: tuple, *rest: Any) -> int:
        # ``Thread.start`` passes the bound ``_bootstrap`` of the thread it starts.
        if not authorized(getattr(function, "__self__", None)):
            baton.fail("unmanaged thread started: threading._start_new_thread (use rt.Thread)")
        return original_alias(function, args, *rest)

    def refused_raw_start(*_args: Any, **_kwargs: Any) -> int:
        baton.fail("unmanaged thread started: _thread.start_new_thread (use rt.Thread)")

    threading.Thread.start = guarded_start
    threading._start_new_thread = guarded_alias  # type: ignore[attr-defined]
    for name in originals:
        setattr(_thread, name, refused_raw_start)

    def uninstall() -> None:
        threading.Thread.start = original_start
        threading._start_new_thread = original_alias  # type: ignore[attr-defined]
        for name, original in originals.items():
            setattr(_thread, name, original)

    return uninstall


def apply_native_thread_defaults() -> None:
    """OpenCV runs its own kernels single-threaded under lockstep (if installed)."""
    if importlib.util.find_spec("cv2") is not None:
        importlib.import_module("cv2").setNumThreads(0)


__all__ = ["apply_native_thread_defaults", "install_thread_guard"]
