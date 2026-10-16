"""Hold a 1 ms Windows timer resolution that Windows 11 cannot silently drop.

Event/lock waits on Windows are bound to the process timer resolution. Since
Windows 11 the system may stop honouring timeBeginPeriod for a process it
treats as invisible (power throttling), so waits fall back to the 15.6 ms tick
mid-run.  While the request is held the process also opts out of that
throttling (SetProcessInformation, ProcessPowerThrottling).
"""

from __future__ import annotations

import ctypes
import logging
import os
from typing import Optional

from navpy.core.kernel import runtime_time as rt

_LOG = logging.getLogger(__name__)

_TIMER_PERIOD_MS = 1
_timer_resolution_lock = rt.RLock()
_timer_resolution_refcount = 0
_PROCESS_POWER_THROTTLING = 4  # PROCESS_INFORMATION_CLASS ProcessPowerThrottling
_POWER_THROTTLING_VERSION = 1  # PROCESS_POWER_THROTTLING_CURRENT_VERSION
_IGNORE_TIMER_RESOLUTION = 0x4  # PROCESS_POWER_THROTTLING_IGNORE_TIMER_RESOLUTION
# The (ControlMask, StateMask) policy to restore when the last request ends.
_saved_power_throttling: Optional[tuple[int, int]] = None
_native_timer_cache: Optional["_WindowsTimerNative"] = None
_native_timer_unavailable = False  # load failed once; do not retry or re-warn


class _PowerThrottlingState(ctypes.Structure):
    _fields_ = [
        ("Version", ctypes.c_uint32),
        ("ControlMask", ctypes.c_uint32),
        ("StateMask", ctypes.c_uint32),
    ]


class _WindowsTimerNative:
    """The winmm timer and kernel32 power-throttling calls, with checked results."""

    def __init__(self) -> None:
        from ctypes import wintypes

        self._winmm = ctypes.WinDLL("winmm")  # type: ignore[attr-defined]
        self._kernel32 = ctypes.WinDLL(  # type: ignore[attr-defined]
            "kernel32", use_last_error=True,
        )
        for name in ("timeBeginPeriod", "timeEndPeriod"):
            function = getattr(self._winmm, name)
            function.argtypes = [wintypes.UINT]
            function.restype = wintypes.UINT  # MMRESULT
        self._kernel32.GetCurrentProcess.argtypes = []
        self._kernel32.GetCurrentProcess.restype = wintypes.HANDLE
        for name in ("GetProcessInformation", "SetProcessInformation"):
            function = getattr(self._kernel32, name)
            function.argtypes = [
                wintypes.HANDLE, ctypes.c_int, ctypes.c_void_p, wintypes.DWORD,
            ]
            function.restype = wintypes.BOOL

    def time_begin_period(self, period_ms: int) -> int:
        return int(self._winmm.timeBeginPeriod(period_ms))

    def time_end_period(self, period_ms: int) -> int:
        return int(self._winmm.timeEndPeriod(period_ms))

    def get_power_throttling(self) -> tuple[int, int]:
        state = _PowerThrottlingState(_POWER_THROTTLING_VERSION, 0, 0)
        self._process_information("GetProcessInformation", state)
        return int(state.ControlMask), int(state.StateMask)

    def set_power_throttling(self, control_mask: int, state_mask: int) -> None:
        state = _PowerThrottlingState(
            _POWER_THROTTLING_VERSION, control_mask, state_mask,
        )
        self._process_information("SetProcessInformation", state)

    def _process_information(self, name: str, state: _PowerThrottlingState) -> None:
        ok = getattr(self._kernel32, name)(
            self._kernel32.GetCurrentProcess(),
            _PROCESS_POWER_THROTTLING,
            ctypes.byref(state),
            ctypes.sizeof(state),
        )
        if not ok:
            error = ctypes.get_last_error()  # type: ignore[attr-defined]
            raise OSError(error, f"{name}(ProcessPowerThrottling) failed: {error}")


def _native_timer() -> Optional[_WindowsTimerNative]:
    global _native_timer_cache, _native_timer_unavailable
    if os.name != "nt" or _native_timer_unavailable:
        return None
    if _native_timer_cache is None:
        try:
            _native_timer_cache = _WindowsTimerNative()
        except Exception as error:
            _native_timer_unavailable = True
            _LOG.warning("Windows timer API unavailable: %s", error)
            return None
    return _native_timer_cache


def request_high_resolution_timer() -> bool:
    """Request 1 ms Windows sleeps while accelerated SITL is active."""
    global _timer_resolution_refcount, _saved_power_throttling
    with _timer_resolution_lock:
        if _timer_resolution_refcount == 0:
            native = _native_timer()
            if native is None:
                return False
            saved = _honour_timer_resolution(native)
            try:
                result: object = native.time_begin_period(_TIMER_PERIOD_MS)
            except Exception as error:
                result = error
            if result != 0:
                _LOG.warning("timeBeginPeriod(%d) failed: %s", _TIMER_PERIOD_MS, result)
                _restore_power_throttling(native, saved)
                return False
            _saved_power_throttling = saved
        _timer_resolution_refcount += 1
        return True


def release_high_resolution_timer() -> None:
    """Drop one 1 ms request; the last one restores the process state."""
    global _timer_resolution_refcount, _saved_power_throttling
    with _timer_resolution_lock:
        if _timer_resolution_refcount <= 0:
            return
        _timer_resolution_refcount -= 1
        if _timer_resolution_refcount > 0:
            return
        saved, _saved_power_throttling = _saved_power_throttling, None
        native = _native_timer()
        if native is None:
            return
        try:
            result = native.time_end_period(_TIMER_PERIOD_MS)
            if result != 0:
                _LOG.warning("timeEndPeriod(%d) failed: %s", _TIMER_PERIOD_MS, result)
        except Exception as error:
            _LOG.warning("timeEndPeriod(%d) failed: %s", _TIMER_PERIOD_MS, error)
        finally:
            _restore_power_throttling(native, saved)


def _honour_timer_resolution(
    native: _WindowsTimerNative,
) -> Optional[tuple[int, int]]:
    """Opt out of timer-resolution throttling; return the policy to restore."""
    try:
        control, state = native.get_power_throttling()
        native.set_power_throttling(
            control | _IGNORE_TIMER_RESOLUTION,
            state & ~_IGNORE_TIMER_RESOLUTION,
        )
    except OSError as error:
        _LOG.warning("Timer resolution may be throttled by Windows: %s", error)
        return None
    _LOG.info(
        "Timer resolution throttling opt-out held (previous policy "
        "control=0x%x state=0x%x)", control, state,
    )
    return control, state


def _restore_power_throttling(
    native: _WindowsTimerNative,
    saved: Optional[tuple[int, int]],
) -> None:
    if saved is None:
        return
    try:
        native.set_power_throttling(*saved)
    except OSError as error:
        _LOG.warning("Restoring the power throttling policy failed: %s", error)


__all__ = ["release_high_resolution_timer", "request_high_resolution_timer"]
