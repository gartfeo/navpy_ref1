"""Schedule the NavPy process as time-critical on Windows (todo 65).

The companion's threads wait on sockets and timers and must run within a few
milliseconds of waking. On a loaded Windows host two scheduler policies broke
that (measured in `.logs/t65`, 20 busy workers on a 20-core hybrid CPU):

- Priority: when every core runs normal-priority work, a woken
  normal-priority thread can wait one quantum (about 31 ms) for a core; at
  the 10x SITL speedup that is about 300 ms of aircraft time with no new pose.
  Above-normal priority lets it pre-empt normal work at once.
- EcoQoS: Windows treats a windowless process as background work and, on a
  hybrid CPU, runs it on the efficiency cores when the fast ones are busy, so
  each frame took twice as long. Opting out of execution-speed throttling
  keeps it at high QoS.

NavPy uses about one core, so neither can starve the desktop. The process
sets both itself: a venv ``python.exe`` is a launcher, and the interpreter it
starts does not inherit an above-normal class. Other platforms are left
unchanged (raising priority on Linux needs privileges, and the aircraft
companion runs alone).
"""

from __future__ import annotations

import ctypes
import logging
import sys

_LOG = logging.getLogger(__name__)

ABOVE_NORMAL_PRIORITY_CLASS = 0x00008000
PROCESS_POWER_THROTTLING = 4  # PROCESS_INFORMATION_CLASS ProcessPowerThrottling
POWER_THROTTLING_VERSION = 1  # PROCESS_POWER_THROTTLING_CURRENT_VERSION
EXECUTION_SPEED = 0x1  # PROCESS_POWER_THROTTLING_EXECUTION_SPEED


class PowerThrottlingState(ctypes.Structure):
    _fields_ = [
        ("Version", ctypes.c_uint32),
        ("ControlMask", ctypes.c_uint32),
        ("StateMask", ctypes.c_uint32),
    ]


def _kernel32() -> object:
    from ctypes import wintypes

    kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)  # type: ignore[attr-defined]
    kernel32.GetCurrentProcess.argtypes = []
    kernel32.GetCurrentProcess.restype = wintypes.HANDLE
    kernel32.SetPriorityClass.argtypes = [wintypes.HANDLE, wintypes.DWORD]
    kernel32.SetPriorityClass.restype = wintypes.BOOL
    kernel32.SetProcessInformation.argtypes = [
        wintypes.HANDLE, ctypes.c_int, ctypes.c_void_p, wintypes.DWORD,
    ]
    kernel32.SetProcessInformation.restype = wintypes.BOOL
    return kernel32


def raise_process_qos(platform: str = sys.platform, kernel32: object | None = None) -> bool:
    """Set above-normal priority and high QoS; True when both were set.

    A failure is logged and returns False: NavPy still runs, only without the
    protection against a loaded host.
    """
    if platform != "win32":
        return False
    try:
        native = _kernel32() if kernel32 is None else kernel32
        process = native.GetCurrentProcess()
        priority = bool(native.SetPriorityClass(process, ABOVE_NORMAL_PRIORITY_CLASS))
        if not priority:
            _LOG.warning("SetPriorityClass(ABOVE_NORMAL) failed: error %s", ctypes.get_last_error())
        # Control the execution-speed bit and clear it: never throttle.
        state = PowerThrottlingState(POWER_THROTTLING_VERSION, EXECUTION_SPEED, 0)
        qos = bool(native.SetProcessInformation(
            process, PROCESS_POWER_THROTTLING, ctypes.byref(state), ctypes.sizeof(state),
        ))
        if not qos:
            _LOG.warning("EcoQoS opt-out failed: error %s", ctypes.get_last_error())
        return priority and qos
    except (AttributeError, OSError) as error:
        _LOG.warning("Windows scheduling API unavailable: %s", error)
    return False


__all__ = ["ABOVE_NORMAL_PRIORITY_CLASS", "EXECUTION_SPEED", "raise_process_qos"]
