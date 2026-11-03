"""This process's identity for an owner that must kill it later: its pid and, on Windows, its creation time.

A pid alone can be reused once the process exits; the pid with its creation time names one process. The creation
time is the FILETIME GetProcessTimes gives (100 ns since 1601, UTC), the value PowerShell's
``(Get-Process -Id N).StartTime.ToFileTimeUtc()`` reads back (approach_handles.windows_process).
"""
from __future__ import annotations

import ctypes
import os


def created_filetime() -> int:
    """This process's creation time as a Windows FILETIME; 0 off Windows (no owner reads it there)."""
    if os.name != "nt":
        return 0
    from ctypes import wintypes

    kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    kernel32.GetCurrentProcess.restype = wintypes.HANDLE
    kernel32.GetProcessTimes.argtypes = [wintypes.HANDLE, *[ctypes.POINTER(wintypes.FILETIME)] * 4]
    kernel32.GetProcessTimes.restype = wintypes.BOOL
    times = [wintypes.FILETIME() for _ in range(4)]
    if not kernel32.GetProcessTimes(kernel32.GetCurrentProcess(), *[ctypes.byref(t) for t in times]):
        raise ctypes.WinError(ctypes.get_last_error())
    return times[0].dwHighDateTime << 32 | times[0].dwLowDateTime


def own_identity() -> str:
    """"pid N, started T" for a ready line (T = created_filetime())."""
    return f"pid {os.getpid()}, started {created_filetime()}"


__all__ = ["created_filetime", "own_identity"]
