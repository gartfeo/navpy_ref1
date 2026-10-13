"""The bridge process runs in Windows' HIGH priority class.

The board sends one frame per 5 ms loop and counts a loop stale when the bridge's reply arrives after
it has sent the next frame (HIL link contract v2; gate HIL_STALE_FRACTION_MAX in hitl_gate.py). With
Unreal rendering the camera on the same PC, the bridge at NORMAL priority lost the CPU often enough to
fail that gate on the board (cam1: 14.0% stale, cam2 9.3%; without Unreal 0.5%). HIGH (not REALTIME,
which would starve the USB stack and the OS) together with Unreal at BELOW_NORMAL measured 2.8%
(cam3, docs/design/hitl_unreal_camera.md). Elsewhere (WSL, Linux) the priority is left alone.
"""
from __future__ import annotations

import sys
from typing import Callable, Optional

HIGH_PRIORITY_CLASS = 0x00000080        # winbase.h


def _set_own_priority_class(priority_class: int) -> bool:
    import ctypes                                               # noqa: PLC0415 - Windows only
    kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    kernel32.GetCurrentProcess.restype = ctypes.c_void_p
    return bool(kernel32.SetPriorityClass(ctypes.c_void_p(kernel32.GetCurrentProcess()), priority_class))


def raise_bridge_priority(platform: str = sys.platform,
                          set_class: Optional[Callable[[int], bool]] = None) -> str:
    """Put this process in the HIGH priority class on Windows; returns a line for the bridge's log."""
    if platform != "win32":
        return "process priority unchanged (not Windows)"
    if (set_class or _set_own_priority_class)(HIGH_PRIORITY_CLASS):
        return "process priority HIGH"
    return "process priority unchanged (SetPriorityClass refused)"
