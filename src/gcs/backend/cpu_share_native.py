"""Windows scheduling calls behind the CPU share (``bounded_cpu_share.md``).

The process's own priority class and affinity, and the CPU set efficiency
classes that tell performance cores from efficiency cores. Off win32 every
call is a no-op (``NoScheduling``).
"""

from __future__ import annotations

import ctypes
import sys

ABOVE_NORMAL_PRIORITY_CLASS = 0x00008000
BELOW_NORMAL_PRIORITY_CLASS = 0x00004000
_CPU_SET_INFORMATION = 0  # CPU_SET_INFORMATION_TYPE CpuSetInformation
_ERROR_INSUFFICIENT_BUFFER = 122


class _CpuSet(ctypes.Structure):
    _fields_ = [
        ("Id", ctypes.c_uint32),
        ("Group", ctypes.c_uint16),
        ("LogicalProcessorIndex", ctypes.c_uint8),
        ("CoreIndex", ctypes.c_uint8),
        ("LastLevelCacheIndex", ctypes.c_uint8),
        ("NumaNodeIndex", ctypes.c_uint8),
        ("EfficiencyClass", ctypes.c_uint8),
        ("AllFlags", ctypes.c_uint8),
        ("Reserved", ctypes.c_uint32),
        ("AllocationTag", ctypes.c_uint64),
    ]


class _CpuSetInformation(ctypes.Structure):
    _fields_ = [("Size", ctypes.c_uint32), ("Type", ctypes.c_uint32), ("CpuSet", _CpuSet)]


def mask_of(cpus: object) -> int:
    """The affinity mask of logical CPUs (processor group 0)."""
    mask = 0
    for cpu in cpus:  # type: ignore[attr-defined]
        mask |= 1 << int(cpu)
    return mask


def cpus_of(mask: int) -> list[int]:
    return [cpu for cpu in range(mask.bit_length()) if mask >> cpu & 1]


class WindowsScheduling:
    """The real calls, on this process."""

    def __init__(self) -> None:
        from ctypes import wintypes

        k = ctypes.WinDLL("kernel32", use_last_error=True)  # type: ignore[attr-defined]
        k.GetCurrentProcess.argtypes = []
        k.GetCurrentProcess.restype = wintypes.HANDLE
        k.GetPriorityClass.argtypes = [wintypes.HANDLE]
        k.GetPriorityClass.restype = wintypes.DWORD
        k.SetPriorityClass.argtypes = [wintypes.HANDLE, wintypes.DWORD]
        k.SetPriorityClass.restype = wintypes.BOOL
        k.GetProcessAffinityMask.argtypes = [
            wintypes.HANDLE, ctypes.POINTER(ctypes.c_size_t), ctypes.POINTER(ctypes.c_size_t),
        ]
        k.GetProcessAffinityMask.restype = wintypes.BOOL
        k.SetProcessAffinityMask.argtypes = [wintypes.HANDLE, ctypes.c_size_t]
        k.SetProcessAffinityMask.restype = wintypes.BOOL
        k.GetSystemCpuSetInformation.argtypes = [
            ctypes.c_void_p, wintypes.ULONG, ctypes.POINTER(wintypes.ULONG),
            wintypes.HANDLE, wintypes.ULONG,
        ]
        k.GetSystemCpuSetInformation.restype = wintypes.BOOL
        self._k = k
        self._me = k.GetCurrentProcess()

    def efficiency_classes(self) -> dict[int, int]:
        """Logical CPU (group 0) -> efficiency class; higher is faster."""
        from ctypes import wintypes

        length = wintypes.ULONG(0)
        self._k.GetSystemCpuSetInformation(None, 0, ctypes.byref(length), self._me, 0)
        if ctypes.get_last_error() != _ERROR_INSUFFICIENT_BUFFER or not length.value:  # type: ignore[attr-defined]
            raise ctypes.WinError(ctypes.get_last_error())  # type: ignore[attr-defined]
        buffer = ctypes.create_string_buffer(length.value)
        if not self._k.GetSystemCpuSetInformation(buffer, length, ctypes.byref(length), self._me, 0):
            raise ctypes.WinError(ctypes.get_last_error())  # type: ignore[attr-defined]
        classes: dict[int, int] = {}
        offset = 0
        while offset < length.value:
            entry = _CpuSetInformation.from_buffer(buffer, offset)
            if entry.Type == _CPU_SET_INFORMATION and entry.CpuSet.Group == 0:
                classes[entry.CpuSet.LogicalProcessorIndex] = entry.CpuSet.EfficiencyClass
            offset += entry.Size or ctypes.sizeof(_CpuSetInformation)
        return classes

    def system_mask(self) -> int:
        """Every CPU the system offers this process."""
        process, system = ctypes.c_size_t(), ctypes.c_size_t()
        if not self._k.GetProcessAffinityMask(self._me, ctypes.byref(process), ctypes.byref(system)):
            raise ctypes.WinError(ctypes.get_last_error())  # type: ignore[attr-defined]
        return system.value

    def own(self) -> tuple[int, int]:
        """This process's priority class and affinity mask."""
        process, system = ctypes.c_size_t(), ctypes.c_size_t()
        if not self._k.GetProcessAffinityMask(self._me, ctypes.byref(process), ctypes.byref(system)):
            raise ctypes.WinError(ctypes.get_last_error())  # type: ignore[attr-defined]
        return self._k.GetPriorityClass(self._me), process.value

    def set_own(self, priority: int, mask: int) -> None:
        """Set this process's priority class and affinity; OSError on failure."""
        if not self._k.SetPriorityClass(self._me, priority):
            raise ctypes.WinError(ctypes.get_last_error())  # type: ignore[attr-defined]
        if not self._k.SetProcessAffinityMask(self._me, mask):
            raise ctypes.WinError(ctypes.get_last_error())  # type: ignore[attr-defined]


class NoScheduling:
    """Off win32: no CPUs to pick, nothing to set."""

    def efficiency_classes(self) -> dict[int, int]:
        return {}

    def system_mask(self) -> int:
        return 0

    def own(self) -> tuple[int, int]:
        return 0, 0

    def set_own(self, priority: int, mask: int) -> None:
        return None


def native_scheduling() -> WindowsScheduling | NoScheduling:
    return WindowsScheduling() if sys.platform == "win32" else NoScheduling()


_WAIT_OBJECT_0 = 0x0
_WAIT_ABANDONED = 0x80


class NamedMutex:
    """A machine-session mutex: a holder that dies hands it over at once
    (``WAIT_ABANDONED``), so no stale lock is ever left behind.

    Off win32 it always acquires: there are no shares to guard there.
    """

    def __init__(self, name: str) -> None:
        self._handle = None
        if sys.platform != "win32":
            return
        from ctypes import wintypes

        k = ctypes.WinDLL("kernel32", use_last_error=True)  # type: ignore[attr-defined]
        k.CreateMutexW.argtypes = [ctypes.c_void_p, wintypes.BOOL, wintypes.LPCWSTR]
        k.CreateMutexW.restype = wintypes.HANDLE
        k.WaitForSingleObject.argtypes = [wintypes.HANDLE, wintypes.DWORD]
        k.WaitForSingleObject.restype = wintypes.DWORD
        k.ReleaseMutex.argtypes = [wintypes.HANDLE]
        k.ReleaseMutex.restype = wintypes.BOOL
        self._k = k
        self._handle = k.CreateMutexW(None, False, f"Local\\{name}")
        if not self._handle:
            raise ctypes.WinError(ctypes.get_last_error())  # type: ignore[attr-defined]

    def acquire(self, timeout_s: float) -> bool:
        """Take it within *timeout_s* (0: one attempt); False when busy."""
        if self._handle is None:
            return True
        result = self._k.WaitForSingleObject(self._handle, int(timeout_s * 1000))
        return result in (_WAIT_OBJECT_0, _WAIT_ABANDONED)

    def release(self) -> None:
        if self._handle is not None:
            self._k.ReleaseMutex(self._handle)


__all__ = [
    "ABOVE_NORMAL_PRIORITY_CLASS",
    "BELOW_NORMAL_PRIORITY_CLASS",
    "NamedMutex",
    "NoScheduling",
    "WindowsScheduling",
    "cpus_of",
    "mask_of",
    "native_scheduling",
]
