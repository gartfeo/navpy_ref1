"""Operating-system probes used by the shared GCS instance registry."""

from __future__ import annotations

import os
import socket
import subprocess
from typing import Callable

from gcs.backend import instance_ports as ip


def port_bound(port: int) -> bool:
    """Return whether a TCP listener accepts connections on localhost."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.settimeout(0.3)
        return probe.connect_ex(("127.0.0.1", port)) == 0


def pid_alive(pid: object) -> bool:
    """Return whether *pid* names a currently running process."""
    if not pid:
        return False
    try:
        numeric_pid = int(pid)
    except (TypeError, ValueError):
        return False
    if numeric_pid <= 0:
        return False
    if os.name == "nt":
        return _windows_pid_alive(numeric_pid)
    try:
        os.kill(numeric_pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    except OSError:
        return False


def _windows_pid_alive(pid: int) -> bool:
    import ctypes

    process_query = 0x1000
    synchronize = 0x00100000
    wait_timeout = 0x102
    kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    handle = kernel32.OpenProcess(process_query | synchronize, False, pid)
    if not handle:
        return False
    try:
        return kernel32.WaitForSingleObject(handle, 0) == wait_timeout
    finally:
        kernel32.CloseHandle(handle)


def pid_start_time(pid: object) -> int | None:
    """Return a Windows process-creation stamp, when one is available."""
    if not pid or os.name != "nt":
        return None
    try:
        numeric_pid = int(pid)
    except (TypeError, ValueError):
        return None
    if numeric_pid <= 0:
        return None
    return _windows_pid_start_time(numeric_pid)


def _windows_pid_start_time(pid: int) -> int | None:
    import ctypes
    from ctypes import wintypes

    process_query = 0x1000
    kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    handle = kernel32.OpenProcess(process_query, False, pid)
    if not handle:
        return None
    try:
        creation = wintypes.FILETIME()
        exit_time = wintypes.FILETIME()
        kernel_time = wintypes.FILETIME()
        user_time = wintypes.FILETIME()
        if not kernel32.GetProcessTimes(
            handle,
            ctypes.byref(creation),
            ctypes.byref(exit_time),
            ctypes.byref(kernel_time),
            ctypes.byref(user_time),
        ):
            return None
        return (creation.dwHighDateTime << 32) | creation.dwLowDateTime
    finally:
        kernel32.CloseHandle(handle)


def pid_matches(pid: object, expected_start: object = None) -> bool:
    """Return whether a PID is alive and still has its recorded identity."""
    if not pid_alive(pid):
        return False
    if not expected_start:
        return True
    actual = pid_start_time(pid)
    return actual is None or actual == expected_start


def port_bindable(port: int, kind: str) -> bool:
    """Return whether localhost *port* can be bound for TCP or UDP."""
    socket_kind = socket.SOCK_STREAM if kind == "tcp" else socket.SOCK_DGRAM
    with socket.socket(socket.AF_INET, socket_kind) as probe:
        try:
            probe.bind(("127.0.0.1", port))
            return True
        except OSError:
            return False


def udp_port_held(port: int) -> bool:
    """Return whether any process holds a UDP port, including reuse binds."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        try:
            if os.name == "nt":
                probe.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
            probe.bind(("0.0.0.0", port))
            return False
        except OSError:
            return True


def slot_ports_free(chat: int) -> bool:
    """Return whether every port assigned to *chat* is available."""
    return (
        port_bindable(ip.frontend_port(chat), "tcp")
        and port_bindable(ip.backend_port(chat), "tcp")
        and port_bindable(ip.monitor_port(chat), "udp")
        and port_bindable(ip.mission_planner_port(chat), "udp")
        and not any(
            udp_port_held(port) for port in ip.companion_ports_for_chat(chat)
        )
    )


def listeners_on_port(port: int, kind: str = "tcp") -> list[int]:
    """Return Windows PIDs listening on or holding *port*."""
    if os.name != "nt":
        return []
    try:
        output = subprocess.run(
            ["netstat", "-ano"],
            capture_output=True,
            text=True,
            timeout=10,
        ).stdout
    except Exception:
        return []
    return _parse_netstat(output, port=port, kind=kind)


def _parse_netstat(output: str, *, port: int, kind: str) -> list[int]:
    pids: set[int] = set()
    target = str(port)
    for line in output.splitlines():
        parts = line.split()
        pid_field = _matching_pid_field(parts, target=target, kind=kind)
        if pid_field is None:
            continue
        try:
            pids.add(int(pid_field))
        except ValueError:
            pass
    return sorted(pids)


def _matching_pid_field(
    parts: list[str],
    *,
    target: str,
    kind: str,
) -> str | None:
    if not parts:
        return None
    protocol = parts[0].upper()
    if kind == "udp":
        if (
            not protocol.startswith("UDP")
            or len(parts) < 4
            or parts[1].rsplit(":", 1)[-1] != target
        ):
            return None
        return parts[3]
    if (
        not protocol.startswith("TCP")
        or len(parts) < 5
        or parts[3].upper() != "LISTENING"
        or parts[1].rsplit(":", 1)[-1] != target
    ):
        return None
    return parts[4]


def reap_port(
    port: int,
    kind: str = "tcp",
    exclude: set[int] | None = None,
    *,
    listener_lookup: Callable[[int, str], list[int]] = listeners_on_port,
) -> list[int]:
    """Terminate orphan process trees holding *port* and return their PIDs."""
    pids = [
        pid
        for pid in listener_lookup(port, kind)
        if not exclude or pid not in exclude
    ]
    for pid in pids:
        _terminate_pid(pid)
    return pids


def _terminate_pid(pid: int) -> None:
    if os.name == "nt":
        subprocess.run(
            ["taskkill", "/PID", str(pid), "/T", "/F"],
            capture_output=True,
        )
        return
    import signal

    try:
        os.kill(pid, signal.SIGTERM)
    except (ProcessLookupError, ValueError):
        pass


def reap_companion_ports(
    chat: int,
    exclude: set[int] | None = None,
    *,
    port_reaper: Callable[[int, str, set[int] | None], list[int]] | None = None,
) -> list[int]:
    """Terminate orphan holders of one chat's companion UDP ports."""
    reaped: list[int] = []
    reaper = port_reaper or _reap_port_adapter
    for port in ip.companion_ports_for_chat(chat):
        reaped += reaper(port, "udp", exclude)
    return reaped


def _reap_port_adapter(
    port: int,
    kind: str,
    exclude: set[int] | None,
) -> list[int]:
    return reap_port(port, kind=kind, exclude=exclude)
