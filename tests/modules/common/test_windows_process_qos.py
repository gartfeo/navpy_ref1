"""NavPy schedules itself as time-critical on Windows (todo 65)."""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import pytest

from navpy.core.kernel import windows_process_qos
from navpy.core.kernel.windows_process_qos import (
    ABOVE_NORMAL_PRIORITY_CLASS,
    EXECUTION_SPEED,
    raise_process_qos,
)

_SRC = Path(__file__).resolve().parents[3] / "src"


class _Kernel:
    def __init__(self, priority: bool = True, qos: bool = True) -> None:
        self.results = {"priority": priority, "qos": qos}
        self.priority_calls: list[tuple[object, int]] = []
        self.qos_calls: list[tuple[object, int, int, int]] = []

    def GetCurrentProcess(self) -> object:  # noqa: N802 - Win32 name
        return "this-process"

    def SetPriorityClass(self, handle: object, priority: int) -> bool:  # noqa: N802
        self.priority_calls.append((handle, priority))
        return self.results["priority"]

    def SetProcessInformation(self, handle, information_class, state, size) -> bool:  # noqa: N802
        info = state._obj
        self.qos_calls.append((handle, information_class, info.ControlMask, info.StateMask))
        return self.results["qos"]


def test_sets_above_normal_priority_and_turns_execution_throttling_off() -> None:
    kernel = _Kernel()

    assert raise_process_qos("win32", kernel) is True
    assert kernel.priority_calls == [("this-process", ABOVE_NORMAL_PRIORITY_CLASS)]
    # ProcessPowerThrottling, control the execution-speed bit, state off.
    assert kernel.qos_calls == [("this-process", 4, EXECUTION_SPEED, 0)]


@pytest.mark.parametrize("priority, qos", [(False, True), (True, False)])
def test_a_failure_is_logged_and_navpy_keeps_running(
    caplog, monkeypatch, priority, qos,
) -> None:
    # get_last_error exists only on Windows; the fake kernel runs anywhere.
    monkeypatch.setattr(windows_process_qos.ctypes, "get_last_error", lambda: 5, raising=False)
    kernel = _Kernel(priority, qos)

    assert raise_process_qos("win32", kernel) is False
    assert len(kernel.priority_calls) == len(kernel.qos_calls) == 1  # both still tried
    assert "failed" in caplog.text


def test_other_platforms_are_unchanged(monkeypatch) -> None:
    monkeypatch.setattr(windows_process_qos, "_kernel32", pytest.fail)

    assert raise_process_qos("linux") is False


def test_navpy_main_runs_above_normal_at_high_qos() -> None:
    """The real calls, in the process that runs ``navpy.main.main``.

    A child is used so the test runner keeps its own policy. The child stops
    ``compose_runtime`` before it builds anything and reports its state.
    """
    if sys.platform != "win32":
        pytest.skip("Windows scheduling policy")
    script = (
        "import ctypes, navpy.main as m\n"
        "from navpy.core.kernel.windows_process_qos import PowerThrottlingState\n"
        "class Stop(Exception): pass\n"
        "def stop(_args): raise Stop\n"
        "m.compose_runtime = stop\n"
        "try:\n"
        "    m.main(None)\n"
        "except Stop:\n"
        "    pass\n"
        "k = ctypes.WinDLL('kernel32'); k.GetCurrentProcess.restype = ctypes.c_void_p\n"
        "me = ctypes.c_void_p(k.GetCurrentProcess())\n"
        "s = PowerThrottlingState(1, 0, 0)\n"
        "assert k.GetProcessInformation(me, 4, ctypes.byref(s), ctypes.sizeof(s))\n"
        "print(hex(k.GetPriorityClass(me)), s.ControlMask & 1, s.StateMask & 1)\n"
    )
    out = subprocess.run(
        [sys.executable, "-c", script], capture_output=True, text=True, check=True,
        env={"PYTHONPATH": str(_SRC), "SYSTEMROOT": os.environ["SYSTEMROOT"]},
    ).stdout.split()
    assert out[-3:] == [hex(ABOVE_NORMAL_PRIORITY_CLASS), "1", "0"]
