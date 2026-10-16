"""The 1 ms timer request also opts out of Windows timer-resolution throttling.

Windows 11 can stop honouring timeBeginPeriod mid-run for a process it treats
as invisible; the sim poll loop then waits on the 15.6 ms tick.  These tests
drive the request/release helpers and the ctypes adapter against fakes.
"""

from __future__ import annotations

import ctypes

import pytest

from navpy.core.kernel import windows_timer_resolution as wtr
from navpy.core.kernel.scheduler_cadence import (
    IdentitySchedulerCadence,
    SchedulerCadence,
)

IGNORE = 0x4
OTHER = 0x1  # PROCESS_POWER_THROTTLING_EXECUTION_SPEED, must be kept


class _FakeNative:
    def __init__(self, control: int = 0, state: int = 0) -> None:
        self.policy = (control, state)
        self.calls: list[tuple] = []
        self.begin_result: object = 0
        self.end_result: object = 0
        self.get_error = False
        self.set_errors = 0

    def time_begin_period(self, period_ms: int) -> int:
        self.calls.append(("begin", period_ms))
        if isinstance(self.begin_result, Exception):
            raise self.begin_result
        return self.begin_result  # type: ignore[return-value]

    def time_end_period(self, period_ms: int) -> int:
        self.calls.append(("end", period_ms))
        if isinstance(self.end_result, Exception):
            raise self.end_result
        return self.end_result  # type: ignore[return-value]

    def get_power_throttling(self) -> tuple[int, int]:
        self.calls.append(("get",))
        if self.get_error:
            raise OSError(87, "get failed")
        return self.policy

    def set_power_throttling(self, control: int, state: int) -> None:
        self.calls.append(("set", control, state))
        if self.set_errors:
            self.set_errors -= 1
            raise OSError(5, "set failed")
        self.policy = (control, state)


@pytest.fixture
def native(monkeypatch) -> _FakeNative:
    fake = _FakeNative()
    monkeypatch.setattr(wtr, "_native_timer", lambda: fake)
    monkeypatch.setattr(wtr, "_timer_resolution_refcount", 0)
    monkeypatch.setattr(wtr, "_saved_power_throttling", None)
    return fake


def _clock() -> float:
    return 20.0


def test_request_opts_out_and_release_restores_saved_policy(native) -> None:
    native.policy = (OTHER, OTHER)

    assert wtr.request_high_resolution_timer() is True
    assert native.policy == (OTHER | IGNORE, OTHER)
    assert native.calls == [("get",), ("set", OTHER | IGNORE, OTHER), ("begin", 1)]

    wtr.release_high_resolution_timer()
    assert native.calls[-2:] == [("end", 1), ("set", OTHER, OTHER)]
    assert native.policy == (OTHER, OTHER)


def test_opt_out_clears_an_existing_ignore_state(native) -> None:
    native.policy = (IGNORE, IGNORE)

    wtr.request_high_resolution_timer()

    assert native.policy == (IGNORE, 0)
    wtr.release_high_resolution_timer()
    assert native.policy == (IGNORE, IGNORE)


def test_overlapping_leases_touch_native_state_once(native) -> None:
    assert wtr.request_high_resolution_timer() is True
    assert wtr.request_high_resolution_timer() is True
    wtr.release_high_resolution_timer()
    assert ("end", 1) not in native.calls
    assert native.policy == (IGNORE, 0)

    wtr.release_high_resolution_timer()
    wtr.release_high_resolution_timer()  # extra release is a no-op

    names = [call[0] for call in native.calls]
    assert names == ["get", "set", "begin", "end", "set"]
    assert native.policy == (0, 0)


def test_policy_read_failure_skips_opt_out_but_keeps_timer(native, caplog) -> None:
    native.get_error = True

    assert wtr.request_high_resolution_timer() is True
    wtr.release_high_resolution_timer()

    assert [call[0] for call in native.calls] == ["get", "begin", "end"]
    assert "throttled" in caplog.text


def test_policy_write_failure_keeps_timer_and_restores_nothing(native, caplog) -> None:
    native.set_errors = 1

    assert wtr.request_high_resolution_timer() is True
    wtr.release_high_resolution_timer()

    assert [call[0] for call in native.calls] == ["get", "set", "begin", "end"]
    assert native.policy == (0, 0)
    assert "throttled" in caplog.text


@pytest.mark.parametrize("failure", [97, OSError("no winmm")])
def test_timer_failure_rolls_policy_back(native, caplog, failure) -> None:
    native.policy = (OTHER, 0)
    native.begin_result = failure

    assert wtr.request_high_resolution_timer() is False

    assert native.policy == (OTHER, 0)
    assert native.calls[-1] == ("set", OTHER, 0)
    assert wtr._timer_resolution_refcount == 0
    assert "timeBeginPeriod" in caplog.text


@pytest.mark.parametrize("failure", [97, OSError("no winmm")])
def test_restore_runs_when_time_end_period_fails(native, caplog, failure) -> None:
    native.end_result = failure
    wtr.request_high_resolution_timer()

    wtr.release_high_resolution_timer()

    assert native.calls[-1] == ("set", 0, 0)
    assert native.policy == (0, 0)
    assert "timeEndPeriod" in caplog.text


def test_restore_failure_is_logged(native, caplog) -> None:
    wtr.request_high_resolution_timer()
    native.set_errors = 1

    wtr.release_high_resolution_timer()

    assert "Restoring the power throttling policy failed" in caplog.text
    assert wtr._timer_resolution_refcount == 0


def test_missing_native_api_requests_nothing(monkeypatch) -> None:
    monkeypatch.setattr(wtr, "_native_timer", lambda: None)
    monkeypatch.setattr(wtr, "_timer_resolution_refcount", 0)

    assert wtr.request_high_resolution_timer() is False
    wtr.release_high_resolution_timer()
    assert wtr._timer_resolution_refcount == 0


def test_non_windows_has_no_native_timer(monkeypatch) -> None:
    monkeypatch.setattr(wtr.os, "name", "posix")

    assert wtr._native_timer() is None


def test_accelerated_cadence_holds_opt_out_until_close(native) -> None:
    cadence = SchedulerCadence(lambda: 10.0, wall_clock=_clock)
    assert native.policy == (IGNORE, 0)

    cadence.close()
    cadence.close()

    assert native.policy == (0, 0)
    assert [call[0] for call in native.calls] == ["get", "set", "begin", "end", "set"]


@pytest.mark.parametrize(
    "make",
    [
        lambda: SchedulerCadence(lambda: 1.0, wall_clock=_clock),
        IdentitySchedulerCadence,
    ],
    ids=["realtime", "lockstep"],
)
def test_realtime_and_lockstep_make_no_native_calls(native, make) -> None:
    make().close()

    assert native.calls == []


# --- the ctypes adapter, against fake DLL exports -------------------------


class _FakeFunction:
    def __init__(self, impl) -> None:
        self.impl = impl
        self.argtypes = None
        self.restype = None

    def __call__(self, *args):
        return self.impl(*args)


class _FakeDll:
    def __init__(self, **functions) -> None:
        for name, impl in functions.items():
            setattr(self, name, _FakeFunction(impl))


def _fake_kernel32(record: list, *, get_ok: int = 1, set_ok: int = 1) -> _FakeDll:
    def get(handle, info_class, ref, size):
        state = ref._obj
        record.append(("get", handle, info_class, size, state.Version))
        state.ControlMask, state.StateMask = 0x5, 0x1
        return get_ok

    def set_(handle, info_class, ref, size):
        state = ref._obj
        record.append(
            ("set", handle, info_class, size,
             state.Version, state.ControlMask, state.StateMask),
        )
        return set_ok

    return _FakeDll(
        GetCurrentProcess=lambda: -1,
        GetProcessInformation=get,
        SetProcessInformation=set_,
    )


def _install_dlls(monkeypatch, kernel32: _FakeDll) -> dict:
    loaded: dict = {}
    winmm = _FakeDll(timeBeginPeriod=lambda ms: 0, timeEndPeriod=lambda ms: 97)

    def win_dll(name, use_last_error=False):
        loaded[name] = use_last_error
        return {"winmm": winmm, "kernel32": kernel32}[name]

    monkeypatch.setattr(wtr.ctypes, "WinDLL", win_dll, raising=False)
    monkeypatch.setattr(wtr.ctypes, "get_last_error", lambda: 5, raising=False)
    return loaded


def test_adapter_binds_signatures_and_passes_the_policy_struct(monkeypatch) -> None:
    from ctypes import wintypes

    record: list = []
    kernel32 = _fake_kernel32(record)
    loaded = _install_dlls(monkeypatch, kernel32)

    native = wtr._WindowsTimerNative()

    assert loaded == {"winmm": False, "kernel32": True}
    assert kernel32.SetProcessInformation.restype is wintypes.BOOL
    assert kernel32.GetCurrentProcess.restype is wintypes.HANDLE
    assert kernel32.SetProcessInformation.argtypes == [
        wintypes.HANDLE, ctypes.c_int, ctypes.c_void_p, wintypes.DWORD,
    ]
    assert native.time_begin_period(1) == 0
    assert native.time_end_period(1) == 97
    assert native.get_power_throttling() == (0x5, 0x1)
    native.set_power_throttling(0x5, 0x1)
    assert record == [
        ("get", -1, 4, 12, 1),
        ("set", -1, 4, 12, 1, 0x5, 0x1),
    ]


@pytest.mark.parametrize("failing", ["get", "set"])
def test_adapter_raises_with_last_error_when_bool_is_zero(monkeypatch, failing) -> None:
    kernel32 = _fake_kernel32(
        [], get_ok=int(failing != "get"), set_ok=int(failing != "set"),
    )
    _install_dlls(monkeypatch, kernel32)
    native = wtr._WindowsTimerNative()

    with pytest.raises(OSError) as raised:
        if failing == "get":
            native.get_power_throttling()
        else:
            native.set_power_throttling(0x4, 0)

    assert raised.value.errno == 5


def test_missing_export_disables_native_timer_once(monkeypatch, caplog) -> None:
    kernel32 = _FakeDll(GetCurrentProcess=lambda: -1)  # no Get/SetProcessInformation
    _install_dlls(monkeypatch, kernel32)
    monkeypatch.setattr(wtr.os, "name", "nt")
    monkeypatch.setattr(wtr, "_native_timer_cache", None)
    monkeypatch.setattr(wtr, "_native_timer_unavailable", False)

    assert wtr._native_timer() is None
    assert wtr._native_timer() is None

    assert caplog.text.count("Windows timer API unavailable") == 1


@pytest.mark.skipif(wtr.os.name != "nt", reason="real Windows API")
def test_real_windows_request_holds_opt_out_and_restores(monkeypatch) -> None:
    monkeypatch.setattr(wtr, "_timer_resolution_refcount", 0)
    monkeypatch.setattr(wtr, "_saved_power_throttling", None)
    native = wtr._native_timer()
    assert native is not None
    before = native.get_power_throttling()

    assert wtr.request_high_resolution_timer() is True
    try:
        control, state = native.get_power_throttling()
        assert control & IGNORE and not state & IGNORE
    finally:
        wtr.release_high_resolution_timer()

    assert native.get_power_throttling() == before
