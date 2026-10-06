"""The eval harness must not arm before the EKF owns its NED origin."""

from __future__ import annotations

from types import SimpleNamespace

import pytest
from pymavlink import mavutil

from scripts import eval_navigation_telemetry as telemetry
from scripts import eval_nav_origin as nav_origin
from tests.scripts.test_eval_navigation_telemetry import SYSID, _FakeMaster, _ack


def _ekf(flags: int, sysid: int = SYSID) -> SimpleNamespace:
    return SimpleNamespace(
        get_type=lambda: "EKF_STATUS_REPORT",
        get_srcSystem=lambda: sysid,
        flags=flags,
    )


class _ClockedMaster(_FakeMaster):
    """Spends exactly the timeout it is handed, so a wait's cost is countable.

    ``_FakeMaster`` answers instantly, which hides how long a wait would
    really have blocked.
    """

    def __init__(self, clock: list[float], messages: object = ()) -> None:
        super().__init__(messages)
        self._clock = clock

    def recv_match(self, type=None, blocking=False, timeout=None):
        self._clock[0] += timeout or 0.0
        return super().recv_match(type=type, blocking=blocking, timeout=timeout)


class _StallingMaster(_ClockedMaster):
    """Overruns every timeout it is handed, the way a slow read does.

    pymavlink checks its timeout before reading and not after, so a receive
    that began inside the budget can still return once the budget has passed.
    """

    def __init__(self, clock: list[float], overrun: float, messages: object = ()) -> None:
        super().__init__(clock, messages)
        self._overrun = overrun

    def recv_match(self, type=None, blocking=False, timeout=None):
        message = super().recv_match(type=type, blocking=blocking, timeout=timeout)
        # Only the read that hands back the report overruns, which is the case
        # the review reproduced: an ACK stall would exhaust the budget before
        # the report loop ever runs and would prove nothing about acceptance.
        if message is not None and message.get_type() == "EKF_STATUS_REPORT":
            self._clock[0] += self._overrun
        return message


@pytest.fixture
def clock(monkeypatch: pytest.MonkeyPatch) -> list[float]:
    """Drive ``time.monotonic`` from the fake transport's own spending."""
    ticks = [0.0]
    monkeypatch.setattr(nav_origin.time, "monotonic", lambda: ticks[0])
    return ticks


def _ack_from(sysid: int) -> SimpleNamespace:
    return SimpleNamespace(
        get_type=lambda: "COMMAND_ACK",
        get_srcSystem=lambda: sysid,
        command=mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        result=mavutil.mavlink.MAV_RESULT_ACCEPTED,
    )


# Hardcoded on purpose: EKF_ATTITUDE=1, EKF_POS_HORIZ_ABS=16,
# EKF_POS_VERT_ABS=32.  Deriving these from NAV_SOLUTION_FLAGS would make the
# tests agree with any editing accident in the constant itself.
_READY = 1 | 16 | 32
# The state the 2026-09-05 fleet run armed in: attitude and height were
# already good, but the filter had no absolute horizontal position because
# its NED origin did not exist yet.
_PRE_ORIGIN = 1 | 32


def test_the_flag_constant_is_the_documented_bit_set() -> None:
    assert nav_origin.NAV_SOLUTION_FLAGS == _READY


def test_absolute_solution_flags_are_ready() -> None:
    assert nav_origin.nav_solution_ready(_READY)


@pytest.mark.parametrize("missing", (1, 16, 32))
def test_each_required_bit_is_individually_load_bearing(missing: int) -> None:
    assert not nav_origin.nav_solution_ready(_READY & ~missing)


def test_uninitialised_filter_is_not_ready_even_with_every_flag() -> None:
    assert not nav_origin.nav_solution_ready(_READY | 1024)


def test_wait_requests_the_ekf_status_stream_then_accepts_a_ready_report() -> None:
    master = _FakeMaster([_ack(mavutil.mavlink.MAV_RESULT_ACCEPTED), _ekf(_READY)])
    assert nav_origin.wait_for_nav_solution(master)
    args = master.sent[0]
    assert args[4] == mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT
    assert args[5] == 200000


def test_pre_origin_reports_are_waited_out_not_accepted() -> None:
    master = _FakeMaster(
        [_ack(mavutil.mavlink.MAV_RESULT_ACCEPTED)]
        + [_ekf(_PRE_ORIGIN) for _ in range(4)]
        + [_ekf(_READY)]
    )
    assert nav_origin.wait_for_nav_solution(master)


def test_a_denied_stream_request_still_listens_for_the_default_stream() -> None:
    # ArduPlane streams EKF_STATUS_REPORT at 1 Hz by default (EXTRA3), so a
    # rejected interval request must not abort the wait.
    master = _FakeMaster([_ack(mavutil.mavlink.MAV_RESULT_DENIED), _ekf(_READY)])
    assert nav_origin.wait_for_nav_solution(master, timeout_s=0.2)


def test_wait_times_out_while_the_origin_is_still_missing() -> None:
    master = _FakeMaster(
        [_ack(mavutil.mavlink.MAV_RESULT_ACCEPTED)]
        + [_ekf(_PRE_ORIGIN) for _ in range(5)]
    )
    assert not nav_origin.wait_for_nav_solution(master, timeout_s=0.2)


def test_wait_ignores_a_ready_report_from_another_vehicle() -> None:
    master = _FakeMaster(
        [_ack(mavutil.mavlink.MAV_RESULT_ACCEPTED), _ekf(_READY, sysid=SYSID + 1)]
    )
    assert not nav_origin.wait_for_nav_solution(master, timeout_s=0.2)


def test_require_raises_with_the_vehicle_named() -> None:
    master = _FakeMaster([])
    with pytest.raises(RuntimeError, match="vehicle 121 did not report"):
        nav_origin.require_nav_solution(master, 121, timeout_s=0.05)


def _select(master, sys_id):
    master.target_system = sys_id
    master.target_component = 1


def test_fleet_gate_clears_every_vehicle_through_the_real_chain() -> None:
    rows = []
    for sysid in (121, 122, 123):
        rows += [_ack_from(sysid), _ekf(_READY, sysid=sysid)]
    master = _FakeMaster(rows)
    nav_origin.require_fleet_nav_solution(
        master, [121, 122, 123], select=_select
    )
    assert [args[0] for args in master.sent] == [121, 122, 123]


def test_fleet_gate_fails_on_the_vehicle_that_never_becomes_ready() -> None:
    rows = [
        _ack_from(121), _ekf(_READY, sysid=121),
        _ack_from(122), _ekf(_PRE_ORIGIN, sysid=122),
    ]
    master = _FakeMaster(rows)
    with pytest.raises(RuntimeError, match="vehicle 122 did not report"):
        nav_origin.require_fleet_nav_solution(
            master, [121, 122, 123], select=_select, timeout_s=0.05
        )


# 0.4 s is shorter than one receive slice, 3.0 s shorter than the ACK wait's
# own default, and 6.25 s long enough to reach the report loop: the three
# regimes the round-3 review measured overrunning.
@pytest.mark.parametrize("budget", (0.4, 3.0, 6.25))
def test_the_wait_never_outlives_its_advertised_timeout(
    clock: list[float],
    budget: float,
) -> None:
    """A silent link must cost the caller its budget and not a byte more.

    The stream request used to run on its own five-second default outside the
    deadline, and every report receive could overrun it by a further second,
    so a three-second wait spent five seconds and a 6.25-second wait seven.
    """
    master = _ClockedMaster(clock)

    assert nav_origin.wait_for_nav_solution(master, timeout_s=budget) is False
    assert clock[0] == pytest.approx(budget)


def test_a_report_arriving_inside_the_budget_is_still_accepted(
    clock: list[float],
) -> None:
    """Clamping the waits must not cost a vehicle that answers in time."""
    master = _ClockedMaster(
        clock,
        [_ack(mavutil.mavlink.MAV_RESULT_ACCEPTED), _ekf(_READY)],
    )

    assert nav_origin.wait_for_nav_solution(master, timeout_s=5.0)
    assert clock[0] <= 5.0


def test_a_report_read_slightly_late_is_still_accepted(clock: list[float]) -> None:
    """One receive's worth of overrun must not fail a vehicle that answered."""
    master = _StallingMaster(
        clock,
        telemetry.RECV_SLICE_S / 2,
        [_ack(mavutil.mavlink.MAV_RESULT_ACCEPTED), _ekf(_READY)],
    )

    assert nav_origin.wait_for_nav_solution(master, timeout_s=1.0)


def test_a_report_read_long_after_the_deadline_is_refused(clock: list[float]) -> None:
    """Round-4 review: a stalled read returned True at 5.406 s on a 5 s budget.

    Accepting readiness whenever it eventually appears makes the advertised
    timeout meaningless, so the tolerance is bounded by one receive slice.
    """
    master = _StallingMaster(
        clock,
        telemetry.RECV_SLICE_S * 4,
        [_ack(mavutil.mavlink.MAV_RESULT_ACCEPTED), _ekf(_READY)],
    )

    assert nav_origin.wait_for_nav_solution(master, timeout_s=1.0) is False


def test_an_exhausted_budget_sends_no_command(clock: list[float]) -> None:
    """Round-5 review: a zero budget still sent, costing 2.0 s on a slow link."""
    master = _ClockedMaster(clock)

    assert nav_origin.wait_for_nav_solution(master, timeout_s=0.0) is False
    assert master.sent == []
    assert clock[0] == 0.0
