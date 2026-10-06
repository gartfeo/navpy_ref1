"""The scratch harnesses must not arm before the EKF owns its NED origin.

`test_eval_nav_origin.py` pins the gate itself. This pins the OTHER family of
callers. `scratch_sitl_uav._arm` is reached from its own `run` and from
`scratch_navigation_uav`, both of which set ARMING_CHECK to 0 first -- so
ArduPilot's own pre-arm refusal, the thing that would normally hold the ground
until the filter has an origin, is not there. An origin taken in the air is
permanent (`NavEKF3_core::setOrigin` rejects a second one) and biases every
altitude the aircraft reports for the rest of the flight.

The nonzero GLOBAL_POSITION_INT that `_wait_ready` already waits for does not
close this: AHRS falls back to DCM/GPS and publishes coordinates with no EKF
origin behind them.
"""

from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

import pytest
from pymavlink import mavutil

from scripts import eval_nav_origin as nav_origin
from scripts import scratch_navigation_uav
from scripts import scratch_sitl_scale
from scripts import scratch_sitl_uav
from tests.scripts.test_eval_navigation_telemetry import SYSID, _FakeMaster
from tests.scripts.test_eval_nav_origin import _PRE_ORIGIN, _READY, _ack_from, _ekf


# Captured at import, BEFORE `_bounded_gate` shortens the gate for the arm
# tests: this is the bound the harness actually ships and budgets against.
SHIPPED_GATE_S = nav_origin.NAV_SOLUTION_TIMEOUT_S


@pytest.fixture(autouse=True)
def _bounded_gate(monkeypatch):
    """The shipped 120 s wait is a flight budget, not a test budget."""
    monkeypatch.setattr(nav_origin, "NAV_SOLUTION_TIMEOUT_S", 0.2)


def _heartbeat(armed: bool, sysid: int = SYSID) -> SimpleNamespace:
    return SimpleNamespace(
        get_type=lambda: "HEARTBEAT",
        get_srcSystem=lambda: sysid,
        base_mode=mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED if armed else 0,
    )


def _link(messages, locked_to: int = SYSID) -> _FakeMaster:
    """A link in the state `_connect` actually hands to `_arm`.

    pymavlink does not leave target_system at 0: `mavutil.post_message` locks
    it onto the FIRST vehicle heartbeat the link decodes (`if self.sysid == 0:
    self.sysid = src_system`), which on a dedicated per-vehicle link is this
    aircraft and on a shared one is whoever spoke first. `locked_to` is how
    that second case is written here. target_component is never adopted at
    all and stays 0. Both halves reproduced against a real `mavfile` over a
    loopback link: 0/0 before the heartbeat, 121/0 after it.
    """
    link = _FakeMaster(messages)
    link.target_system = locked_to
    link.target_component = 0
    return link


def _commands(link: _FakeMaster) -> list[int]:
    return [args[2] for args in link.sent]


def test_the_gate_clears_before_the_arm_command_is_sent() -> None:
    link = _link([_ack_from(SYSID), _ekf(_READY), _heartbeat(armed=True)])
    armed, attempts, _acks = scratch_sitl_uav._arm(
        link, SYSID, attempts=3, timeout_s=1.0
    )
    assert armed and attempts == 1
    # ORDER is the assertion: the stream request the gate makes, and only then
    # the arm. A gate that ran after the arm would pass every other check here.
    assert _commands(link) == [
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    ]


def test_a_missing_origin_refuses_and_the_arm_is_never_sent() -> None:
    """The aircraft is answering and would arm; it is never asked to."""
    link = _link(
        [_ack_from(SYSID)]
        + [_ekf(_PRE_ORIGIN) for _ in range(3)]
        + [_heartbeat(armed=True)]
    )
    with pytest.raises(RuntimeError, match=f"vehicle {SYSID} did not report"):
        scratch_sitl_uav._arm(link, SYSID, attempts=3, timeout_s=1.0)
    assert mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM not in _commands(link)


def test_refusing_is_raised_not_reported_as_a_failed_arm() -> None:
    """`main` files it as the run's error; a False would be retried instead.

    `_arm`'s own contract for "this aircraft would not arm" is
    `(False, attempts, acks)`, and `run` records that as `never armed` next to
    the firmware's STATUSTEXT. A missing origin is not that: nothing about the
    aircraft is wrong, and the attempts must not be spent.
    """
    link = _link([_ack_from(SYSID), _ekf(_PRE_ORIGIN)])
    with pytest.raises(RuntimeError, match="an origin captured in flight"):
        scratch_sitl_uav._arm(link, SYSID, attempts=3, timeout_s=1.0)


def test_the_gate_is_addressed_and_filtered_at_this_aircraft() -> None:
    """Re-addressed, not inherited: the lock pymavlink made can be the wrong
    aircraft, and the component it left at 0 is MAV_COMP_ID_ALL -- every
    component of the receiving system, not the autopilot the arm below goes to.
    """
    link = _link(
        [_ack_from(SYSID), _ekf(_READY), _heartbeat(armed=True)],
        locked_to=SYSID + 1,
    )
    scratch_sitl_uav._arm(link, SYSID, attempts=1, timeout_s=1.0)
    assert (link.target_system, link.target_component) == (SYSID, 1)
    # Not a broadcast: the stream request carries this aircraft's own address,
    # on the same component the arm command below is sent to.
    assert link.sent[0][:2] == (SYSID, 1)


def test_a_ready_report_from_a_neighbour_does_not_open_the_gate() -> None:
    """The connection is a CLI argument, so it can be a shared router port.

    A sibling on the same wire reaching its origin says nothing about this
    aircraft, and accepting it would arm exactly the vehicle that is not ready.
    """
    link = _link(
        [_ack_from(SYSID), _ekf(_READY, sysid=SYSID + 1), _heartbeat(armed=True)]
    )
    with pytest.raises(RuntimeError, match=f"vehicle {SYSID} did not report"):
        scratch_sitl_uav._arm(link, SYSID, attempts=1, timeout_s=1.0)
    assert mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM not in _commands(link)


def test_the_navigation_harness_arms_through_the_same_gated_function() -> None:
    """It imports `_arm` rather than owning one, so one gate covers both."""
    assert scratch_navigation_uav._arm is scratch_sitl_uav._arm


def test_the_gate_is_summed_into_the_navigation_budget(monkeypatch) -> None:
    """`worst_case_wall_s` is the number the parent sizes its kill timer from.

    `scratch_navigation_launch` adds only TEARDOWN_MARGIN_S (60 s) on top, and
    that margin exists so the child can WRITE result.json after its own guard
    fires -- it is not slack for a 120 s wait nobody budgeted. A run held
    legitimately in the gate would be killed, producing the no-result-no-reason
    outcome that budget's docstring already records three times.

    Measured as a CONTRIBUTION rather than looked for as a name. Reading the
    source cannot settle this: `inspect.getsource` keeps comments, so a
    substring check passes on a commented-out term, and walking the returned
    expression still passes on a term that is subtracted or multiplied by
    zero. Moving the bound and re-reading the total catches all four.
    """
    # Read from the gate, never retyped, so the two cannot drift apart.
    assert scratch_navigation_uav.NAV_SOLUTION_TIMEOUT_S == SHIPPED_GATE_S
    options = scratch_navigation_uav._parser().parse_args(
        ["--connection", "udp:0.0.0.0:1", "--sysid", str(SYSID)]
    )
    before = scratch_navigation_uav.worst_case_wall_s(options)
    monkeypatch.setattr(
        scratch_navigation_uav, "NAV_SOLUTION_TIMEOUT_S", SHIPPED_GATE_S + 300.0
    )
    assert scratch_navigation_uav.worst_case_wall_s(options) - before == 300.0


def test_the_scale_parent_inherits_the_gate_by_launching_the_child(
    tmp_path, monkeypatch
) -> None:
    """`scratch_sitl_scale` needs no gate of its own, and this says why.

    It holds no MAVLink link at all -- it launches one `scratch_sitl_uav`
    process per aircraft, and each child arms itself through the gated `_arm`.
    Both halves are asserted: a parent that grew its own link could arm behind
    the gate while the spawn assertion still passed.
    """
    source = Path(scratch_sitl_scale.__file__).read_text(encoding="utf-8")
    assert "pymavlink" not in source
    assert "command_long_send" not in source

    captured: dict[str, list[str]] = {}

    class _FakePopen:
        def __init__(self, command, **kwargs) -> None:
            captured["command"] = command
            for stream in ("stdout", "stderr"):
                kwargs[stream].close()

    monkeypatch.setattr(scratch_sitl_scale.subprocess, "Popen", _FakePopen)
    options = scratch_sitl_scale._parser().parse_args([])
    scratch_sitl_scale._spawn(
        Path(sys.executable), tmp_path, SYSID, -20.0, None, options
    )
    command = captured["command"]
    assert Path(command[1]).name == "scratch_sitl_uav.py"
    assert command[command.index("--sysid") + 1] == str(SYSID)
