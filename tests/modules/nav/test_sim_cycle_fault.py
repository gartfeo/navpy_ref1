"""The SITL-only NAV cycle fault hook (navpy/sim/cycle_fault.py) for the recovery checks of navpy_recovery.md, step 9."""
from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from navpy.app.lockstep import navpy_host
from navpy.app.navpy_argparse import make_parser
from navpy.core.kernel.config.nav_args import NavArgs
from tests.args.test_nav_args import _make_vehicle
from navpy.sim.cycle_fault import (
    FaultingCycle,
    SimCycleFaultInjected,
    SimCycleFaultRefused,
    sim_cycle_fault,
)
from tests.logger.recording_logger import RecordingLogger

SERIAL0 = "lockstep:serial0.1"
WALL = ["-c", "tcp:127.0.0.1:5762", "-ss", "1"]


def _sim(simulated: bool = True) -> SimpleNamespace:
    return SimpleNamespace(is_simulated_autopilot=Mock(return_value=simulated))


# ---------- off by default ----------


def test_off_by_default_the_cycle_is_not_wrapped_and_nothing_is_asked():
    args = make_parser().parse_args(WALL)
    assert args.sim_cycle_fault_file is None
    assert NavArgs(args, _make_vehicle()).sim_cycle_fault_file is None
    cycle, vehicle, logger = Mock(), _sim(), RecordingLogger()
    assert sim_cycle_fault(cycle, args.sim_cycle_fault_file, vehicle, logger) is cycle
    vehicle.is_simulated_autopilot.assert_not_called()
    assert logger.journal.records == []


def test_the_flag_reaches_the_nav_args(tmp_path):
    args = make_parser().parse_args([*WALL, "--sim-cycle-fault-file", str(tmp_path / "fault")])
    assert NavArgs(args, _make_vehicle()).sim_cycle_fault_file == str(tmp_path / "fault")


# ---------- refused where it must never run ----------


def test_refused_on_an_autopilot_that_is_not_simulated(tmp_path):
    with pytest.raises(SimCycleFaultRefused, match="SITL only"):
        sim_cycle_fault(Mock(), str(tmp_path / "fault"), _sim(False), RecordingLogger())


def test_refused_by_the_lockstep_host(tmp_path):
    argv = ["-c", SERIAL0, "-ss", "1", "--stop-on-lifecycle", "--sim-cycle-fault-file", str(tmp_path / "f")]
    with pytest.raises(ValueError, match="sim-cycle-fault-file"):
        navpy_host.parse_lockstep_args(argv, SERIAL0)


# ---------- inject and heal ----------


def test_raise_fails_each_cycle_until_the_file_is_gone(tmp_path):
    path = tmp_path / "fault"
    inner, logger = Mock(), RecordingLogger()
    cycle = sim_cycle_fault(inner, str(path), _sim(), logger)
    cycle.run()
    assert inner.run.call_count == 1
    path.write_text("raise", encoding="utf-8")
    for _ in range(3):
        with pytest.raises(SimCycleFaultInjected):
            cycle.run()
    path.unlink()
    cycle.run()
    assert inner.run.call_count == 2


def test_an_empty_file_raises_too(tmp_path):
    path = tmp_path / "fault"
    path.write_text("", encoding="utf-8")
    with pytest.raises(SimCycleFaultInjected):
        FaultingCycle(Mock(), path, lambda message: None).run()


def test_hang_blocks_the_cycle_until_healed(tmp_path):
    """A stuck NAV loop, for the in-process watchdog: the cycle does not return while the file says hang."""
    path = tmp_path / "fault"
    path.write_text("hang\n", encoding="utf-8")
    sleeps = []

    def sleep(seconds: float) -> None:
        sleeps.append(seconds)
        if len(sleeps) == 5:
            path.unlink()               # healed while hung

    inner, reports = Mock(), []
    FaultingCycle(inner, path, reports.append, sleep=sleep).run()
    assert len(sleeps) == 5 and inner.run.call_count == 1
    assert reports == ["sim cycle fault: hang", "sim cycle fault: healed"]


def test_a_hang_turned_into_raise_raises(tmp_path):
    path = tmp_path / "fault"
    path.write_text("hang", encoding="utf-8")

    def sleep(seconds: float) -> None:
        path.write_text("raise", encoding="utf-8")

    with pytest.raises(SimCycleFaultInjected):
        FaultingCycle(Mock(), path, lambda message: None, sleep=sleep).run()


# ---------- the journal ----------


def test_arming_and_each_mode_change_are_journaled_so_a_run_with_it_is_never_clean(tmp_path):
    from scripts.eval_gcs_demo_recovery import unexpected_recovery_errors

    path = tmp_path / "fault"
    logger = RecordingLogger()
    cycle = sim_cycle_fault(Mock(), str(path), _sim(), logger)
    path.write_text("raise", encoding="utf-8")
    with pytest.raises(SimCycleFaultInjected):
        cycle.run()
    with pytest.raises(SimCycleFaultInjected):
        cycle.run()
    path.unlink()
    cycle.run()
    assert logger.journal.records == [
        ("sim_fault", f"sim cycle fault hook armed: {path}"),
        ("sim_fault", "sim cycle fault: raise"),
        ("sim_fault", "sim cycle fault: healed"),
    ]
    events = [{"source": source, "message": message} for source, message in logger.journal.records]
    vehicle = SimpleNamespace(sys_id=1, recoveries=events)
    assert unexpected_recovery_errors([vehicle], faults_planned=False)    # the clean-run gate fails it


def test_a_raised_cycle_is_a_failed_nav_cycle(tmp_path):
    """Wrapped at composition, the injected failure reaches NAV recovery as a failed cycle."""
    from navpy.core.flow.navigation_loop import NavigationLoop

    path = tmp_path / "fault"
    path.write_text("raise", encoding="utf-8")
    cycle = FaultingCycle(Mock(), path, lambda message: None)
    loop = NavigationLoop(cycle, Mock(), Mock())
    assert loop._run_cycle() is False
    path.unlink()
    assert loop._run_cycle() is True
