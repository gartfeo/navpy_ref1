"""The fleet actor's per-UAV mapping (``lockstep_fleet.md``, item 4; steps 10e, 10f).

Devices and ordinals follow the fleet's naming scheme for any N, the
operator's context reads each sysid's own NavPy tail, each UAV has its own
companion role ``navpy<i>``, and the slice is the N = 1 layout (its role
configs are in its spawn records, so in its digest).
"""

from __future__ import annotations

from pathlib import Path

import pytest

from navpy.sim.lockstep.coordinator_types import Manifest, RoleSpec
from navpy.sim.lockstep.operator_baton import OperatorBaton
from navpy.sim.lockstep.participant_hooks import DEVICE_KEY, RequestIds
from navpy.sim.lockstep.pose_recording import POSE_CADENCE_DIR
from scripts.eval_gcs_demo_config import build_settings
from scripts.eval_gcs_demo_models import RegressionError
from scripts.eval_gcs_demo_scenario import CERTIFIED_SCENARIO_PATH, load_scenario
from scripts.eval_lockstep_fleet_backend import load_fleet_scenario
from scripts.lockstep_fleet_actor import BACKEND_HOOKS, FleetLayout, backend_role, fleet_roles, stack_context
from scripts.lockstep_slice_actor import SLICE, slice_argv, slice_roles

ROOT = Path(__file__).resolve().parents[2]
FLEET_SCENARIO, _PIN = load_fleet_scenario()
SLICE_SCENARIO = load_scenario(ROOT / "scripts" / "eval_lockstep_slice_scenario.json")


def test_a_layout_maps_each_uav_to_its_devices_and_ordinals():
    fleet = FleetLayout(3)
    assert fleet.sys_ids == (1, 2, 3) and fleet.backend == 4 and fleet.router == "lockstep:router"
    assert [fleet.serial0(s) for s in fleet.sys_ids] == [f"lockstep:serial0.{s}" for s in (1, 2, 3)]
    assert [fleet.navpy(s) for s in fleet.sys_ids] == [1, 2, 3]
    assert fleet.sitl_ports == (("lockstep:serial0.1", "sitl1.serial1"),
                                ("lockstep:serial0.2", "sitl2.serial1"),
                                ("lockstep:serial0.3", "sitl3.serial1"))
    with pytest.raises(ValueError, match="not in the fleet"):
        fleet.serial0(4)
    for not_an_int in (True, 1.0, "1"):
        with pytest.raises(ValueError, match="not in the fleet"):
            fleet.serial0(not_an_int)
        with pytest.raises(ValueError, match="not in the fleet"):
            fleet.navpy(not_an_int)
    with pytest.raises(ValueError, match="at least one UAV"):
        FleetLayout(0)
    assert (SLICE.sys_ids, SLICE.backend) == ((1,), 2)


def test_the_context_reads_each_sysids_own_navpy_tail():
    fleet = FleetLayout(3)
    operator = OperatorBaton(RequestIds(0), lambda _operator: None)
    context = stack_context(operator, FLEET_SCENARIO, fleet)
    assert context.sys_ids == (1, 2, 3) and context.monitor_device == "lockstep:router"
    operator.journal.log_lines.update({1: ["one"], 2: ["two"], 3: ["three"], 4: ["backend"]})
    assert [context.navigation_tail(s).read_new() for s in (1, 2, 3)] == ["one\n", "two\n", "three\n"]
    assert context.backend_tail.read_new() == "backend\n"
    with pytest.raises(ValueError):
        context.navigation_tail(4)


def test_the_backend_role_owns_the_router_for_every_vehicle():
    fleet = FleetLayout(3)
    settings = build_settings(FLEET_SCENARIO)
    spec = backend_role(settings, fleet)
    assert spec == RoleSpec(BACKEND_HOOKS, ("lockstep:router",),
                            {DEVICE_KEY: "lockstep:router", "settings": settings})
    assert settings["flight"]["uavs_per_set"] == settings["simulation"]["simulated_vehicle_count"] == 3
    assert settings["launch"]["default_channel_map"] == {}  # channels resolve to sysids 1-3
    manifest = Manifest(seed=1, epoch_us=0, max_frame_gap_us=1, drain_period_us=1, mission_budget_us=0,
                        roles={"backend": spec}, sitl_ports=fleet.sitl_ports, router=fleet.router)
    assert manifest.sitl_devices[1::2] == ("sitl1.serial1", "sitl2.serial1", "sitl3.serial1")


def test_the_slice_roles_are_the_fleet_roles_for_one_uav():
    roles = slice_roles(SLICE_SCENARIO, navpy=True, pose_cadence_dir="poses")
    settings = build_settings(SLICE_SCENARIO)
    argv = slice_argv(settings)
    backend = roles["backend"]
    assert list(roles) == ["navpy1", "backend"]
    assert list(backend.config) == [DEVICE_KEY, "settings", POSE_CADENCE_DIR, "companions"]
    assert backend.config["companions"] == {"1": {"role": "navpy1", "ordinal": 1, "device": "lockstep:serial0.1",
                                                  "argv": argv}}
    assert list(roles["navpy1"].config) == [DEVICE_KEY, "argv", POSE_CADENCE_DIR]
    assert list(slice_roles(SLICE_SCENARIO)["backend"].config) == [DEVICE_KEY, "settings"]
    assert roles == fleet_roles(settings, SLICE, navpy=True, pose_cadence_dir="poses")


def test_each_uav_has_its_own_companion_role_device_and_command_line():
    fleet = FleetLayout(3)
    settings = build_settings(SLICE_SCENARIO)
    roles = fleet_roles(settings, fleet, navpy=True)
    assert fleet.companion_roles == ("navpy1", "navpy2", "navpy3") and fleet.expected_fleet == (1, 2, 3)
    assert list(roles) == ["navpy1", "navpy2", "navpy3", "backend"]
    companions = roles["backend"].config["companions"]
    assert list(companions) == ["1", "2", "3"]
    for sys_id in fleet.sys_ids:
        spec, device = roles[f"navpy{sys_id}"], f"lockstep:serial0.{sys_id}"
        argv = spec.config["argv"]
        assert spec.devices == (device,) and spec.config[DEVICE_KEY] == device
        assert argv[argv.index("-c") + 1] == device and argv[argv.index("-ss") + 1] == str(sys_id)
        assert argv[argv.index("-nt") + 1] == "mav"  # the vehicle bus: peers over the router
        assert companions[str(sys_id)] == {"role": f"navpy{sys_id}", "ordinal": sys_id, "device": device,
                                           "argv": argv}
    manifest = Manifest(seed=1, epoch_us=0, max_frame_gap_us=1, drain_period_us=1, mission_budget_us=0,
                        roles=roles, sitl_ports=fleet.sitl_ports, router=fleet.router,
                        companion_roles=fleet.companion_roles, expected_fleet=fleet.expected_fleet)
    assert manifest.expected_fleet == (1, 2, 3)
    stand_in = fleet_roles(settings, fleet, companion=lambda sys_id: RoleSpec("toy", (fleet.serial0(sys_id),)))
    assert [stand_in[f"navpy{i}"].devices for i in fleet.sys_ids] == [(fleet.serial0(i),) for i in fleet.sys_ids]


def test_the_fleet_fixture_keeps_the_certified_home_and_slots():
    """The 10e fixture freezes the certified scenario's missions (they move under
    other work); its home and slots stay the certified ones."""
    certified = load_scenario(CERTIFIED_SCENARIO_PATH)
    assert FLEET_SCENARIO.home == certified.home
    assert FLEET_SCENARIO.slots == certified.slots


class _SimClock:
    def __init__(self) -> None:
        self.now = 0.0
        self.sleeps: list[float] = []

    def monotonic(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.sleeps.append(seconds)
        self.now += seconds


class _Tail:
    def __init__(self, clock: _SimClock, checkin_s: float | None) -> None:
        self._clock, self._checkin_s = clock, checkin_s

    @property
    def text(self) -> str:
        lines = ["2026-01-01 00:00:14,664 INFO ONHOLD: DISARMED",
                 "2026-01-01 00:00:15,000 INFO CheckInFailed: no",  # not the check-in line
                 "2026-01-01 00:00:16,000 DEBUG CheckIn"]
        if self._checkin_s is not None and self._clock.now >= self._checkin_s:
            lines.append("2026-01-01 00:00:56,584 INFO CheckIn")
        return "".join(f"{line}\n" for line in lines)


def _launch_context(monkeypatch, checkins: dict[int, float | None]):
    from types import SimpleNamespace

    from scripts import lockstep_fleet_companions as actor

    clock, calls = _SimClock(), []
    for name in ("start_esp32_simulator", "wait_for_missions", "prepare_container_launch", "trigger_all"):
        monkeypatch.setattr(actor, name, lambda context, *args, _name=name: calls.append((_name, args)))
    tails = {sys_id: _Tail(clock, at) for sys_id, at in checkins.items()}
    context = SimpleNamespace(sys_ids=tuple(checkins), clock=clock, navigation_tail=tails.__getitem__)
    return actor, context, clock, calls


def test_the_launch_runs_in_the_wall_order_and_holds_after_the_last_check_in(monkeypatch):
    actor, context, clock, calls = _launch_context(monkeypatch, {1: 56.0, 2: 79.0, 3: 107.0})
    assert actor.checked_in(context) == []
    checked_in_s = actor.launch_until_checked_in(context)
    assert calls == [("start_esp32_simulator", ()), ("wait_for_missions", (actor.VEHICLES_TIMEOUT_S,)),
                     ("prepare_container_launch", (actor.LAUNCH_TIMEOUT_S,)), ("trigger_all", ())]
    assert 107.0 <= checked_in_s < 107.5  # the first poll after the last check-in
    assert clock.sleeps[-1] == actor.PEER_HOLD_S == 2.0  # two heartbeat periods
    assert actor.checked_in(context) == [1, 2, 3]


def test_the_launch_fails_while_any_navpy_has_not_checked_in(monkeypatch):
    actor, context, clock, _calls = _launch_context(monkeypatch, {1: 56.0, 2: None, 3: 107.0})
    with pytest.raises(RegressionError, match="timed out waiting for every NavPy checked in"):
        actor.launch_until_checked_in(context)
    assert actor.checked_in(context) == [1, 3]
    assert clock.now == actor.CHECKIN_TIMEOUT_S and actor.PEER_HOLD_S not in clock.sleeps  # no hold
