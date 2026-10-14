"""The fleet actor's per-UAV mapping (``lockstep_fleet.md``, item 4; step 10e).

Devices and ordinals follow the fleet's naming scheme for any N, the
operator's context reads each sysid's own NavPy tail, and the slice is the
N = 1 layout with its role configs unchanged (their bytes are in its spawn
records, so in its digest).
"""

from __future__ import annotations

from pathlib import Path

import pytest

from navpy.sim.lockstep.coordinator_types import Manifest, RoleSpec
from navpy.sim.lockstep.operator_baton import OperatorBaton
from navpy.sim.lockstep.participant_hooks import DEVICE_KEY, RequestIds
from navpy.sim.lockstep.pose_recording import POSE_CADENCE_DIR
from scripts.eval_gcs_demo_config import build_settings
from scripts.eval_gcs_demo_scenario import CERTIFIED_SCENARIO_PATH, load_scenario
from scripts.eval_lockstep_fleet_backend import load_fleet_scenario
from scripts.lockstep_fleet_actor import BACKEND_HOOKS, FleetLayout, backend_role, stack_context
from scripts.lockstep_slice_actor import SLICE, companion_argv, slice_roles

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


def test_the_slice_roles_keep_their_config_bytes():
    roles = slice_roles(SLICE_SCENARIO, navpy=True, pose_cadence_dir="poses")
    settings = build_settings(SLICE_SCENARIO)
    argv = companion_argv(settings)
    backend = roles["backend"]
    assert list(roles) == ["navpy", "backend"]
    assert list(backend.config) == [DEVICE_KEY, "settings", POSE_CADENCE_DIR, "companions"]
    assert backend.config["companions"] == {"1": {"role": "navpy", "ordinal": 1, "device": "lockstep:serial0.1",
                                                  "argv": argv}}
    assert list(roles["navpy"].config) == [DEVICE_KEY, "argv", POSE_CADENCE_DIR]
    assert list(slice_roles(SLICE_SCENARIO)["backend"].config) == [DEVICE_KEY, "settings"]


def test_the_fleet_fixture_keeps_the_certified_home_and_slots():
    """The 10e fixture freezes the certified scenario's missions (they move under
    other work); its home and slots stay the certified ones."""
    certified = load_scenario(CERTIFIED_SCENARIO_PATH)
    assert FLEET_SCENARIO.home == certified.home
    assert FLEET_SCENARIO.slots == certified.slots
