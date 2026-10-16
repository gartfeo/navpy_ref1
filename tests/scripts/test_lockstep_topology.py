"""The lockstep fleet's topology inputs (``lockstep_fleet.md``, Wall topology to match, Homes).

The homes fixture was produced by the deployed ``run_swarm.sh``'s own
``home_coords`` and carries that script's sha256: a re-pinned script needs
a re-recorded fixture. The live check (WSL) is opt-in: ``NAVPY_LOCKSTEP_SITL=1``.
"""

from __future__ import annotations

import hashlib
import json
import os
import shlex
from pathlib import Path

import pytest

from navpy.sim.lockstep.coordinator_queues import FIFO_CAP
from navpy.sim.lockstep.router_entries import TABLE_FILE
from scripts import lockstep_topology as topology
from scripts import lockstep_wsl
from scripts.eval_gcs_demo_models import RegressionError
from scripts.eval_gcs_demo_scenario import CERTIFIED_SCENARIO_PATH, load_scenario
from scripts.swarm_run_cli import build_parser

FIXTURES = Path(__file__).resolve().parents[1] / "fixtures" / "lockstep"
HOMES = json.loads((FIXTURES / "run_swarm_homes.json").read_text(encoding="utf-8"))


def test_the_homes_fixture_belongs_to_the_pinned_script():
    assert HOMES["run_swarm_sha256"] == topology.RUN_SWARM_SHA256


@pytest.mark.parametrize("case", HOMES["cases"], ids=lambda c: f"{c['home_coords']}-d{c['dist_m']}")
def test_fleet_homes_reproduce_the_scripts_home_coords(case):
    homes = topology.fleet_homes(case["home_coords"], len(case["homes"]), case["dist_m"])
    assert list(homes) == case["homes"]


def test_the_fleet_flies_from_the_wall_evals_homes():
    """The wall eval passes no ``-d``: swarm_run's default spacing reaches the script."""
    assert build_parser(1).parse_args([]).dist == topology.WALL_SWARM_DIST_M
    base = load_scenario(CERTIFIED_SCENARIO_PATH).home
    assert list(topology.fleet_homes(base, 3)) == HOMES["cases"][0]["homes"][:3]


def test_a_changed_script_is_refused():
    topology.require_pinned_script(topology.RUN_SWARM_SHA256)
    with pytest.raises(RegressionError, match="run_swarm.sh changed"):
        topology.require_pinned_script("0" * 64)


def test_the_topology_section_names_every_wiring_input():
    homes = ("a", "b", "c")
    section = topology.topology_inputs(homes, "f" * 64)
    parity = json.loads((FIXTURES / "router_parity.json").read_text(encoding="utf-8"))
    assert section == {
        "run_swarm_sha256": "f" * 64,
        "homes": ["a", "b", "c"],
        "router_entries_sha256": hashlib.sha256(TABLE_FILE.read_bytes()).hexdigest(),
        "router_binary_sha256": parity["binary_sha256"],
        "fifo_cap": FIFO_CAP,
    }


@pytest.mark.skipif(os.environ.get("NAVPY_LOCKSTEP_SITL") != "1",
                    reason="reads the deployed run_swarm.sh in WSL: set NAVPY_LOCKSTEP_SITL=1")
def test_the_deployed_script_is_the_pinned_one_and_its_home_coords_match():
    assert topology.run_swarm_sha256() == topology.RUN_SWARM_SHA256
    case = HOMES["cases"][0]
    home = lockstep_wsl.wsl('printf %s "$HOME"')
    script = (f"eval \"$(sed -n '/^home_coords() {{/,/^}}/p' "
              f"{shlex.quote(home + '/' + topology.RUN_SWARM_SCRIPT)})\"; "
              f"HOME_COORDS={shlex.quote(case['home_coords'])}; DIST_M={case['dist_m']}; "
              "for i in 1 2 3; do home_coords $i; echo; done")
    assert lockstep_wsl.wsl(script).split() == list(topology.fleet_homes(case["home_coords"], 3))
