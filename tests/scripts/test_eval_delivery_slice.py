"""e2b slice: the gate over synthetic NavPy logs, and the one-slot scenario."""

from __future__ import annotations

import json

import pytest

from scripts import eval_delivery_slice as runner
from scripts.eval_delivery_slice_gates import evaluate_slice
from scripts.eval_gcs_demo_config import aas_params_for
from scripts.eval_gcs_demo_models import RegressionError

DOCKS = ["d1", "d2", "home-east", "home-middle", "home-west"]
T = "2026-10-16 10:00:00,000 INFO "
FLEET = T + "Fleet docks: P1=d1; P2=d2; P3=home-east; P4=home-middle; P5=home-west"
ORBIT = T + "Self-detect orbit: poi=40.287150, 44.473707, 1290.0 orbit_r=500m"
SELECT_D1 = T + "POI: P1 (tracking obj_id=0)"
CONFIRMING = T + "CONFIRMING: P1"
OFFER_D2 = T + "Task offered: task_id=2 obj_id=1 at (40.2884990, 44.4737067) sent=True"
DWELL_END = T + "CONFIRM max dwell exceeded: under review 61.0s >= 60.0s, aborting to DETECT"


def _gate(lines):
    return evaluate_slice(lines, DOCKS, "d1", "d2")


def _passing():
    return [FLEET, ORBIT, SELECT_D1, CONFIRMING, OFFER_D2, DWELL_END]


def test_pass_records_each_evidence_line():
    result = _gate(_passing())
    assert result.passed, result.errors
    assert result.evidence["selection"] == SELECT_D1
    assert result.evidence["offer"] == OFFER_D2
    assert result.evidence["window_end"] == DWELL_END


def test_pass_with_window_still_open_at_log_end():
    result = _gate(_passing()[:-1])
    assert result.passed, result.errors
    assert result.evidence["window_end"] == "open at log end"


def test_d2_selected_first_fails():
    lines = _passing()
    lines[2] = T + "POI: P1 (tracking obj_id=1)"
    lines[4] = T + "Task offered: task_id=2 obj_id=0 at (40.2871501, 44.4737067) sent=True"
    errors = _gate(lines).errors
    assert any("first selected dock is 'd2'" in e for e in errors)
    assert any("d2 was never offered" in e for e in errors)


def test_missing_approval_request_fails():
    assert any("no approval request" in e for e in _gate([FLEET, ORBIT, SELECT_D1, OFFER_D2]).errors)


def test_missing_orbit_fails():
    assert any("no self-detect orbit" in e for e in _gate([FLEET, SELECT_D1, CONFIRMING, OFFER_D2]).errors)


def test_orbit_abandoned_before_selection_fails():
    lines = [FLEET, ORBIT, T + "REJECTED: resuming AUTO mission", SELECT_D1, CONFIRMING, OFFER_D2]
    assert any("no self-detect orbit" in e for e in _gate(lines).errors)


def test_orbit_superseded_before_selection_fails():
    lines = [FLEET, ORBIT, T + "GUIDED_LOITER cmd=DO_REPOSITION center=40.3,44.4,260.0 radius=500m",
             SELECT_D1, CONFIRMING, OFFER_D2]
    assert any("no self-detect orbit" in e for e in _gate(lines).errors)


def test_offer_between_orbit_command_and_selection_fails():
    lines = [FLEET, ORBIT, OFFER_D2, SELECT_D1, CONFIRMING]
    assert any("first offered before" in e for e in _gate(lines).errors)


def test_no_offer_fails():
    assert any("d2 was never offered" in e for e in _gate([FLEET, ORBIT, SELECT_D1, CONFIRMING]).errors)


def test_offer_in_the_orbit_before_the_approval_request_passes():
    """Run e2b-1's order: orbit command, selection, offer, then CONFIRMING."""
    result = _gate([FLEET, ORBIT, SELECT_D1, OFFER_D2, CONFIRMING, DWELL_END])
    assert result.passed, result.errors


def test_offer_before_the_orbit_fails_even_with_a_later_offer_line():
    lines = [FLEET, OFFER_D2, ORBIT, SELECT_D1, CONFIRMING,
             T + "Rebroadcast task 2, waiting for 1 peers", OFFER_D2]
    errors = _gate(lines).errors
    assert any("first offered before" in e for e in errors)


def test_approval_request_after_the_orbit_ended_fails():
    lines = [FLEET, ORBIT, SELECT_D1, OFFER_D2, T + "REJECTED: resuming AUTO mission", CONFIRMING]
    assert any("no approval request" in e for e in _gate(lines).errors)


def test_offer_after_window_end_fails():
    lines = [FLEET, ORBIT, SELECT_D1, CONFIRMING, DWELL_END, OFFER_D2]
    assert any("first offered after" in e for e in _gate(lines).errors)


def test_unsent_offer_fails():
    lines = _passing()
    lines[4] = OFFER_D2.replace("sent=True", "sent=False")
    assert any("was not sent" in e for e in _gate(lines).errors)


def test_offer_of_another_dock_is_a_diagnostic_not_an_error():
    lines = _passing()
    lines.insert(4, T + "Task offered: task_id=3 obj_id=2 at (40.3165000, 44.4737000) sent=True")
    result = _gate(lines)
    assert result.passed, result.errors
    assert len(result.diagnostics) == 1


@pytest.mark.parametrize("answer", [
    "POI 1 confirmed by ground station.",
    "[Global] Assign request: task 2 -> peer 2",
    "Sent task assignment response for task 2 to 1",
])
def test_any_answer_fails(answer):
    assert any("was answered" in e for e in _gate(_passing() + [T + answer]).errors)


def test_missing_or_mismatched_fleet_docks_line_fails():
    assert any("no 'Fleet docks:'" in e for e in _gate(_passing()[1:]).errors)
    lines = _passing()
    lines[0] = T + "Fleet docks: P1=d2; P2=d1; P3=home-east; P4=home-middle; P5=home-west"
    assert any("differ from the dock file" in e for e in _gate(lines).errors)


def _fixture():
    return json.loads(runner.FIXTURE_PATH.read_text(encoding="utf-8"))


def test_slice_scenario_is_the_offerer_zone_alone():
    fixture = _fixture()
    scenario = runner.slice_scenario(fixture, runner.FIXTURE_PATH)
    zone = fixture["expected"]["offerer_zone_index"]
    assert [slot.role for slot in scenario.slots] == ["owner"]
    assert scenario.missions == (fixture["missions"][zone],)
    assert scenario.poi_nav_waypoint_ordinals == ()
    assert aas_params_for(scenario, 1, 1)["targ_wps"] == 0
    home = fixture["inputs"]["home"]
    assert scenario.home == f"{home['lat']},{home['lon']},{home['alt_m']},0.0"


def test_fixture_runtime_matches_the_applied_launch_settings():
    fixture = _fixture()
    runner.require_fixture_runtime(fixture, runner.slice_scenario(fixture, runner.FIXTURE_PATH))
    fixture["runtime"]["search_zoom"] = 2
    with pytest.raises(RegressionError):
        runner.require_fixture_runtime(fixture, runner.slice_scenario(fixture, runner.FIXTURE_PATH))


def test_dock_file_is_the_fixture_world(tmp_path):
    fixture = _fixture()
    path, names = runner.write_dock_file(fixture, tmp_path)
    assert json.loads(path.read_text(encoding="utf-8")) == fixture["docks"]
    assert names == DOCKS
