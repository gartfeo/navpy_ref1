"""`prepare_vehicles` must PUSH the parameters it is handed, per vehicle.

The caller-side wiring test in `test_eval_direct_pixel_pn.py` proves the
harness passes `sim_parameters(args)` in, but it never runs `prepare_vehicles`
itself -- it would stay green if this function ignored the argument. This file
exercises the mechanism with the MAVLink layer faked out.
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from scripts import eval_three_uav_mission as mission


@pytest.fixture()
def stubbed(monkeypatch):
    pushed: list[tuple[int | None, str, float]] = []
    state = SimpleNamespace(selected=None, pushed=pushed, echo=True)
    monkeypatch.setattr(
        mission, "request_coordinate_score_stream", lambda master: True
    )
    monkeypatch.setattr(mission, "download_mission", lambda master: "mission")
    monkeypatch.setattr(
        mission, "resolve_home_abs_alt_m", lambda master, timeout_s: 100.0
    )
    monkeypatch.setattr(
        mission,
        "resolve_poi_expectation",
        lambda *a, **k: SimpleNamespace(location="poi", mission_seq=4),
    )

    def fake_set_param(master, name, value):
        pushed.append((state.selected, name, value))
        return state.echo

    monkeypatch.setattr(mission, "set_param", fake_set_param)
    state.gated = []
    monkeypatch.setattr(
        mission,
        "require_nav_solution",
        lambda master, sys_id: state.gated.append(sys_id),
    )
    return state


def _select(state):
    def select(master, sys_id):
        state.selected = sys_id

    return select


def test_prepare_vehicles_pushes_exactly_the_given_parameters(stubbed) -> None:
    parameters = (("SIM_RATE_HZ", 1000.0), ("SIM_GPS_HZ", 10.0))

    pois, run_navigation_episode = mission.prepare_vehicles(
        object(),
        [121, 122],
        select=_select(stubbed),
        poi_wp=6,
        scoring_start_wp=4,
        poi_rel_alt_m=60.0,
        parameters=parameters,
    )

    assert stubbed.pushed == [
        (121, "SIM_RATE_HZ", 1000.0),
        (121, "SIM_GPS_HZ", 10.0),
        (122, "SIM_RATE_HZ", 1000.0),
        (122, "SIM_GPS_HZ", 10.0),
    ]
    assert set(pois) == set(run_navigation_episode) == {121, 122}


def test_a_failed_parameter_echo_stops_the_case(stubbed) -> None:
    """A parameter SITL did not acknowledge means the case would fly a
    different simulation than the artifacts record."""
    stubbed.echo = False

    with pytest.raises(RuntimeError, match="parameter echo failed"):
        mission.prepare_vehicles(
            object(),
            [121],
            select=_select(stubbed),
            poi_wp=6,
            scoring_start_wp=4,
            poi_rel_alt_m=60.0,
            parameters=(("SIM_RATE_HZ", 1000.0),),
        )


def test_prepare_vehicles_holds_each_vehicle_for_its_ekf_origin(stubbed) -> None:
    """The origin wait is absorbed here, before any timed child exists."""
    mission.prepare_vehicles(
        object(),
        [121, 122],
        select=_select(stubbed),
        poi_wp=6,
        scoring_start_wp=4,
        poi_rel_alt_m=60.0,
        parameters=(),
    )
    assert stubbed.gated == [121, 122]
