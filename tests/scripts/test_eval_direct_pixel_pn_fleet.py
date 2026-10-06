"""A fleet must not let launch order stand in for the wind cell.

Sysid tracks launch order, and launch order has carried real effects before:
the aircraft that needed a MISSION_START retry were exactly the lowest sysids
in their wave, and their scores were the worst in it.

A fixed round-robin looks like it handles that and does not. It gives cell `i`
positions `i, i+len, i+2*len ...` in every single run, so each cell keeps a
constant mean launch position and anything periodic in the fleet width aliases
onto cell index exactly. Randomising within each replicate keeps the matrix
balanced while removing the fixed offset, so those properties are pinned here.
"""

from __future__ import annotations

import pytest

from scripts.eval_fleet_cells import DEFAULT_CELLS, assign, parse_cells
from scripts.eval_fleet_report import miss_3d_m
from scripts.eval_fleet_span import check_span, chats_spanned, fleet_sysids

SEED = 20260821


def test_a_cell_reads_speed_and_the_direction_it_blows_from() -> None:
    assert parse_cells("8@90") == [(8.0, 90.0)]


def test_calm_needs_no_direction() -> None:
    """`0` alone is the calm cell; a direction on it would be meaningless."""
    assert parse_cells("0") == [(0.0, 0.0)]


def test_the_default_matrix_is_calm_plus_two_speeds_by_four_directions() -> None:
    cells = parse_cells(DEFAULT_CELLS)

    assert len(cells) == 9
    assert cells[0] == (0.0, 0.0)
    assert {direction for speed, direction in cells if speed} == {0.0, 90.0, 180.0, 270.0}


def test_an_empty_matrix_is_refused_rather_than_flown_as_calm() -> None:
    with pytest.raises(ValueError):
        parse_cells("  ")


def test_the_fleet_is_cells_times_repeats() -> None:
    assert len(assign(parse_cells(DEFAULT_CELLS), 4, seed=SEED)) == 36


def test_every_cell_gets_the_same_number_of_aircraft() -> None:
    """An unbalanced matrix would weight some winds more than others."""
    cells = parse_cells(DEFAULT_CELLS)
    winds = assign(cells, 4, seed=SEED)

    assert {winds.count(cell) for cell in cells} == {4}


def test_each_replicate_holds_every_cell_exactly_once() -> None:
    """What makes it a complete block: no replicate is missing a wind."""
    cells = parse_cells(DEFAULT_CELLS)
    winds = assign(cells, 4, seed=SEED)

    for start in range(0, len(winds), len(cells)):
        assert sorted(winds[start:start + len(cells)]) == sorted(cells)


def test_no_cell_keeps_the_fixed_launch_offset_a_round_robin_would_give_it() -> None:
    """The property a plain round-robin fails: a constant position per cell."""
    cells = parse_cells(DEFAULT_CELLS)
    winds = assign(cells, 4, seed=SEED)

    offsets = {
        cell: {position % len(cells)
               for position, wind in enumerate(winds) if wind == cell}
        for cell in cells
    }

    assert any(len(seen) > 1 for seen in offsets.values())


def test_the_same_seed_reproduces_the_same_matrix() -> None:
    """Randomised, not unrepeatable -- a run has to be re-flyable."""
    cells = parse_cells(DEFAULT_CELLS)

    assert assign(cells, 4, seed=SEED) == assign(cells, 4, seed=SEED)
    assert assign(cells, 4, seed=SEED) != assign(cells, 4, seed=SEED + 1)


def test_the_miss_is_read_from_the_field_the_verdict_actually_fills() -> None:
    """`closest` is null in the verdict; reading it emptied a scored table."""
    row = {"scoring_source": "ekf_snap_estimate", "coordinate": {"dist_3d_m": 0.09}}
    assert miss_3d_m(row) == pytest.approx(0.09)
    assert miss_3d_m({
        "scoring_source": "ekf_snap_estimate", "closest": {"dist_3d_m": 0.09}
    }) is None


def test_a_truth_scored_row_tables_the_truth_miss_not_the_ekf_projection() -> None:
    """The EKF's own error projection ranks ANTI to the real miss; tabling it
    for a truth-scored fleet would order the cells backwards."""
    row = {
        "scoring_source": "sim_state_truth",
        "truth": {"dist_3d_m": 0.05},
        "coordinate": {"dist_3d_m": 0.4},
    }
    assert miss_3d_m(row) == pytest.approx(0.05)


def test_a_truth_row_with_no_certified_distance_reports_none() -> None:
    """Fail visibly empty, never fall back to the estimate unlabeled."""
    row = {"scoring_source": "sim_state_truth", "coordinate": {"dist_3d_m": 0.4}}
    assert miss_3d_m(row) is None


def test_an_uncertified_truth_policy_row_never_falls_back_to_the_estimate() -> None:
    """The real failure shape: truth policy, certification failed, source
    `none`. Falling back to `coordinate` would feed the anti-ranked EKF
    projection into the medians precisely on the rows where truth failed."""
    row = {"scoring_source": "none", "coordinate": {"dist_3d_m": 1.88}}
    assert miss_3d_m(row) is None
    assert miss_3d_m({"coordinate": {"dist_3d_m": 1.88}}) is None


def test_the_cell_table_excludes_rows_that_do_not_stand_as_evidence() -> None:
    """Invalid and unscored rows must not enter the medians."""
    from scripts.eval_fleet_report import by_cell

    vehicles = {
        "121": {
            "valid": True, "scoring_source": "sim_state_truth",
            "truth": {"dist_3d_m": 0.05},
            "wind_speed_mps": 0.0, "wind_dir_deg": 0.0,
        },
        "122": {  # certified miss, but the run is invalid
            "valid": False, "scoring_source": "sim_state_truth",
            "truth": {"dist_3d_m": 0.07},
            "wind_speed_mps": 0.0, "wind_dir_deg": 0.0,
        },
        "123": {  # truth policy, certification failed
            "valid": False, "scoring_source": "none",
            "coordinate": {"dist_3d_m": 1.88},
            "wind_speed_mps": 0.0, "wind_dir_deg": 0.0,
        },
    }

    assert by_cell(vehicles) == {(0.0, 0.0): [0.05]}


def test_a_fleet_spans_the_chats_its_sysids_reach_into() -> None:
    """Three per chat is a GCS convention, so 36 aircraft touch twelve chats."""
    sys_ids = fleet_sysids(40, 36)

    assert sys_ids[0] == 121
    assert sys_ids[-1] == 156
    assert chats_spanned(sys_ids) == list(range(40, 52))


def test_a_fleet_that_would_address_ids_no_vehicle_can_hold_is_refused() -> None:
    """A uint8 sysid overflow otherwise surfaces as silent missing aircraft."""
    with pytest.raises(ValueError, match="MAX_SYSID"):
        check_span(80, fleet_sysids(80, 40))


def test_a_fleet_inside_the_limits_is_allowed() -> None:
    check_span(40, fleet_sysids(40, 36))
