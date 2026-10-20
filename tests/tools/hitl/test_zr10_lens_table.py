"""ZR10 lens-table builder (tools/hitl/zr10_lens_table.py) on synthetic data; the real inputs stay private."""
from __future__ import annotations

import json
import math

import numpy as np
import pytest

from navpy.core.perception.camera.reading_intrinsics import (INTERPOLATED, MEASURED, ReadingIntrinsicsTable,
                                                            StreamMode, ZoomSetting)
from tools.hitl import zr10_board_sets as boards
from tools.hitl import zr10_lens_profile as lp
from tools.hitl import zr10_lens_table as lt

MANIFEST = lt.Manifest(0.98, 1.03, ("run",), {3.0: "s3", 4.0: "s4"}, frozenset({4.0}), 2.0)
GRID = np.array([[x * 40.0, y * 40.0] for y in range(6) for x in range(9)])


def _view(command: float, reading: float, scale: float, reading_after: float | None = None) -> dict:
    corners = (GRID - GRID.mean(axis=0)) * scale + [960.0, 540.0]
    return {"command": command, "reading": reading, "reading_after": reading if reading_after is None else reading_after,
            "frames": [{"corners": corners.tolist()}, {"corners": None}]}


def test_sweep_scales_chain_runs_through_a_shared_setting_and_drop_travelling_views():
    run1 = [_view(1.0, 1.1, 1.0), _view(2.0, 1.8, 1.6), _view(3.0, 2.8, 2.4, reading_after=2.9)]
    run2 = [_view(2.0, 1.8, 0.8), _view(4.0, 3.8, 1.6)]     # same board at half the image scale
    scales = lt.sweep_scales([run1, run2])
    assert set(scales) == {(11, 10), (18, 20), (38, 40)}
    np.testing.assert_allclose(scales[(18, 20)], [1.6, 1.6], rtol=1e-6)
    np.testing.assert_allclose(scales[(38, 40)], [3.2], rtol=1e-6)


def test_sweep_scales_refuse_a_run_with_no_shared_setting():
    with pytest.raises(ValueError, match="shares no"):
        lt.sweep_scales([[_view(1.0, 1.1, 1.0)], [_view(4.0, 3.8, 2.0)]])


SHAPE_A = {"fx": 1000.0, "fy": 998.0, "cx": 950.0, "cy": 530.0, "k1": -0.2, "k2": 0.1}
SHAPE_B = {"fx": 4000.0, "fy": 3980.0, "cx": 990.0, "cy": 550.0, "k1": 0.4, "k2": -3.2}


def test_lens_at_a_measured_point_is_that_measurement():
    lens = lt.lens_at(1000.0, [lt._shape(SHAPE_A, 1000.0), lt._shape(SHAPE_B, 4000.0)])
    for key, value in SHAPE_A.items():
        assert lens[key] == pytest.approx(value)


def test_lens_between_points_is_interpolated_in_log_focal_length_and_pixel_distortion():
    lens = lt.lens_at(2000.0, [lt._shape(SHAPE_A, 1000.0), lt._shape(SHAPE_B, 4000.0)])
    assert lens["cx"] == pytest.approx(970.0)
    assert lens["fy"] / lens["fx"] == pytest.approx((0.998 + 0.995) / 2)
    assert lens["k1"] / 2000.0 ** 2 == pytest.approx((-0.2 / 1000.0 ** 2 + 0.4 / 4000.0 ** 2) / 2)
    beyond = lt.lens_at(8000.0, [lt._shape(SHAPE_A, 1000.0), lt._shape(SHAPE_B, 4000.0)])
    assert beyond["cx"] == pytest.approx(990.0)


def test_a_measurement_sits_at_the_tables_focal_length_not_its_own():
    """A fit whose focal length reads high still anchors its own setting's row exactly."""
    lens = lt.lens_at(980.0, [lt._shape(SHAPE_A, 980.0), lt._shape(SHAPE_B, 4000.0)])
    assert (lens["cx"], lens["cy"]) == pytest.approx((950.0, 530.0))
    assert lens["k1"] == pytest.approx(-0.2 * 0.98 ** 2)    # same distortion in pixels


def test_table_rows_mark_measured_settings_and_derive_the_wide_stop():
    scales = {(11, 10): 1.0, (18, 20): 1.6, (38, 40): 3.2}
    found = {(18, 20): {"lens": SHAPE_A}, (38, 40): {"lens": SHAPE_B}}
    rows = lt.table_rows(scales, found, 1000.0, MANIFEST)
    assert [(r["reading"], r["set_by"], r["basis"]) for r in rows] == [
        (1.0, "wide_stop", INTERPOLATED), (1.1, {"absolute_zoom": 1.0}, INTERPOLATED),
        (1.8, {"absolute_zoom": 2.0}, MEASURED), (3.8, {"absolute_zoom": 4.0}, MEASURED)]
    assert rows[0]["fx"] == pytest.approx(0.98 * 1000.0 / 1.03, abs=1e-3)
    assert rows[2]["fx"] == pytest.approx(0.98 * 1600.0, abs=1e-3)
    assert rows[2]["cx"] == SHAPE_A["cx"]


def test_score_is_zero_for_views_made_with_the_rows_lens():
    import cv2
    row = {"fx": 2000.0, "fy": 1998.0, "cx": 960.0, "cy": 540.0, "dist": [-0.1, 0.05, 0.0, 0.0, 0.0]}
    k = np.array([[2000.0, 0, 960.0], [0, 1998.0, 540.0], [0, 0, 1]])
    obj = boards.board((9, 6))
    imgs = []
    for angle in (0.2, -0.3):
        p, _ = cv2.projectPoints(obj, np.array([angle, 0.1, 0.0]), np.array([-4.0, -2.5, 30.0]), k,
                                 np.asarray(row["dist"]))
        imgs.append(p.astype(np.float32))
    assert boards.score(row, [obj, obj], imgs)["max_px"] < 1e-3


def _profile_text(newline: str) -> str:
    profile = {"profiles": {"siyi_zr10": {"devices": [{"name": "siyi_zr10", "camera": {
        "zoom_calibration": {"1.0": 1.1}, "reading_intrinsics": {"source": "old", "streams": []}, "after": [1, 2]}}]},
        "other": {"devices": []}}}
    return json.dumps(profile, indent=2).replace("\n", newline)


@pytest.mark.parametrize("newline", ["\n", "\r\n"])
def test_write_profile_replaces_only_the_section_one_row_per_line(tmp_path, newline):
    path = tmp_path / "profile.json"
    path.write_bytes(_profile_text(newline).encode())
    rows = lt.table_rows({(11, 10): 1.0, (18, 20): 1.6}, {(18, 20): {"lens": SHAPE_A}}, 1000.0, MANIFEST)
    lp.write_profile(path, lp.section(rows, (1920, 1080)))
    raw = path.read_bytes().decode()
    assert raw.count("\r\n") == (raw.count("\n") if newline == "\r\n" else 0)
    camera = json.loads(raw)["profiles"]["siyi_zr10"]["devices"][0]["camera"]
    assert camera["after"] == [1, 2] and camera["zoom_calibration"] == {"1.0": 1.1}
    assert camera["reading_intrinsics"]["streams"][0]["rows"] == rows
    assert sum('{"reading": ' in line for line in raw.splitlines()) == len(rows)
    table = ReadingIntrinsicsTable.from_profile(camera["reading_intrinsics"])
    stream = StreamMode("main", 1920, 1080, (0, 0, 1920, 1080))
    assert table.optics(stream, 1.8, 1.8, ZoomSetting.absolute_zoom(2.0)).basis == MEASURED
    assert not math.isnan(table.optics(stream, 1.0, 1.0, ZoomSetting.wide_stop()).fx)


TRUE_FX = {1.0: 1000.0, 2.0: 1600.0, 3.0: 2400.0, 4.0: 3200.0}     # absolute zoom command -> true focal length
READING = {1.0: 1.1, 2.0: 1.8, 3.0: 2.8, 4.0: 3.8}


def _lens_views(fx: float) -> list:
    import cv2
    k = np.array([[fx, 0, 965.0], [0, fx * 0.999, 545.0], [0, 0, 1]])
    obj = boards.board((9, 6))
    views = []
    for rx, ry in ((0.35, 0.0), (-0.35, 0.1), (0.0, 0.4), (0.1, -0.4), (0.25, 0.25)):
        p, _ = cv2.projectPoints(obj, np.array([rx, ry, 0.05]), np.array([-4.0, -2.5, fx / 40.0]), k,
                                 np.array([-0.05, 0.02, 0.0, 0.0, 0.0]))
        views.append({"corners": p.reshape(-1, 2).tolist()})
    return views


def _write(path, data) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data), encoding="utf-8")


def test_build_goes_from_inputs_to_rows(tmp_path):
    """Sweep scale x the constant from the saved fits x the laser factor; a refit set recovers its true focal
    length, and the wide stop divides the absolute 1.0 row by the wide ratio."""
    inputs = lt.Inputs(tmp_path / "sweep", tmp_path / "sessions", tmp_path / "anchors", tmp_path / "inputs.json")
    _write(inputs.sweep / "run" / "sweep.json",
           {"records": [_view(c, READING[c], TRUE_FX[c] / 1000.0) for c in TRUE_FX]})
    saved_fx = {3.0: 2400.0 * 1.02, 4.0: 5000.0}     # board-fit bias at 3; a bad (local-minimum) fit at 4
    for command, sid in MANIFEST.tilted.items():
        _write(inputs.sessions / sid / "session.json",
               {"pattern": [9, 6], "zoom_reading": READING[command], "absolute_zoom_request": {"requested": command},
                "views": _lens_views(TRUE_FX[command])})
        _write(inputs.sessions / sid / "candidate-x.json",
               {"fx": saved_fx[command], "fy": saved_fx[command], "cx": 965.0, "cy": 545.0, "k1": -0.05, "k2": 0.02})
    _write(inputs.anchors / "charuco_zoom2_base_model.json",
           {"provenance": {"absolute_zoom_request": {"requested": 2.0, "reported": 1.8}}, "width": 1920,
            "height": 1080, "fx": 1630.0, "fy": 1628.0, "cx": 962.0, "cy": 541.0, "k1": -0.1, "k2": 0.05})
    _write(inputs.anchors / "wall_solution_v3.json",
           {"matrix": {"f": 980.0, "aspect": 0.999, "cx": 958.0, "cy": 539.0, "k1": -0.2, "k2": 0.1}})
    _write(inputs.manifest,
           {"laser_factor": 0.98, "wide_in": 1.03, "sweep_runs": ["run"], "tilted": {"3.0": "s3", "4.0": "s4"},
            "refit_from_sweep": [4.0], "charuco_command": 2.0})

    rows, report = lt.build(inputs)

    constant = (2400.0 * 1.02 / 2.4 + 3200.0 / 3.2) / 2     # saved fit at 3; the refit recovers the true 3200 at 4
    assert report["sweep_constant"] == pytest.approx(constant, rel=1e-4)
    by_key = {(r["reading"], str(r["set_by"])): r for r in rows}
    for command, fx in TRUE_FX.items():
        row = by_key[(READING[command], str({"absolute_zoom": command}))]
        assert row["fx"] == pytest.approx(0.98 * constant * fx / 1000.0, rel=1e-4)
    wide = by_key[(1.0, "wide_stop")]
    assert wide["fx"] == pytest.approx(0.98 * constant / 1.03, rel=1e-4)
    assert [r["basis"] for r in rows].count(MEASURED) == 4    # wall, ChArUco and both tilted sets
    assert by_key[(1.1, str({"absolute_zoom": 1.0}))]["basis"] == INTERPOLATED
    refit = by_key[(3.8, str({"absolute_zoom": 4.0}))]
    assert (refit["cx"], refit["cy"]) == pytest.approx((965.0, 545.0), abs=0.05)
    assert {s["session"] for s in report["sessions"]} == {"s3", "s4"}
    assert all(s["result"]["p95_px"] < 0.5 for s in report["sessions"])
