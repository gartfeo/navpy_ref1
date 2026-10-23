"""tools/unreal/lens_fov_check.py: the pinhole fit per row and the FOV comparison with the reference knots."""
from __future__ import annotations

import json

import numpy as np
import pytest

from navpy.core.perception.camera.reading_intrinsics import ABSOLUTE_ZOOM, ZoomSetting
from tests.tools.unreal.test_cadence_lens import STREAM, TABLE, row_after
from tools.unreal import lens_fov_check as fc


def views(row, count: int = 4, seed: int = 3):
    """Normalized corner coordinates over several views and their pixels through the row's K."""
    rng = np.random.default_rng(seed)
    points = [np.column_stack([rng.uniform(-0.2, 0.2, 40), rng.uniform(-0.1, 0.1, 40)]) for _ in range(count)]
    pixels = [np.column_stack([row.fx * p[:, 0] + row.skew * p[:, 1] + row.cx, row.fy * p[:, 1] + row.cy])
              for p in points]
    return points, pixels


def test_the_fit_recovers_k_with_skew():
    rng = np.random.default_rng(1)
    xy = np.column_stack([rng.uniform(-0.3, 0.3, 80), rng.uniform(-0.2, 0.2, 80)])
    uv = np.column_stack([2000.0 * xy[:, 0] + 25.0 * xy[:, 1] + 970.0, 1990.0 * xy[:, 1] + 545.0])
    assert fc.fit_pinhole(xy, uv) == pytest.approx((2000.0, 1990.0, 970.0, 545.0, 25.0))


def test_fov_is_measured_on_both_sides_of_the_centre():
    h, v = fc.fov_deg(1000.0, 1000.0, 960.0, 540.0, 1920, 1080)
    assert h == pytest.approx(2 * np.degrees(np.arctan(0.96))) and v == pytest.approx(2 * np.degrees(np.arctan(0.54)))


def test_only_integer_commands_and_the_wide_stop_have_a_knot():
    knots = {"1.0": (50.0, 29.0), "1.1": (49.0, 28.0)}
    assert fc.knot_for(row_after(10), knots) == (49.0, 28.0)                     # command 1 settles at knot 1.1
    assert fc.knot_for(TABLE.optics(STREAM, 1.1, 1.1, ZoomSetting(ABSOLUTE_ZOOM, 12)), knots) is None
    wide = TABLE.optics(STREAM, 1.0, 1.0, ZoomSetting("wide_stop"))
    assert fc.knot_for(wide, knots) == (50.0, 29.0)


@pytest.mark.parametrize("off_pct, passes", [(0.0, True), (1.9, True), (2.1, False), (-2.1, False)])
def test_rows_pass_within_two_percent_of_their_knot(off_pct, passes):
    rows = [row_after(10), row_after(20)]
    groups = {}
    knots = {}
    for row in rows:
        groups[(row.reading, f"absolute_zoom:{row.setting.command_tenths / 10:.1f}")] = (row, *views(row))
        h, v = fc.fov_deg(row.fx, row.fy, row.cx, row.cy, STREAM.image_width, STREAM.image_height)
        knots[row.reading] = (h * (1 + off_pct / 100), v)
    results = fc._row_results(groups, STREAM, knots)
    assert all(r["vs_row_pct"] == [0.0, 0.0] for r in results)                   # the fit is the rendered row
    verdict = fc._verdict(results, lens_pass=True)
    assert verdict["scored_rows"] == 2 and verdict["pass"] is passes


def test_failed_lens_labels_or_a_row_without_a_knot_fail():
    rows = [row_after(10), row_after(20)]
    groups = {(row.reading, f"absolute_zoom:{row.setting.command_tenths / 10:.1f}"): (row, *views(row)) for row in rows}
    knots = {row.reading: fc.fov_deg(row.fx, row.fy, row.cx, row.cy, STREAM.image_width, STREAM.image_height)
             for row in rows}
    results = fc._row_results(groups, STREAM, knots)
    assert fc._verdict(results, lens_pass=True)["pass"]
    assert not fc._verdict(results, lens_pass=False)["pass"]
    partial = fc._row_results(groups, STREAM, {rows[0].reading: knots[rows[0].reading]})
    verdict = fc._verdict(partial, lens_pass=True)
    assert not verdict["pass"] and verdict["unreferenced_rows"] == [f"{rows[1].reading}/absolute_zoom:2.0"]


def test_a_row_with_too_few_corners_fails():
    row = row_after(20)
    points, pixels = views(row, count=1)
    results = fc._row_results({(row.reading, "absolute_zoom:2.0"): (row, [points[0][:10]], [pixels[0][:10]])},
                              STREAM, {row.reading: (30.0, 17.0)})
    assert "fit" not in results[0] and not fc._verdict(results, lens_pass=True)["pass"]


def test_reference_knots_by_reading(tmp_path):
    path = tmp_path / "reference.json"
    path.write_text(json.dumps({"optics": {"zoom_knots": {"rows": [
        {"reading": 1.0, "hfov_deg_pinhole": 50.6, "vfov_deg_pinhole": 29.8},
        {"reading": 10.0, "hfov_deg_pinhole": 5.6, "vfov_deg_pinhole": 3.2}]}}}), encoding="utf-8")
    assert fc.reference_knots(path) == {"1.0": (50.6, 29.8), "10.0": (5.6, 3.2)}
