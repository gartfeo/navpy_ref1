"""tools/unreal/lens_fixtures.py: the ray fixture the Unreal lens test (NavPy.Lens.FixtureRays) checks against."""
from __future__ import annotations

import json

import numpy as np
import pytest

from navpy.core.perception.camera.reading_intrinsics import ReadingIntrinsicsTable
from tools.unreal import lens_fixtures as lf


def test_every_row_and_the_synthetic_skews_have_rays_on_their_pixels():
    fixture = lf.build()
    section = fixture["reading_intrinsics"]
    ReadingIntrinsicsTable.from_profile(section)
    rows = [row for stream in section["streams"] for row in stream["rows"]]
    assert len(fixture["rows"]) == len(rows) > 0
    assert {s["name"] for s in fixture["synthetic"]} == {"skew_positive", "skew_negative", "off_centre_small"}
    per_case = len(lf.GRID) ** 2 * len(lf.DEPTHS_M)
    for case, row in zip(fixture["rows"], rows):
        assert (case["reading"], case["set_by"]) == (row["reading"], row["set_by"])
        rays, pixels = np.array(case["rays_cv"]), np.array(case["pixels"])
        assert rays.shape == (per_case, 3) and pixels.shape == (per_case, 2)
        u = row["fx"] * rays[:, 0] / rays[:, 2] + row["skew"] * rays[:, 1] / rays[:, 2] + row["cx"]
        assert np.allclose(pixels[:, 0], u, atol=1e-6)


def test_skew_moves_u_by_skew_times_y_over_z():
    rays = np.array([[0.0, 0.5, 1.0]])
    assert lf.pinhole_pixels(1000.0, 1000.0, 960.0, 540.0, 40.0, rays)[0] == pytest.approx([980.0, 1040.0])


def test_check_passes_the_written_fixture_and_catches_a_changed_row(tmp_path):
    out = tmp_path / "lens_fixtures.json"
    assert lf.main(["write", "--out", str(out)]) == 0
    assert lf.check(out) == []
    fixture = json.loads(out.read_text(encoding="utf-8"))
    fixture["rows"][3]["pixels"][0][0] += 0.2
    out.write_text(lf.fixture_text(fixture), encoding="utf-8")
    assert lf.check(out) == ["rows differs"]
    assert lf.main(["check", "--fixture", str(out)]) == 1


def test_check_catches_a_profile_changed_after_the_fixture(tmp_path):
    out = tmp_path / "lens_fixtures.json"
    lf.main(["write", "--out", str(out)])
    profile = json.loads(lf.PROFILE.read_text(encoding="utf-8"))
    device = next(d for d in profile["profiles"][lf.PROFILE_NAME]["devices"] if d["name"] == lf.DEVICE_NAME)
    device["camera"]["reading_intrinsics"]["streams"][0]["rows"][0]["fx"] += 1.0
    changed = tmp_path / "vision_profiles.json"
    changed.write_text(json.dumps(profile), encoding="utf-8")
    assert set(lf.check(out, changed)) == {"reading_intrinsics differs", "rows differs"}
