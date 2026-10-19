"""Reading-keyed optics (HITL plan section 9, stage 1 step 3).

Intrinsics are looked up by stream mode, settled zoom reading and the zoom setting that produced it, never by zoom
command and never between rows: an unknown, travelling or off-grid reading, a reading without a row for the given
setting and an unmeasured stream mode are refused. Lookup tests use synthetic tables; the profile tests pin the
measured ZR10 rows, check every row is reachable and that the command table stays a separate thing.
"""

from __future__ import annotations

import copy
import json
import math
from pathlib import Path

import numpy as np
import pytest

from navpy.core.perception.camera.reading_intrinsics import (
    INTERPOLATED,
    MEASURED,
    ReadingIntrinsicsTable,
    StreamMode,
    ZoomSetting,
    zoom_tenths,
)

PROFILE_PATH = Path(__file__).resolve().parents[3] / "src/navpy/core/perception/profile/vision_profiles.json"
STREAM = StreamMode("main", 640, 360, (0, 0, 640, 360))
WIDE = ZoomSetting.wide_stop()
ABS_2 = ZoomSetting.absolute_zoom(2.0)
ABS_10 = ZoomSetting.absolute_zoom(10.0)
ABS_1 = ZoomSetting.absolute_zoom(1.0)
SETTINGS = {"1.0": WIDE, "1.8": ABS_2, "10.0": ABS_10}


def _row(reading: object, fx: float, set_by: object = "wide_stop", basis: str = MEASURED) -> dict:
    return {"reading": reading, "set_by": set_by, "basis": basis, "fx": fx, "fy": fx * 0.999, "cx": 321.5,
            "cy": 179.25, "skew": 0.25, "dist": [-0.2, 0.1, 0.001, -0.002, 0.03]}


def _rows() -> list:
    return [_row(1.0, 500.0), _row(1.8, 800.0, {"absolute_zoom": 2.0}),
            _row(10.0, 4100.0, {"absolute_zoom": 10}, INTERPOLATED)]


def _stream(rows: list, mode: str = "main", size: tuple = (640, 360)) -> dict:
    return {"mode": mode, "image_width": size[0], "image_height": size[1], "crop": [0, 0, *size], "rows": rows}


def _section(rows: dict | None = None) -> dict:
    return {"source": "synthetic", "streams": [_stream(rows or _rows())]}


@pytest.fixture
def table() -> ReadingIntrinsicsTable:
    return ReadingIntrinsicsTable.from_profile(_section())


def test_every_row_resolves_to_its_own_values(table):
    for row in _rows():
        reading = f"{row['reading']:.1f}"
        optics = table.optics(STREAM, row["reading"], row["reading"], SETTINGS[reading])
        assert optics is not None, reading
        assert (optics.reading, optics.setting, optics.basis) == (reading, SETTINGS[reading], row["basis"])
        np.testing.assert_array_equal(optics.k(), [[row["fx"], row["skew"], row["cx"]], [0.0, row["fy"], row["cy"]],
                                                  [0.0, 0.0, 1.0]])
        np.testing.assert_array_equal(optics.dist_coeffs(), row["dist"])


def test_readings_list_the_supported_rows_in_order(table):
    assert table.readings(STREAM) == ("1.0", "1.8", "10.0")
    assert table.streams() == (STREAM,)


def test_row_labels_are_normalized_readings():
    table = ReadingIntrinsicsTable.from_profile(_section([_row(1.80, 800.0), _row(10, 4100.0)]))
    assert table.readings(STREAM) == ("1.8", "10.0")
    assert table.optics(STREAM, 1.8, 1.8, WIDE).reading == "1.8"


def test_one_reading_can_hold_a_row_per_setting():
    """Reading 1.1 after absolute zoom 1.0 is another lens state than 1.1 after other settings."""
    table = ReadingIntrinsicsTable.from_profile(_section([_row(1.1, 600.0, {"absolute_zoom": 1.0}),
                                                          _row(1.1, 620.0, {"absolute_zoom": 1.2})]))
    abs_1_2 = ZoomSetting.absolute_zoom(1.2)
    assert table.readings(STREAM) == ("1.1",)
    assert table.settings(STREAM, 1.1) == (ABS_1, abs_1_2)
    assert table.optics(STREAM, 1.1, 1.1, ABS_1).fx == 600.0
    assert table.optics(STREAM, 1.1, 1.1, abs_1_2).fx == 620.0
    assert table.optics(STREAM, 1.1, 1.1, WIDE) is None
    assert table.settings(STREAM, 1.2) == ()


def test_each_stream_mode_selects_its_own_rows():
    sub = StreamMode("sub", 1280, 720, (0, 0, 1280, 720))
    section = {"streams": [_stream(_rows()), _stream([_row(1.8, 1600.0, {"absolute_zoom": 2.0})], "sub",
                                                    (1280, 720))]}
    table = ReadingIntrinsicsTable.from_profile(section)
    assert table.streams() == (STREAM, sub)
    assert table.optics(STREAM, 1.8, 1.8, ABS_2).fx == 800.0
    assert table.optics(sub, 1.8, 1.8, ABS_2).fx == 1600.0
    assert table.optics(sub, 1.0, 1.0, WIDE) is None
    assert table.readings(sub) == ("1.8",)


def test_wire_readings_land_on_the_grid(table):
    """The SDK builds a reading as int_part + float_part / 10 (siyi_sdk.parseCurrentZoomLevelMsg)."""
    assert table.optics(STREAM, 1 + 8 / 10, 1 + 8 / 10, ABS_2).reading == "1.8"
    assert table.optics(STREAM, 10, 10.0, ABS_10).reading == "10.0"
    assert zoom_tenths(8 + 6 / 10) == 86


@pytest.mark.parametrize("reading", [1.5, 1.1, 0.5, 9.9, 10.1, 11.0, 30.0])
def test_unknown_readings_are_refused_without_interpolation(table, reading):
    for setting in SETTINGS.values():
        assert table.optics(STREAM, reading, reading, setting) is None


@pytest.mark.parametrize("reading", [1.85, 1.0001, 0.0, 0.04, 1e-8, -1.0, -0.1, math.nan, math.inf, 1e308, 10**400,
                                     None, True, "1.8", [1.8]])
def test_malformed_readings_are_refused(table, reading):
    assert zoom_tenths(reading) is None
    assert table.optics(STREAM, reading, reading, ABS_2) is None


@pytest.mark.parametrize("before, after", [(1.0, 1.8), (1.8, 1.0), (1.8, 10.0), (1.8, None), (None, 1.8)])
def test_travelling_readings_are_refused(table, before, after):
    """Unequal readings bracketing an exposure mean the lens moved during it."""
    assert table.optics(STREAM, before, after, ABS_2) is None


@pytest.mark.parametrize("reading, setting", [
    (1.0, ABS_2), (1.8, WIDE), (1.8, ZoomSetting.absolute_zoom(1.9)), (1.8, ABS_10), (10.0, ABS_2), (1.8, None),
    (1.8, "absolute_zoom"),
])
def test_a_row_for_another_setting_is_refused(table, reading, setting):
    """One reading can cover several lens states; only a row for the frame's own setting is admitted."""
    assert table.optics(STREAM, reading, reading, setting) is None


@pytest.mark.parametrize("command", [1.85, 0.0, -2.0, math.nan, 10**400, None, True, "2.0"])
def test_an_absolute_setting_needs_a_command_on_the_grid(command):
    with pytest.raises(ValueError, match="is not a number >= 0.1 on the 0.1 grid"):
        ZoomSetting.absolute_zoom(command)


@pytest.mark.parametrize("stream", [
    StreamMode("sub", 640, 360, (0, 0, 640, 360)),
    StreamMode("main", 1280, 720, (0, 0, 640, 360)),
    StreamMode("main", 640, 360, (0, 0, 1280, 720)),
    StreamMode("main", 640, 360, (8, 0, 640, 360)),
])
def test_unmeasured_stream_modes_are_refused(table, stream):
    assert table.optics(stream, 1.8, 1.8, ABS_2) is None
    assert table.readings(stream) == ()


def _broken(change) -> dict:
    section = copy.deepcopy(_section())
    change(section, section["streams"][0], section["streams"][0]["rows"][1])
    return section


@pytest.mark.parametrize("change, message", [
    (lambda s, st, r: s.pop("streams"), "section.streams is missing"),
    (lambda s, st, r: s.update(extra=1), "section.extra is not a known field"),
    (lambda s, st, r: s.update(streams=[]), "streams must be a non-empty list"),
    (lambda s, st, r: s["streams"].append(copy.deepcopy(st)), "repeats stream mode"),
    (lambda s, st, r: st.update(mode=""), "mode must be a non-empty string"),
    (lambda s, st, r: st.update(image_width=640.0), "must be positive integers"),
    (lambda s, st, r: st.update(image_height=True), "must be positive integers"),
    (lambda s, st, r: st.update(crop=[0, 0, 640]), "crop must be"),
    (lambda s, st, r: st.update(crop=[-1, 0, 640, 360]), "crop must be"),
    (lambda s, st, r: st.pop("crop"), "crop is missing"),
    (lambda s, st, r: st.update(rows=[]), "rows must be a non-empty list"),
    (lambda s, st, r: st.update(rows={"1.8": r}), "rows must be a non-empty list"),
    (lambda s, st, r: st.pop("rows"), "rows is missing"),
    (lambda s, st, r: st.update(readings=st.pop("rows")), "readings is not a known field"),
    (lambda s, st, r: st["rows"].append(_row(1.80, 801.0, {"absolute_zoom": 2.0})),
     r"rows\[3\] repeats reading 1.8 after the same setting"),
    (lambda s, st, r: r.update(reading=1.85), "reading must be a number >= 0.1 on the 0.1 grid"),
    (lambda s, st, r: r.update(reading="1.8"), "reading must be a number >= 0.1 on the 0.1 grid"),
    (lambda s, st, r: r.update(reading=0.0), "reading must be a number >= 0.1 on the 0.1 grid"),
    (lambda s, st, r: r.update(reading=1e-8), "reading must be a number >= 0.1 on the 0.1 grid"),
    (lambda s, st, r: r.update(reading=10**400), "reading must be a number >= 0.1 on the 0.1 grid"),
    (lambda s, st, r: r.update(reading=True), "reading must be a number >= 0.1 on the 0.1 grid"),
    (lambda s, st, r: r.pop("reading"), "reading is missing"),
    (lambda s, st, r: r.pop("basis"), "basis is missing"),
    (lambda s, st, r: r.update(basis="fitted"), "basis must be one of measured, interpolated"),
    (lambda s, st, r: r.update(basis=None), "basis must be one of measured, interpolated"),
    (lambda s, st, r: r.pop("set_by"), "set_by is missing"),
    (lambda s, st, r: r.update(set_by="manual"), "set_by must be"),
    (lambda s, st, r: r.update(set_by={"absolute_zoom": 1.85}), "set_by must be"),
    (lambda s, st, r: r.update(set_by={"absolute_zoom": True}), "set_by must be"),
    (lambda s, st, r: r.update(set_by={"absolute_zoom": 2.0, "settle_s": 6}), "set_by must be"),
    (lambda s, st, r: r.update(fx=0.0), "fx and fy must be positive"),
    (lambda s, st, r: r.update(fy=math.nan), "fx and fy must be positive"),
    (lambda s, st, r: r.update(fx=10**400), "fx and fy must be positive"),
    (lambda s, st, r: r.update(cx=641.0), "cx and cy must lie in the 640x360 frame"),
    (lambda s, st, r: r.update(cy=-0.5), "cx and cy must lie in the 640x360 frame"),
    (lambda s, st, r: r.update(skew=None), "skew must be a finite number"),
    (lambda s, st, r: r.update(dist=[-0.2, 0.1, 0.0, 0.0]), "dist must be five finite numbers"),
    (lambda s, st, r: r.update(dist=[-0.2, 0.1, 0.0, 0.0, math.inf]), "dist must be five finite numbers"),
    (lambda s, st, r: r.update(dist=[-0.2, 0.1, 0.0, 0.0, 10**400]), "dist must be five finite numbers"),
    (lambda s, st, r: r.pop("dist"), "dist is missing"),
    (lambda s, st, r: r.update(k3=0.0), "k3 is not a known field"),
])
def test_malformed_tables_are_refused(change, message):
    with pytest.raises(ValueError, match=message):
        ReadingIntrinsicsTable.from_profile(_broken(change))


def test_a_refused_table_lists_every_problem():
    def two_faults(section, stream, row):
        row.update(fx=-1.0)
        stream["rows"][0]["dist"] = []

    with pytest.raises(ValueError) as error:
        ReadingIntrinsicsTable.from_profile(_broken(two_faults))
    assert "rows[1] fx and fy" in str(error.value)
    assert "rows[0] dist" in str(error.value)


# The measured ZR10 rows on the main stream (tools/hitl/zr10_lens_table.py; sources in its report): focal length
# from the still-board zoom sweep and the laser factor, centre and distortion measured at that setting. Every other
# (reading, absolute zoom) row has an interpolated centre and distortion.
ZR10_STREAM = StreamMode("main", 1920, 1080, (0, 0, 1920, 1080))
ZR10_MEASURED = {
    ("1.0", WIDE): ((2044.329, 2042.51, 987.347, 548.66), (-0.279401, 0.210833, 0.0, 0.0, 0.0)),
    ("1.8", ABS_2): ((3273.052, 3263.063, 991.156, 548.917), (-0.134216, 0.523165, 0.0, 0.0, 0.0)),
    ("2.8", ZoomSetting.absolute_zoom(3.0)): ((4912.759, 4894.129, 996.166, 569.352),
                                              (0.0238116, 0.63641, 0.0, 0.0, 0.0)),
    ("3.8", ZoomSetting.absolute_zoom(4.0)): ((6615.413, 6589.584, 991.108, 569.242),
                                              (0.234957, -2.28675, 0.0, 0.0, 0.0)),
    ("5.7", ZoomSetting.absolute_zoom(6.0)): ((9926.347, 9883.289, 963.306, 554.322),
                                              (0.883029, -16.8585, 0.0, 0.0, 0.0)),
    ("7.7", ZoomSetting.absolute_zoom(8.0)): ((13395.173, 13336.932, 1018.992, 544.115),
                                              (1.04812, 91.3083, 0.0, 0.0, 0.0)),
    ("8.6", ZoomSetting.absolute_zoom(9.0)): ((15142.431, 15066.045, 1004.276, 557.057),
                                              (2.24813, 62.8684, 0.0, 0.0, 0.0)),
    ("10.0", ABS_10): ((16970.564, 16812.132, 1017.668, 534.582), (2.39739, 395.477, 0.0, 0.0, 0.0)),
}


@pytest.fixture(scope="module")
def zr10_camera() -> dict:
    data = json.loads(PROFILE_PATH.read_text(encoding="utf-8"))
    device = data["profiles"]["siyi_zr10"]["devices"][0]
    assert device["name"] == "siyi_zr10"
    return device["camera"]


@pytest.fixture(scope="module")
def zr10_table(zr10_camera) -> ReadingIntrinsicsTable:
    return ReadingIntrinsicsTable.from_profile(zr10_camera["reading_intrinsics"])


def _zr10_rows(zr10_camera) -> list:
    return zr10_camera["reading_intrinsics"]["streams"][0]["rows"]


def _setting(set_by: object) -> ZoomSetting:
    return WIDE if set_by == "wide_stop" else ZoomSetting.absolute_zoom(set_by["absolute_zoom"])


def test_the_zr10_table_has_the_wide_stop_and_every_absolute_zoom_command(zr10_camera, zr10_table):
    """One row per command 1.0..10.0 at the reading it settles on (zoom_calibration), plus the wide stop."""
    assert zr10_table.streams() == (ZR10_STREAM,)
    assert len(_zr10_rows(zr10_camera)) == 92
    for command, reading in zr10_camera["zoom_calibration"].items():
        optics = zr10_table.optics(ZR10_STREAM, reading, reading, ZoomSetting.absolute_zoom(float(command)))
        assert optics is not None, command
    assert zr10_table.optics(ZR10_STREAM, 1.0, 1.0, WIDE) is not None


def test_every_zr10_row_is_reachable_with_its_values(zr10_camera, zr10_table):
    for row in _zr10_rows(zr10_camera):
        optics = zr10_table.optics(ZR10_STREAM, row["reading"], row["reading"], _setting(row["set_by"]))
        assert (optics.fx, optics.fy, optics.cx, optics.cy, optics.basis) == (
            row["fx"], row["fy"], row["cx"], row["cy"], row["basis"])


def test_only_the_measured_zr10_settings_say_measured(zr10_camera, zr10_table):
    measured = [r for r in _zr10_rows(zr10_camera) if r["basis"] == MEASURED]
    assert {(f"{r['reading']:.1f}", _setting(r["set_by"])) for r in measured} == set(ZR10_MEASURED)
    for (reading, setting), ((fx, fy, cx, cy), dist) in ZR10_MEASURED.items():
        optics = zr10_table.optics(ZR10_STREAM, float(reading), float(reading), setting)
        assert (optics.fx, optics.fy, optics.cx, optics.cy, optics.skew) == (fx, fy, cx, cy, 0.0)
        assert optics.dist == dist


@pytest.mark.parametrize("reading", [0.9, 10.1, 11.0, 30.0])
def test_zr10_readings_outside_the_zoom_range_are_refused(zr10_table, reading):
    for setting in (WIDE, ABS_1, ABS_10):
        assert zr10_table.optics(ZR10_STREAM, reading, reading, setting) is None


def test_a_zoom_command_is_not_a_reading(zr10_camera, zr10_table):
    """Command 2.0 settles on reading 1.8 (zoom_calibration); keying by the command would pick the wrong lens."""
    commands = {float(command): reading for command, reading in zr10_camera["zoom_calibration"].items()}
    assert commands[2.0] == 1.8
    assert zr10_table.optics(ZR10_STREAM, 2.0, 2.0, ABS_2) is None
    assert zr10_table.optics(ZR10_STREAM, commands[2.0], commands[2.0], ABS_2).reading == "1.8"
    assert commands[1.0] == 1.1     # absolute zoom 1.0 does not reach the wide stop: another lens state
    assert zr10_table.optics(ZR10_STREAM, commands[1.0], commands[1.0], WIDE) is None
    assert zr10_table.optics(ZR10_STREAM, 1.1, 1.1, ABS_1).fx > zr10_table.optics(ZR10_STREAM, 1.0, 1.0, WIDE).fx
