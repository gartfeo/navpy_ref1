"""Reading-keyed optics (HITL plan section 9, stage 1 step 3).

Intrinsics are looked up by stream mode, settled zoom reading and the zoom setting that produced it, never by zoom
command and never between rows: an unknown, travelling or off-grid reading, a row measured after another setting and
an unmeasured stream mode are refused. Lookup tests use synthetic tables; the profile tests pin the selected ZR10 rows
and check that the command table stays a separate thing.
"""

from __future__ import annotations

import copy
import json
import math
from pathlib import Path

import numpy as np
import pytest

from navpy.core.perception.camera.reading_intrinsics import (
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
SETTINGS = {"1.0": WIDE, "1.8": ABS_2, "10.0": ABS_10}


def _row(fx: float, set_by: object = "wide_stop") -> dict:
    return {"set_by": set_by, "fx": fx, "fy": fx * 0.999, "cx": 321.5, "cy": 179.25, "skew": 0.25,
            "dist": [-0.2, 0.1, 0.001, -0.002, 0.03]}


def _rows() -> dict:
    return {"1.0": _row(500.0), "1.8": _row(800.0, {"absolute_zoom": 2.0}),
            "10.0": _row(4100.0, {"absolute_zoom": 10})}


def _stream(rows: dict, mode: str = "main", size: tuple = (640, 360)) -> dict:
    return {"mode": mode, "image_width": size[0], "image_height": size[1], "crop": [0, 0, *size], "readings": rows}


def _section(rows: dict | None = None) -> dict:
    return {"source": "synthetic", "streams": [_stream(rows or _rows())]}


@pytest.fixture
def table() -> ReadingIntrinsicsTable:
    return ReadingIntrinsicsTable.from_profile(_section())


def test_every_row_resolves_to_its_own_values(table):
    for reading, row in _rows().items():
        optics = table.optics(STREAM, float(reading), float(reading), SETTINGS[reading])
        assert optics is not None, reading
        assert (optics.reading, optics.setting) == (reading, SETTINGS[reading])
        np.testing.assert_array_equal(optics.k(), [[row["fx"], row["skew"], row["cx"]], [0.0, row["fy"], row["cy"]],
                                                  [0.0, 0.0, 1.0]])
        np.testing.assert_array_equal(optics.dist_coeffs(), row["dist"])


def test_readings_list_the_supported_rows_in_order(table):
    assert table.readings(STREAM) == ("1.0", "1.8", "10.0")
    assert table.streams() == (STREAM,)


def test_row_labels_are_normalized_readings():
    table = ReadingIntrinsicsTable.from_profile(_section({"1.80": _row(800.0), "10": _row(4100.0)}))
    assert table.readings(STREAM) == ("1.8", "10.0")
    assert table.optics(STREAM, 1.8, 1.8, WIDE).reading == "1.8"


def test_each_stream_mode_selects_its_own_rows():
    sub = StreamMode("sub", 1280, 720, (0, 0, 1280, 720))
    section = {"streams": [_stream(_rows()), _stream({"1.8": _row(1600.0, {"absolute_zoom": 2.0})}, "sub",
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
def test_a_row_measured_after_another_setting_is_refused(table, reading, setting):
    """One reading can cover several lens states; only the measured one is admitted."""
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
    change(section, section["streams"][0], section["streams"][0]["readings"]["1.8"])
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
    (lambda s, st, r: st.update(readings={}), "must be a non-empty mapping"),
    (lambda s, st, r: st["readings"].update({"1.80": _row(801.0)}), "repeats reading 1.8"),
    (lambda s, st, r: st["readings"].update({"1.85": _row(801.0)}), "is not a reading >= 0.1 on the 0.1 grid"),
    (lambda s, st, r: st["readings"].update({"zoom": _row(801.0)}), "is not a reading >= 0.1 on the 0.1 grid"),
    (lambda s, st, r: st["readings"].update({"0.0": _row(801.0)}), "is not a reading >= 0.1 on the 0.1 grid"),
    (lambda s, st, r: st["readings"].update({"1e-8": _row(801.0)}), "is not a reading >= 0.1 on the 0.1 grid"),
    (lambda s, st, r: st["readings"].update({"1e400": _row(801.0)}), "is not a reading >= 0.1 on the 0.1 grid"),
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
        stream["readings"]["1.0"]["dist"] = []

    with pytest.raises(ValueError) as error:
        ReadingIntrinsicsTable.from_profile(_broken(two_faults))
    assert "readings.1.8 fx and fy" in str(error.value)
    assert "readings.1.0 dist" in str(error.value)


# The ZR10 rows selected for step 3: the only readings with a measured centre and distortion on the main stream
# (sources in the profile's reading_intrinsics.source). Every other reading stays disabled.
ZR10_STREAM = StreamMode("main", 1920, 1080, (0, 0, 1920, 1080))
ZR10_SELECTED = {
    "1.0": (WIDE, (2028.919, 2027.113, 987.347, 548.66), (-0.275204, 0.204547, 0.0, 0.0, 0.0)),
    "1.8": (ABS_2, (3280.098, 3270.088, 991.156, 548.917), (-0.134795, 0.527685, 0.0, 0.0, 0.0)),
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


def test_the_zr10_table_holds_exactly_the_selected_rows(zr10_table):
    assert zr10_table.streams() == (ZR10_STREAM,)
    assert zr10_table.readings(ZR10_STREAM) == tuple(ZR10_SELECTED)


@pytest.mark.parametrize("reading", sorted(ZR10_SELECTED))
def test_every_selected_zr10_row_is_reachable_with_its_measured_values(zr10_table, reading):
    setting, (fx, fy, cx, cy), dist = ZR10_SELECTED[reading]
    optics = zr10_table.optics(ZR10_STREAM, float(reading), float(reading), setting)
    assert optics is not None
    assert (optics.fx, optics.fy, optics.cx, optics.cy, optics.skew) == (fx, fy, cx, cy, 0.0)
    assert optics.dist == dist


def test_a_zoom_command_is_not_a_reading(zr10_camera, zr10_table):
    """Command 2.0 settles on reading 1.8 (zoom_calibration); keying by the command would pick the wrong lens."""
    commands = {float(command): reading for command, reading in zr10_camera["zoom_calibration"].items()}
    assert commands[2.0] == 1.8
    assert zr10_table.optics(ZR10_STREAM, 2.0, 2.0, ABS_2) is None
    assert zr10_table.optics(ZR10_STREAM, commands[2.0], commands[2.0], ABS_2).reading == "1.8"
    assert commands[1.0] == 1.1     # absolute zoom 1.0 does not reach the measured wide stop
    assert zr10_table.optics(ZR10_STREAM, commands[1.0], commands[1.0], ZoomSetting.absolute_zoom(1.0)) is None
