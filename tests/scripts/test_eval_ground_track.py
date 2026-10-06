from __future__ import annotations

import csv
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts.eval_ground_track import GroundTrackRecorder
from scripts.eval_navigation_models import PoiLocation


POI = PoiLocation(
    lat_deg=40.0, lon_deg=44.0, rel_alt_m=60.0, abs_alt_m=1000.0
)


def _message(
    *,
    vn: float,
    ve: float,
    lat: float = 40.0,
    lon: float = 44.0,
    alt_m: float = 1000.0,
    boot_ms: int = 1000,
) -> SimpleNamespace:
    return SimpleNamespace(
        vx=vn * 100.0,
        vy=ve * 100.0,
        vz=0.0,
        lat=int(lat * 1e7),
        lon=int(lon * 1e7),
        alt=int(alt_m * 1000.0),
        time_boot_ms=boot_ms,
    )


def _feed(recorder: GroundTrackRecorder, count: int, **kwargs: float) -> None:
    for _ in range(count):
        recorder.add(_message(**kwargs))


def test_track_is_the_measured_direction_of_motion() -> None:
    recorder = GroundTrackRecorder(POI)
    # Due east at 20 m/s.
    _feed(recorder, 10, vn=0.0, ve=20.0)

    summary = recorder.summary(0.0, 0.0)

    assert summary["track_deg"] == pytest.approx(90.0)
    assert summary["ground_speed_m_s"] == pytest.approx(20.0)


def test_headwind_is_positive_and_tailwind_negative() -> None:
    """ArduPilot wind_dir is the direction the wind blows FROM."""
    recorder = GroundTrackRecorder(POI)
    _feed(recorder, 10, vn=20.0, ve=0.0)  # tracking due north

    head = recorder.summary(10.0, 0.0)  # wind from the north = head-on
    tail = recorder.summary(10.0, 180.0)  # wind from the south = from behind

    assert head["headwind_m_s"] == pytest.approx(10.0)
    assert head["crosswind_m_s"] == pytest.approx(0.0, abs=1e-9)
    assert tail["headwind_m_s"] == pytest.approx(-10.0)


def test_crosswind_is_positive_from_the_right() -> None:
    recorder = GroundTrackRecorder(POI)
    _feed(recorder, 10, vn=20.0, ve=0.0)  # tracking due north

    summary = recorder.summary(10.0, 90.0)  # wind from the east

    assert summary["crosswind_m_s"] == pytest.approx(10.0)
    assert summary["headwind_m_s"] == pytest.approx(0.0, abs=1e-9)


def test_same_world_wind_is_head_for_one_track_and_tail_for_another() -> None:
    """The reason this exists: peers approach from any bearing."""
    north = GroundTrackRecorder(POI)
    _feed(north, 10, vn=20.0, ve=0.0)
    south = GroundTrackRecorder(POI)
    _feed(south, 10, vn=-20.0, ve=0.0)

    wind_from_north = (10.0, 0.0)

    assert north.summary(*wind_from_north)["headwind_m_s"] == pytest.approx(10.0)
    assert south.summary(*wind_from_north)["headwind_m_s"] == pytest.approx(-10.0)


def test_track_averages_the_vector_not_the_heading_across_the_wrap() -> None:
    """Scalar-averaging headings either side of north gives 180 deg wrong."""
    recorder = GroundTrackRecorder(POI)
    # Alternating 350 deg and 10 deg: the true mean track is due north.
    _feed(recorder, 5, vn=19.70, ve=-3.47)
    _feed(recorder, 5, vn=19.70, ve=3.47)

    track = recorder.summary(0.0, 0.0)["track_deg"]

    assert track == pytest.approx(0.0, abs=0.5) or track == pytest.approx(
        360.0, abs=0.5
    )


def test_final_approach_window_prefers_samples_near_the_poi() -> None:
    recorder = GroundTrackRecorder(POI)
    # Far away heading east, then committed to a northward final-approach run.
    _feed(recorder, 20, vn=0.0, ve=20.0, lat=40.05)
    _feed(recorder, 5, vn=20.0, ve=0.0, lat=40.0005)

    summary = recorder.summary(0.0, 0.0)

    assert summary["used_final_approach_window"] is True
    assert summary["track_sample_count"] == 5
    assert summary["track_deg"] == pytest.approx(0.0, abs=0.5)


def test_a_level_fly_by_is_scored_on_the_run_in_not_the_departure() -> None:
    """The window is a range band, and a level pass re-enters it outbound.

    Measured on a clean 0.08 m approach flown due north: samples averaging
    18.10 m/s of ground speed produced a 1.80 m/s resultant pointing 284 deg,
    because the departure was averaged in with the approach. The wind was then
    resolved against a track the aircraft never flew and a pure crosswind cell
    was reported as a headwind.
    """
    recorder = GroundTrackRecorder(POI)
    for lat in (39.9985, 39.9990, 39.9995, 40.0000):
        recorder.add(_message(vn=20.0, ve=0.0, lat=lat))
    # Turns back and leaves, for longer than the run-in lasted: averaging the
    # departure in would report a track of roughly 180, due south.
    for lat in (39.9995, 39.9990, 39.9985, 39.9990, 39.9995):
        recorder.add(_message(vn=-20.0, ve=0.0, lat=lat))

    summary = recorder.summary(10.0, 0.0)

    assert summary["track_deg"] == pytest.approx(0.0, abs=0.5)
    assert summary["ground_speed_m_s"] == pytest.approx(20.0, abs=0.5)
    # A headwind for the run-in, which is the leg the navigation flew.
    assert summary["headwind_m_s"] == pytest.approx(10.0, abs=0.5)


def test_out_and_back_samples_do_not_define_a_track() -> None:
    """Every sample moves, but the resultant cancels: refuse, don't invent."""
    recorder = GroundTrackRecorder(POI)
    _feed(recorder, 5, vn=20.0, ve=0.0)
    _feed(recorder, 5, vn=-20.0, ve=0.0)

    with pytest.raises(RuntimeError, match="cancel"):
        recorder.summary(10.0, 0.0)


def test_stationary_samples_do_not_define_a_track() -> None:
    recorder = GroundTrackRecorder(POI)
    _feed(recorder, 10, vn=0.0, ve=0.0)

    with pytest.raises(RuntimeError, match="no moving ground-track samples"):
        recorder.summary(5.0, 0.0)


def test_malformed_message_is_rejected_without_recording() -> None:
    recorder = GroundTrackRecorder(POI)

    assert recorder.add(SimpleNamespace()) is False
    assert recorder.sample_count == 0


def test_write_emits_the_velocity_vector_per_sample(tmp_path: Path) -> None:
    recorder = GroundTrackRecorder(POI)
    _feed(recorder, 3, vn=12.0, ve=-5.0)

    recorder.write(tmp_path / "ground_track.csv")

    with (tmp_path / "ground_track.csv").open(encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    assert len(rows) == 3
    assert float(rows[0]["vn_m_s"]) == pytest.approx(12.0)
    assert float(rows[0]["ve_m_s"]) == pytest.approx(-5.0)
