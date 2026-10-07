"""Internal naming changes preserve saved evaluation evidence exactly."""

from scripts.eval_navigation_models import SelectionEvidence
from scripts.eval_navigation_truth_samples import TruthRecord, read_track_records, write_track_csv


def test_selection_evidence_retains_recorded_field_order_and_values():
    evidence = SelectionEvidence(2, 4, 7, 9, True, 100.5, 43.0, 34.0, 500.0)
    expected = {
        "catalog_wp": 2,
        "catalog_seq": 4,
        "task_id": 7,
        "obj_id": 9,
        "default_ooi_registered": True,
        "event_wall_time_s": 100.5,
        "poi_lat_deg": 43.0,
        "poi_lon_deg": 34.0,
        "poi_abs_alt_m": 500.0,
    }
    assert list(evidence.to_record().items()) == list(expected.items())


def test_truth_track_preserves_bytes_and_reads_old_evidence(tmp_path):
    recorded = (
        b"received_wall_time_s,source_time_s,lat_deg,lon_deg,abs_alt_m,scoring_active,converted,reason\r\n"
        b"100.5,2.0,43.0,34.0,500.0,1,1,\r\n"
        b"101.0,,,,,0,0,missing coordinates\r\n"
    )
    path = tmp_path / "truth_track.csv"
    path.write_bytes(recorded)
    expected = [
        TruthRecord(100.5, 2.0, 43.0, 34.0, 500.0, True, True, None),
        TruthRecord(101.0, None, None, None, None, False, False, "missing coordinates"),
    ]
    assert read_track_records(path) == expected
    write_track_csv(expected, path)
    assert path.read_bytes() == recorded
