"""The live camera check reports the EXPOSURE stamping rate over the span where stamping was possible."""
from __future__ import annotations

from tools.unreal.npyc_live_check import summarize


def _row(index: int, kind: str) -> dict:
    return {"frame_index": index, "stamp_kind": kind,
            "latency_ms": "100.0" if kind == "exposure" else "", "detected": ""}


def test_frames_before_and_after_the_flight_do_not_count_against_the_stamping_rate():
    # A check that outlives the flight: before ATTITUDE/poses and after the bridge stops, frames are READ by rule.
    rows = ([_row(i, "read") for i in range(3)]
            + [_row(3 + i, "exposure") for i in range(5)]
            + [_row(8, "read")]                                  # a missed stamp inside the span counts
            + [_row(9, "exposure")]
            + [_row(10 + i, "read") for i in range(10)])
    summary = summarize(rows, seconds=1.0)
    assert summary["exposure_stamped"] == 6
    assert summary["exposure_span"] == {"frames": 7, "exposure": 6}


def test_without_any_exposure_frame_the_span_is_empty():
    summary = summarize([_row(0, "read"), _row(1, "read")], seconds=1.0)
    assert summary["exposure_span"] == {"frames": 0, "exposure": 0}
