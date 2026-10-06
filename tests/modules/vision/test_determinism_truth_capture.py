"""Original truth packets must survive the production callback boundary."""

import json
from types import SimpleNamespace

import pytest

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.vision.sim import determinism_trace
from tests.modules.vision.test_direct_poi_pixel_source import _source, _vehicle
from tests.modules.vision.truth_packet_factory import truth_packet
from tests.modules.vision.determinism_case_factory import (
    eligible_capture, evidence_bytes, with_rows,
)
from navpy.modules.vision.sim.determinism_truth_sample import (
    CODEC, UNAVAILABLE, capture_truth, frame_sample, valid_record,
)
from navpy.modules.vision.sim.determinism_reader import read_evidence
from navpy.modules.vision.sim.determinism_trace_decode import EvidenceRefused
from navpy.modules.vision.sim.determinism_eligibility import judge


def test_distinct_original_samples_do_not_collapse_to_one_watermark(monkeypatch):
    monkeypatch.setattr(determinism_trace, "ENABLED", True)
    vehicle = _vehicle(truth_attitude=Attitude(0, 0, 0),
                       telemetry_attitude=Attitude(0, 0, 0))
    source = _source(vehicle, [])
    for stamp, wire in ((12345, b"first"), (23456, b"second")):
        source._on_message(SimpleNamespace(
            get_type=lambda: "SIM_STATE", time_us=stamp,
            get_msgbuf=lambda wire=wire: bytearray(wire),
            get_srcSystem=lambda: 1, get_srcComponent=lambda: 1,
        ))
    rows = [r for r in source.determinism_trace.capture().rows if r[0] == "truth"]
    assert len(rows) == 2
    assert rows[0] != rows[1], "distinct raw SIM_STATE packets were erased"


@pytest.mark.parametrize("stamp", [1, 255, 256, 2**53 + 1, 2**64 - 1])
@pytest.mark.parametrize("signed", [False, True])
def test_real_packet_preserves_integer_stamp_original_bytes_and_identity(stamp, signed):
    message = truth_packet(stamp, signed=signed)
    original = bytes(message.get_msgbuf())
    record = capture_truth(message)
    assert record == (CODEC, "captured", stamp, 17, 1, original)
    assert frame_sample(original)[:3] == (17, 1, stamp)
    assert valid_record(record)
    message.get_msgbuf()[:] = b"changed after callback"
    message.time_us = 7
    assert record[-1] == original
    assert record[2] == stamp


def test_transport_sequence_does_not_change_semantic_payload():
    a, b = (capture_truth(truth_packet(sequence=seq)) for seq in (1, 201))
    assert a[-1] != b[-1]
    assert frame_sample(a[-1])[-1] == frame_sample(b[-1])[-1]


@pytest.mark.parametrize("value,expected", [
    (0, "stamp_unavailable"), (None, "stamp_unavailable"),
    (True, "stamp_unavailable"), (1.0, "stamp_unavailable"),
    (-1, "invalid_stamp"), (2**64, "invalid_stamp"),
    (999, "stamp_mismatch"),
])
def test_bad_decoded_stamp_is_retained_as_unusable(value, expected):
    message = truth_packet()
    message.time_us = value
    record = capture_truth(message)
    assert record[1] == expected
    assert record[2] == (value if type(value) is int else None)
    assert valid_record(record)


def test_zero_extension_is_unavailable_not_replaced_with_receipt_time():
    message = truth_packet(0)
    assert len(message.get_msgbuf()) < 112  # MAVLink 2 trailing zero truncation
    assert capture_truth(message)[1] == "stamp_unavailable"
    assert frame_sample(bytes(message.get_msgbuf()))[2] == 0


@pytest.mark.parametrize("change", [
    lambda wire: wire[:-1], lambda wire: wire + wire,
    lambda wire: wire[:12] + bytes([wire[12] ^ 1]) + wire[13:],
    lambda wire: wire[:2] + b"\x02" + wire[3:],
])
def test_corrupt_frames_remain_observable_but_unusable(change):
    message = truth_packet()
    wire = change(bytes(message.get_msgbuf()))
    message._msgbuf = bytearray(wire)
    record = capture_truth(message)
    assert record[1] == "invalid_frame"
    assert record[-1] == wire
    assert valid_record(record)


def test_nonfinite_payload_preserves_original_bytes_and_marks_invalid():
    message = truth_packet(value=float("nan"))
    assert capture_truth(message)[1] == "invalid_frame"
    assert capture_truth(message)[-1] == bytes(message.get_msgbuf())


def test_metadata_disagreement_cannot_claim_captured():
    message = truth_packet()
    message._header.srcSystem = 19
    record = capture_truth(message)
    assert record[1] == "identity_mismatch"
    assert not valid_record((CODEC, "captured", *record[2:]))


def test_v2_round_trip_and_v1_compatibility_are_explicit():
    capture = eligible_capture()
    manifest, trace = evidence_bytes(capture)
    modern = read_evidence(manifest, trace)
    assert modern.capture == capture
    assert judge(modern).eligible
    legacy = with_rows(capture, (r[:5] if r[0] == "truth" else r for r in capture.rows))
    manifest, trace = evidence_bytes(legacy)
    with pytest.raises(EvidenceRefused, match="evidence version"):
        read_evidence(manifest, trace)
    decoded = json.loads(manifest)
    decoded["version"] = 1
    case = read_evidence(json.dumps(decoded).encode(), trace)
    assert case.capture == legacy
    assert judge(case).eligible  # original v1 meaning; no full-input claim
    modern_manifest, modern_trace = evidence_bytes(capture)
    decoded = json.loads(modern_manifest)
    decoded["version"] = 1
    with pytest.raises(EvidenceRefused, match="evidence version"):
        read_evidence(json.dumps(decoded).encode(), modern_trace)


@pytest.mark.parametrize("replacement", [
    lambda r: (*r[:2], r[2] + 1, *r[3:]),
    lambda r: (*r[:3], 200, *r[4:]),
    lambda r: ("unknown-codec", *r[1:]),
    lambda r: (*r[:-1], r[-1][:-1]),
    lambda r: (*r[:2], True, *r[3:]),
])
def test_reader_refuses_raw_fields_that_contradict_claimed_capture(replacement):
    original = eligible_capture()
    altered = with_rows(original, (
        (*r[:5], replacement(r[5])) if r[0] == "truth" else r
        for r in original.rows
    ))
    with pytest.raises(EvidenceRefused):
        read_evidence(*evidence_bytes(altered))


def test_unavailable_sample_is_inspectable_not_eligible():
    original = eligible_capture()
    altered = with_rows(original, (
        (*r[:5], UNAVAILABLE) if r[0] == "truth" else r for r in original.rows
    ))
    verdict = judge(read_evidence(*evidence_bytes(altered)))
    assert verdict.failed_rules == (14,)


def test_overflow_and_seal_keep_event_and_raw_observation_atomic():
    trace = determinism_trace.DeterminismTrace(20000, capacity=1)
    trace.record_truth(epoch=1, outcome="recorded", message=truth_packet(10))
    trace.record_truth(epoch=1, outcome="recorded", message=truth_packet(20))
    capture = trace.capture()
    assert len(capture.rows) == 1
    assert capture.rows[0][5][2] == 10
    assert capture.status.dropped == 1
    trace.journal.seal()
    trace.record_truth(epoch=1, outcome="recorded", message=truth_packet(30))
    assert trace.capture().rows == capture.rows
    assert trace.capture().status.refused == 1


@pytest.mark.parametrize("failure", [ValueError, KeyboardInterrupt])
def test_codec_failures_use_existing_atomic_fault_boundary(failure):
    def broken():
        raise failure("injected packet getter failure")
    trace = determinism_trace.DeterminismTrace(20000)
    escaped = None
    try:
        trace.record_truth(epoch=1, outcome="recorded",
                           message=SimpleNamespace(get_msgbuf=broken))
    except BaseException as error:
        escaped = error
    if failure is KeyboardInterrupt:
        assert isinstance(escaped, KeyboardInterrupt)
    else:
        assert escaped is None
    assert not trace.capture().status.complete
    assert trace.capture().rows == ()


def test_tracing_off_does_not_read_packet_fields(monkeypatch):
    monkeypatch.setattr(determinism_trace, "ENABLED", False)
    vehicle = _vehicle(truth_attitude=Attitude(0, 0, 0),
                       telemetry_attitude=Attitude(0, 0, 0))
    source = _source(vehicle, [])
    def unreadable():
        pytest.fail("disabled trace read a raw packet")
    source._on_message(SimpleNamespace(get_type=lambda: "SIM_STATE", get_msgbuf=unreadable))
    assert source.determinism_trace is None
