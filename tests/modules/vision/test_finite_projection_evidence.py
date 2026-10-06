"""Actual delivery projection evidence, without additional camera sampling."""
from dataclasses import FrozenInstanceError
from unittest.mock import Mock

import numpy as np
import pytest
from pymap3d import ned2geodetic

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.common.models.location import Location
from navpy.modules.vision.models.detect_data import DetectStatus
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from navpy.modules.vision.sim.finite_poi_projector import FinitePoiProjector
from navpy.modules.vision.sim.sim_camera_ports import FrameSize, ProjectionCameraPort
from navpy.modules.vision.simulation_object import SimulationObject


def rig(sink, *, valid=False, pixels=(1293, 355), error_sink=None):
    location = Location(40, 44, 100, is_absolute=True)
    lat, lng, alt = ned2geodetic(100, 0, 0, location.lat, location.lng, location.alt)
    poi = SimulationObject(7, Location(lat, lng, alt), 2)
    matrix = np.array([[1407.42, 3, 670.95], [2, 1401.91, 369.78], [.1, .2, 1.]])
    gimbal = GimbalData(att=Attitude(0, -20, 0),
                        reference_aircraft_attitude=Attitude(1, 20, 2))
    camera = ProjectionCameraPort(Mock(return_value=matrix), Mock(return_value=gimbal),
                                  FrameSize(1280, 720), Mock(return_value=valid))
    calc = Mock(return_value=pixels)
    clock = Mock(return_value=50.)
    projector = FinitePoiProjector(camera, calc, lambda: (poi,), clock,
                                     evidence_sink=sink, evidence_error_sink=error_sink)
    return projector, camera, calc, clock, location, poi, matrix, gimbal


def test_rejected_projection_keeps_actual_rebased_inputs_and_frame_identity():
    records = []
    projector, camera, calc, clock, loc, poi, matrix, raw = rig(records.append)
    result = projector.detect(loc, poi, Attitude(0, 40, 0), timestamp_s=849.764)
    assert result.status == DetectStatus.OutOfView and result.poi is None
    record, = records
    assert record.source.poi_uid == 7 and record.source.source_timestamp_s == 849.764
    assert record.geometry.pixel_uv == (1293., 355.) and record.outcome.pixel_valid is False
    assert record.outcome.name == 'out_of_view' and record.outcome.complete
    assert record.geometry.matrix_values == tuple(matrix.flat)
    assert record.geometry.matrix_shape == (3, 3)
    assert record.geometry.raw_gimbal.attitude == (0., -20., 0.)
    assert record.geometry.raw_gimbal.reference_aircraft_attitude == (1., 20., 2.)
    effective = calc.call_args.args[2]
    assert record.geometry.effective_gimbal.attitude == (effective.att.pitch, effective.att.yaw, effective.att.roll)
    assert record.geometry.effective_gimbal != record.geometry.raw_gimbal
    camera.read_matrix.assert_called_once_with()
    camera.read_gimbal.assert_called_once_with()
    camera.pixel_valid.assert_called_once_with(1293., 355.)
    calc.assert_called_once()
    clock.assert_not_called()
    matrix[:] = 0
    raw.att.yaw = 999
    assert record.geometry.matrix_values[0] == 1407.42 and record.geometry.raw_gimbal.attitude[1] == -20.
    with pytest.raises(FrozenInstanceError):
        record.outcome.name = 'detected'


@pytest.mark.parametrize('pixels, valid, outcome', [
    ((640, 360), True, 'detected'), ((1293, 355), False, 'out_of_view'),
    ((None, None), False, 'no_projection'),
])
def test_observation_preserves_production_result_and_read_counts(pixels, valid, outcome):
    records = []
    args = (Attitude(0, 40, 0),)
    observed = rig(records.append, valid=valid, pixels=pixels)
    baseline = rig(None, valid=valid, pixels=pixels)
    results = []
    for projector, camera, calc, clock, loc, poi, *_ in (baseline, observed):
        results.append(projector.detect(loc, poi, *args, timestamp_s=12.5))
        camera.read_matrix.assert_called_once()
        camera.read_gimbal.assert_called_once()
        calc.assert_called_once()
    assert results[0].status == results[1].status
    assert records[0].outcome.name == outcome
    assert observed[1].pixel_valid.call_count == baseline[1].pixel_valid.call_count
    if results[0].poi is not None:
        assert results[0].poi.pixel == results[1].poi.pixel


def test_sink_failure_does_not_change_detection_and_is_visible():
    sink = Mock(side_effect=RuntimeError('writer unavailable'))
    projector, _, _, _, loc, poi, *_ = rig(sink, valid=True, pixels=(640, 360))
    result = projector.detect(loc, poi, Attitude(0, 0, 0), timestamp_s=12.5)
    assert result.status == DetectStatus.DETECTED
    assert projector.evidence_failures == 1


def test_original_projection_exception_is_preserved_and_recorded():
    records = []
    projector, camera, calc, _, loc, poi, *_ = rig(records.append)
    error = RuntimeError('original projection failure')
    calc.side_effect = error
    with pytest.raises(RuntimeError) as caught:
        projector.detect(loc, poi, Attitude(0, 40, 0), timestamp_s=12.5)
    assert caught.value is error
    record, = records
    assert record.outcome.name == 'exception' and record.outcome.exception_type == 'RuntimeError'
    camera.pixel_valid.assert_not_called()


def test_zero_distance_records_early_exit_without_fabricated_pixels():
    records = []
    projector, camera, calc, _, loc, poi, *_ = rig(records.append)
    poi.g_loc = loc
    result = projector.detect(loc, poi, Attitude(0, 0, 0), timestamp_s=12.5)
    assert result.status == DetectStatus.OutOfView
    record, = records
    assert record.outcome.name == 'zero_distance' and record.geometry.pixel_uv is None
    assert record.outcome.pixel_valid is None
    calc.assert_not_called()
    camera.pixel_valid.assert_not_called()


def test_log_sink_is_explicitly_opt_in_and_binds_process_vehicle_and_sequence(monkeypatch):
    import json
    import os
    from navpy.modules.vision.sim.finite_projection_evidence import projection_log_sink
    debug = Mock()
    monkeypatch.delenv('NAVPY_SIM_PROJECTION_EVIDENCE', raising=False)
    assert projection_log_sink(debug, 3) is None
    debug.assert_not_called()
    monkeypatch.setenv('NAVPY_SIM_PROJECTION_EVIDENCE', '1')
    sink = projection_log_sink(debug, 3)
    projector, _, _, _, loc, poi, *_ = rig(sink)
    projector.detect(loc, poi, Attitude(0, 40, 0), timestamp_s=12.5)
    projector.detect(loc, poi, Attitude(0, 40, 0), timestamp_s=12.5)
    rows = [json.loads(call.args[0].removeprefix('SIM_PROJECTION_EVIDENCE '))
            for call in debug.call_args_list]
    assert [row['projection_sequence'] for row in rows] == [1, 2]
    assert all(row['vehicle_sysid'] == 3 and row['pid'] == os.getpid() for row in rows)
    assert rows[0]['projection']['geometry']['pixel_uv'] == [1293, 355]


def test_too_far_records_the_actual_size_range_exit():
    records = []
    projector, camera, calc, _, loc, poi, matrix, raw = rig(records.append)
    matrix[1, 1] = 0
    raw.max_detect_distance = 1
    result = projector.detect(loc, poi, Attitude(0, 0, 0), timestamp_s=12.5)
    assert result.status == DetectStatus.OutOfView
    assert records[0].outcome.name == 'too_far' and records[0].geometry.pixel_uv is None
    calc.assert_not_called()
    camera.pixel_valid.assert_not_called()


def test_missing_frame_timestamp_is_incomplete_without_adding_clock_reads():
    records = []
    projector, _, _, clock, loc, poi, *_ = rig(records.append, pixels=(None, None))
    result = projector.detect(loc, poi, Attitude(0, 40, 0))
    assert result.status == DetectStatus.OutOfView
    record, = records
    assert record.source.source_timestamp_s is None and record.outcome.snapshot_complete
    assert not record.outcome.complete and not record.outcome.source_identity_complete
    assert projector.evidence_failures == 0
    clock.assert_not_called()


def test_projector_clock_does_not_certify_a_missing_frame_identity():
    records = []
    projector, _, _, clock, loc, poi, *_ = rig(records.append, valid=True, pixels=(640, 360))
    result = projector.detect(loc, poi, Attitude(0, 40, 0))
    assert result.status == DetectStatus.DETECTED
    record, = records
    assert record.source.source_timestamp_s == 50 and not record.source.frame_timestamp_provided
    assert not record.outcome.complete and not record.outcome.source_identity_complete
    clock.assert_called_once_with()


def test_world_lock_diagnostic_disagrees_but_actual_projection_evidence_is_correct():
    from navpy.args.uas_args import UasArgs
    from navpy.modules.navigation.geo.geo_ref_calc import GeoRefCalc
    records = []
    projector, camera, calc, _, loc, poi, matrix, raw = rig(records.append)
    matrix[:] = [[1407.42, 0, 670.95], [0, 1401.91, 369.78], [0, 0, 1]]
    raw.reference_aircraft_attitude = Attitude(0, 20, 0)
    calc.side_effect = GeoRefCalc(UasArgs()).calc_uv
    camera.pixel_valid.side_effect = lambda u, v: 0 <= u <= 1280 and 0 <= v <= 720
    attitude = Attitude(0, 60, 0)
    result = projector.detect(loc, poi, attitude, timestamp_s=12.5)
    assert result.status == DetectStatus.DETECTED
    assert records[0].geometry.pixel_uv == (671, 370) and records[0].outcome.pixel_valid is True
    camera.read_matrix.assert_called_once()
    camera.read_gimbal.assert_called_once()
    calc.assert_called_once()
    # The existing later diagnostic uses unre-based readback. This is a
    # synthetic evidence mismatch, not an attribution of the historical miss.
    reason = projector.diagnose(poi.uid, loc, attitude, matrix, raw)
    assert reason.startswith('out_of_fov')
    assert records[0].geometry.pixel_uv == (671, 370)


def test_pixel_validator_exception_keeps_original_identity():
    records = []
    projector, camera, _, _, loc, poi, *_ = rig(records.append)
    error = RuntimeError('validator failure')
    camera.pixel_valid.side_effect = error
    with pytest.raises(RuntimeError) as caught:
        projector.detect(loc, poi, Attitude(0, 40, 0), timestamp_s=12.5)
    assert caught.value is error
    assert records[0].outcome.name == 'exception' and records[0].geometry.pixel_uv == (1293, 355)


def test_concurrent_projections_keep_separate_inputs_and_results():
    from concurrent.futures import ThreadPoolExecutor
    import threading
    records = []
    projector, _, calc, _, loc, poi, *_ = rig(records.append)
    barrier = threading.Barrier(2)
    def calculate(*args):
        barrier.wait(timeout=2)
        return (1293, 355)
    calc.side_effect = calculate
    second = SimulationObject(8, poi.g_loc, poi.height)
    with ThreadPoolExecutor(max_workers=2) as executor:
        jobs = [executor.submit(projector.detect, loc, spot, Attitude(0, 40, 0),
                                timestamp_s=timestamp)
                for spot, timestamp in ((poi, 1.), (second, 2.))]
        assert all(job.result().status == DetectStatus.OutOfView for job in jobs)
    assert {(record.source.poi_uid, record.source.source_timestamp_s) for record in records} == {(7, 1.), (8, 2.)}
    assert all(record.outcome.complete for record in records)


def test_snapshot_failure_marks_record_incomplete_without_changing_detection(monkeypatch):
    from navpy.modules.vision.sim.finite_projection_evidence import GimbalSnapshot
    records = []
    projector, _, _, _, loc, poi, *_ = rig(records.append, valid=True, pixels=(640, 360))
    monkeypatch.setattr(GimbalSnapshot, 'capture', Mock(side_effect=ValueError('snapshot unavailable')))
    result = projector.detect(loc, poi, Attitude(0, 40, 0), timestamp_s=12.5)
    assert result.status == DetectStatus.DETECTED
    assert not records[0].outcome.complete and not records[0].outcome.snapshot_complete
    assert records[0].outcome.source_identity_complete
    assert projector.evidence_failures == 0


@pytest.mark.parametrize('timestamp', [float('nan'), float('inf'), True])
def test_invalid_frame_timestamp_does_not_certify_identity(timestamp):
    records = []
    projector, _, _, _, loc, poi, *_ = rig(records.append, valid=True, pixels=(640, 360))
    assert projector.detect(loc, poi, Attitude(0, 40, 0), timestamp_s=timestamp).status == DetectStatus.DETECTED
    assert not records[0].outcome.complete and not records[0].outcome.source_identity_complete


def test_real_json_sink_surfaces_nonfinite_serialization_failure(monkeypatch):
    from navpy.modules.vision.sim.finite_projection_evidence import projection_log_sink
    monkeypatch.setenv('NAVPY_SIM_PROJECTION_EVIDENCE', '1')
    debug, warnings = Mock(), Mock()
    projector, _, _, _, loc, poi, *_ = rig(projection_log_sink(debug, 3),
                                             valid=True, pixels=(640, 360), error_sink=warnings)
    assert projector.detect(loc, poi, Attitude(0, 40, 0), timestamp_s=float('nan')).status == DetectStatus.DETECTED
    debug.assert_not_called()
    assert projector.evidence_failures == 1
    assert warnings.call_count == 1
    assert all('SIM_PROJECTION_EVIDENCE_ERROR' in call.args[0] for call in warnings.call_args_list)


def test_numpy_timestamps_are_copied_as_json_numbers(monkeypatch):
    import json
    from navpy.modules.vision.sim.finite_projection_evidence import projection_log_sink
    monkeypatch.setenv('NAVPY_SIM_PROJECTION_EVIDENCE', '1')
    debug = Mock()
    projector, _, _, _, loc, poi, _, raw = rig(projection_log_sink(debug, 3),
                                                valid=True, pixels=(640, 360))
    raw.timestamp_s = np.int64(10)
    assert projector.detect(loc, poi, Attitude(0, 40, 0), timestamp_s=np.int64(12)).status == DetectStatus.DETECTED
    row = json.loads(debug.call_args.args[0].removeprefix('SIM_PROJECTION_EVIDENCE '))
    assert row['projection']['source']['source_timestamp_s'] == 12
    assert row['projection']['geometry']['raw_gimbal']['timestamp_s'] == 10
    assert projector.evidence_failures == 0


def test_input_snapshot_failure_keeps_incomplete_record_without_emit_failure(monkeypatch):
    from navpy.modules.vision.sim.finite_projection_evidence import ProjectionCapture
    records, warnings = [], Mock()
    projector, _, _, _, loc, poi, *_ = rig(records.append, valid=True, pixels=(640, 360), error_sink=warnings)
    monkeypatch.setattr(ProjectionCapture, 'inputs', Mock(side_effect=ValueError('input snapshot unavailable')))
    assert projector.detect(loc, poi, Attitude(0, 40, 0), timestamp_s=12.5).status == DetectStatus.DETECTED
    assert not records[0].outcome.complete and records[0].source.poi_uid is None
    warnings.assert_not_called()
    assert projector.evidence_failures == 0


def test_concurrent_failures_are_counted_exactly_and_warning_failure_is_contained():
    from concurrent.futures import ThreadPoolExecutor
    import threading
    barrier = threading.Barrier(2)
    def fail(record):
        barrier.wait(timeout=2)
        raise RuntimeError('sink unavailable')
    warnings = Mock(side_effect=RuntimeError('warning unavailable'))
    projector, _, _, _, loc, poi, *_ = rig(fail, error_sink=warnings)
    with ThreadPoolExecutor(max_workers=2) as executor:
        jobs = [executor.submit(projector.detect, loc, poi, Attitude(0, 40, 0), timestamp_s=stamp)
                for stamp in (1., 2.)]
        assert all(job.result().status == DetectStatus.OutOfView for job in jobs)
    assert projector.evidence_failures == 2 and warnings.call_count == 2
