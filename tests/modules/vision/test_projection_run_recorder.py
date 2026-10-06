"""Offline retention/fault/concurrency tests for the nonrotating capture boundary."""
from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import threading

import pytest
from navpy.modules.vision.sim.frame_generation_gate import FrameGeneration
from navpy.modules.vision.sim.projection_capture_check import check_projection_capture
from navpy.modules.vision.sim.projection_run_recorder import ProjectionRunRecorder, create_projection_recorder
from navpy.modules.vision.sim.run_evidence_file import EvidenceBudget, RunEvidenceFile
from navpy.modules.vision.sim.sim_frame_types import FrameContext
from tests.modules.vision.test_finite_projection_evidence import rig
from navpy.modules.common.models.attitude import Attitude


def output(tmp_path, *, records=20000, bytes_=100_000_000, opener=None):
    return RunEvidenceFile(tmp_path, 'capture', 3, EvidenceBudget(records, bytes_),
                           **({'opener': opener} if opener else {}))


def read_capture(file):
    rows = [json.loads(line) for line in file.path.read_text(encoding='utf-8').splitlines()]
    manifest = json.loads(file.path.with_suffix('.manifest.json').read_text(encoding='utf-8'))
    return rows, manifest


def frame(i):
    return FrameContext(float(i), FrameGeneration(4, i), float(i) + .01, None, False)


def test_retains_16000_actual_projection_records_and_both_publication_outcomes(tmp_path):
    file = output(tmp_path)
    recorder = ProjectionRunRecorder(file)
    projector, _, _, _, loc, poi, *_ = rig(recorder, valid=True, pixels=(640, 360))
    for i in range(2):
        with recorder.frame(frame(i), 4) as association:
            for _ in range(8000):
                projector.detect(loc, poi, Attitude(0, 40, 0), timestamp_s=float(i))
            association.published(i == 0, 1)
    recorder.close()
    rows, manifest = read_capture(file)
    assert [r['record_sequence'] for r in rows] == list(range(1, 16003))
    projections = [r for r in rows if r['kind'] == 'projection']
    assert len(projections) == 16000 and all(r['projection']['outcome']['complete'] for r in projections)
    assert projections[0]['frame']['frame_id'] == 1 and projections[-1]['frame']['frame_id'] == 2
    publications = [r for r in rows if r['kind'] == 'publication']
    assert [r['frame']['publication'] for r in publications] == ['successful', 'rejected']
    assert manifest['complete'] and manifest['submitted'] == manifest['written'] == 16002
    assert manifest['failed'] == 0 and manifest['sha256'] == hashlib.sha256(file.path.read_bytes()).hexdigest()
    assert check_projection_capture(file.path, expected_projections=16000)['valid']


def test_record_budget_exact_limit_then_overflow_latches_incomplete_without_overwrite(tmp_path):
    file = output(tmp_path, records=2)
    file.emit('test', {})
    file.emit('test', {})
    before = file.status['bytes_written']
    with pytest.raises(OverflowError):
        file.emit('test', {})
    with pytest.raises(RuntimeError):
        file.emit('test', {})
    file.close()
    rows, manifest = read_capture(file)
    assert len(rows) == 2 and manifest['bytes_written'] == before
    assert not manifest['complete'] and manifest['budget_exhausted']
    assert manifest['submitted'] == 4 and manifest['written'] == 2 and manifest['failed'] == 2


def test_byte_budget_exact_limit_and_limit_plus_one(tmp_path):
    probe = output(tmp_path / 'probe')
    probe.emit('test', {})
    exact = probe.status['bytes_written']
    probe.close()
    file = output(tmp_path / 'actual', bytes_=exact)
    file.emit('test', {})
    with pytest.raises(OverflowError):
        file.emit('test', {})
    file.close()
    rows, manifest = read_capture(file)
    assert len(rows) == 1 and manifest['bytes_written'] == exact and not manifest['complete']
    too_small = output(tmp_path / 'small', bytes_=exact - 1)
    with pytest.raises(OverflowError):
        too_small.emit('test', {})
    too_small.close()
    assert too_small.status['written'] == 0 and not too_small.status['complete']


class FaultFile:
    def __init__(self, raw, fault):
        self.raw, self.fault = raw, fault

    def write(self, data):
        if self.fault == 'write':
            raise OSError('injected write failure')
        if self.fault == 'short':
            return self.raw.write(data[:5])
        return self.raw.write(data)

    def flush(self):
        if self.fault == 'flush':
            raise OSError('injected flush failure')
        return self.raw.flush()

    @property
    def closed(self):
        return self.raw.closed

    def fileno(self):
        return self.raw.fileno()

    def close(self):
        self.raw.close()
        if self.fault == 'close':
            raise OSError('injected close failure')

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()


@pytest.mark.parametrize('fault', ['write', 'short', 'flush', 'close', 'manifest_flush', 'manifest_close'])
def test_injected_faults_never_publish_a_complete_capture(tmp_path, fault):
    def opener(path, mode):
        selected = fault.removeprefix('manifest_') if path.suffix == '.pending' and fault.startswith('manifest_') else fault
        return FaultFile(path.open(mode), selected if path.suffix == '.jsonl' and not fault.startswith('manifest_') or path.suffix == '.pending' and fault.startswith('manifest_') else None)
    file = output(tmp_path, opener=opener)
    if fault in {'write', 'short'}:
        with pytest.raises(OSError):
            file.emit('test', {})
    else:
        file.emit('test', {})
    if fault in {'flush', 'close', 'manifest_flush', 'manifest_close'}:
        with pytest.raises(OSError):
            file.close()
    else:
        file.close()
    assert not file.status['complete'] and file.status['failed'] >= 1
    final = file.path.with_suffix('.manifest.json')
    if final.exists():
        assert not json.loads(final.read_text())['complete']
    else:
        assert fault.startswith('manifest_')


def test_no_overwrite_and_after_close_rejection(tmp_path):
    file = output(tmp_path)
    file.emit('test', {})
    file.close()
    before = file.path.read_bytes()
    with pytest.raises(FileExistsError):
        output(tmp_path)
    with pytest.raises(RuntimeError, match='after close'):
        file.emit('test', {})
    file.close()
    assert file.path.read_bytes() == before


def test_concurrent_frames_keep_context_and_file_sequence_order(tmp_path):
    file = output(tmp_path)
    recorder = ProjectionRunRecorder(file)
    records = []
    projector, _, _, _, loc, poi, *_ = rig(records.append, valid=True, pixels=(640, 360))
    projector.detect(loc, poi, Attitude(0, 40, 0), timestamp_s=12.5)
    barrier = threading.Barrier(8)
    def produce(i):
        with recorder.frame(frame(i), 4) as association:
            barrier.wait(timeout=10)
            for _ in range(50):
                recorder(replace(records[0], source=replace(records[0].source, source_timestamp_s=float(i))))
            association.published(True, 1)
    with ThreadPoolExecutor(max_workers=8) as pool:
        list(pool.map(produce, range(8)))
    recorder.close()
    rows, manifest = read_capture(file)
    assert manifest['complete'] and [r['record_sequence'] for r in rows] == list(range(1, 409))
    for i in range(8):
        subset = [r for r in rows if r['frame']['generation'] == i]
        assert len(subset) == 51 and len({r['frame']['frame_id'] for r in subset}) == 1
        assert subset[-1]['kind'] == 'publication' and subset[-1]['frame']['publication'] == 'successful'


def test_disabled_and_ideal_paths_require_no_configuration_or_files(monkeypatch, tmp_path):
    monkeypatch.delenv('NAVPY_SIM_PROJECTION_RECORDER', raising=False)
    assert create_projection_recorder(3, ideal_360=False) is None
    monkeypatch.setenv('NAVPY_SIM_PROJECTION_RECORDER', '1')
    assert create_projection_recorder(3, ideal_360=True) is None
    assert list(tmp_path.iterdir()) == []


def test_unassociated_projection_invalidates_capture_and_preserves_detection(tmp_path):
    file = output(tmp_path)
    recorder = ProjectionRunRecorder(file)
    projector, _, _, _, loc, poi, *_ = rig(recorder, valid=True, pixels=(640, 360))
    result = projector.detect(loc, poi, Attitude(0, 40, 0), timestamp_s=12.5)
    assert result.poi is not None and projector.evidence_failures == 1
    recorder.close()
    assert not file.status['complete']


def test_close_with_active_frame_is_invalid_and_retriable_after_frame_finishes(tmp_path):
    file = output(tmp_path)
    recorder = ProjectionRunRecorder(file)
    with recorder.frame(frame(1),4):
        with pytest.raises(RuntimeError, match='active'):
            recorder.close()
    recorder.close()
    assert file.status['closed'] and not file.status['complete']


def test_missing_manifest_or_tampered_data_never_pass_integrity_gate(tmp_path):
    file = output(tmp_path)
    recorder = ProjectionRunRecorder(file)
    projector, _, _, _, loc, poi, *_ = rig(recorder,valid=True,pixels=(640,360))
    with recorder.frame(frame(1),4) as association:
        projector.detect(loc,poi,Attitude(0,40,0),timestamp_s=1.)
        association.published(True,1)
    assert not check_projection_capture(file.path)['valid']
    recorder.close()
    assert check_projection_capture(file.path,expected_projections=1)['valid']
    assert not check_projection_capture(file.path,expected_projections=2)['valid']
    file.path.write_bytes(file.path.read_bytes().replace(b'640.0',b'641.0',1))
    assert not check_projection_capture(file.path)['valid']


def test_timestamp_mismatch_keeps_original_detection_but_invalidates_capture(tmp_path):
    file = output(tmp_path)
    recorder = ProjectionRunRecorder(file)
    projector, _, _, _, loc, poi, *_ = rig(recorder,valid=True,pixels=(640,360))
    with recorder.frame(frame(1),4):
        assert projector.detect(loc,poi,Attitude(0,40,0),timestamp_s=2.).poi is not None
    recorder.close()
    assert not file.status['complete'] and projector.evidence_failures == 1
    checked = check_projection_capture(file.path)
    assert not checked['valid'] and 'manifest records an incomplete capture' in checked['reasons']


def test_manifest_publication_failure_leaves_no_completion_manifest(tmp_path, monkeypatch):
    file = output(tmp_path)
    file.emit('test',{})
    monkeypatch.setattr('navpy.modules.vision.sim.run_evidence_file.os.link', lambda *args: (_ for _ in ()).throw(OSError('link failed')))
    with pytest.raises(OSError):
        file.close()
    assert not file.status['complete'] and not file.path.with_suffix('.manifest.json').exists()


def test_nonfinite_serialization_failure_is_incomplete(tmp_path):
    file = output(tmp_path)
    with pytest.raises(ValueError):
        file.emit('test',dict(value=float('nan')))
    file.close()
    assert not file.status['complete'] and file.status['written'] == 0


@pytest.mark.parametrize('file_kind',['jsonl','pending'])
def test_failed_close_retains_open_file_ownership_until_retry(tmp_path,file_kind):
    class RetryCloseFile(FaultFile):
        def close(self):
            if self.fault == 'close_before_release':
                self.fault = None
                raise OSError('resource is still open')
            super().close()
    handles = []
    def opener(path,mode):
        handle = RetryCloseFile(path.open(mode),'close_before_release' if path.suffix == '.'+file_kind else None)
        handles.append(handle)
        return handle
    file = output(tmp_path,opener=opener)
    file.emit('test',{})
    with pytest.raises(OSError):
        file.close()
    selected = handles[0] if file_kind == 'jsonl' else handles[1]
    assert not selected.closed
    assert file.status['closed'] is (file_kind == 'pending')
    assert not file.path.with_suffix('.manifest.json').exists()
    file.close()
    assert selected.closed and file.status['closed'] and not file.status['complete']
