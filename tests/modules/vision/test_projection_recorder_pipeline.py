"""Real transaction/composition association without a shared simulator or started worker."""
import json
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from navpy.exception_groups import BaseExceptionGroup
from navpy.modules.common.models.attitude import Attitude
from navpy.modules.vision.sim.sim_detection_context import SimDetectionContext
from navpy.modules.vision.sim.sim_detection_pipeline import SimDetectionPipeline
from navpy.modules.vision.sim.sim_frame_timestamp import SimFrameTimestampResolver
from navpy.modules.vision.sim.sim_frame_transactions import DetectionFrameTransactions
from navpy.modules.vision.sim.projection_run_recorder import ProjectionRunRecorder, create_projection_recorder
from navpy.modules.vision.sim.run_evidence_file import EvidenceBudget, RunEvidenceFile
from navpy.modules.vision.sim.sim_detector_stop_transaction import SimDetectorStopTransaction
from navpy.modules.vision.sim.sim_execution_composition import build_execution_graph
from tests.modules.vision.test_finite_projection_evidence import rig
from tests.modules.vision.test_sim_frame_transactions import _coordinator


@pytest.mark.parametrize('scenario,expected,accepted', [('success','successful',True),('empty','empty',True),('reset','rejected',False),('publish_rejected','rejected',False)])
def test_actual_pipeline_associates_projection_with_committed_empty_or_rejected_frame(tmp_path, scenario, expected, accepted):
    file = RunEvidenceFile(tmp_path, scenario, 3, EvidenceBudget(20,100_000))
    recorder = ProjectionRunRecorder(file)
    projector, _, _, _, loc, target, *_ = rig(recorder, valid=scenario != 'empty', pixels=(640,360))
    coordinator, store, _ = _coordinator()
    def project(location, target, attitude, **kwargs):
        kwargs.pop('navigation_attitude')
        result = projector.detect(location, target, attitude, **kwargs)
        if scenario == 'reset':
            with coordinator.reset():
                pass
        return result.target
    if scenario == 'publish_rejected':
        store.publish = lambda *args, **kwargs: False
    context = SimDetectionContext(False, Mock(), lambda: (target,), project,
                                  SimFrameTimestampResolver(lambda value, **kwargs: value),
                                  DetectionFrameTransactions(coordinator, lambda targets: 'finite'))
    capture, gap, tracking = Mock(), Mock(), Mock()
    capture.wants_frame.return_value = False
    gap.commit.return_value = False
    pipeline = SimDetectionPipeline(context, confirmation_capture=capture, forced_gap_policy=gap,
                                    tracking_updater=tracking, evidence_recorder=recorder)
    token = coordinator.token
    result = pipeline.detect_targets(loc, Attitude(0,40,0), frame_timestamp_s=849.764,
                                     frame_receipt_timestamp_s=900., frame_generation=token, frame_epoch=123)
    assert result is accepted
    recorder.close()
    rows = [json.loads(line) for line in file.path.read_text().splitlines()]
    assert [row['kind'] for row in rows] == ['projection','publication']
    projection, publication = rows
    assert projection['projection']['source']['source_timestamp_s'] == 849.764
    assert publication['frame']['publication'] == expected
    assert projection['frame']['frame_id'] == publication['frame']['frame_id']
    assert projection['frame']['epoch'] == token.epoch and projection['frame']['generation'] == token.generation
    assert projection['frame']['declared_epoch'] == 123 and projection['frame']['receipt_timestamp_s'] == 900.
    if scenario == 'reset':
        capture.capture.assert_not_called()
        tracking.update.assert_not_called()
    assert file.status['complete']


def test_stop_transaction_never_closes_recorder_before_worker_join(tmp_path):
    file = RunEvidenceFile(tmp_path,'stop',3,EvidenceBudget(2,10_000))
    recorder = ProjectionRunRecorder(file)
    worker = Mock()
    worker.join.side_effect = [False,True]
    stop = SimDetectorStopTransaction(Mock(),Mock(),worker,recorder.close)
    assert not stop.run(.01).complete and not file.status['closed']
    assert stop.run(.01).complete and file.status['complete']
    stop.run(.01)
    worker.close.assert_called_once()


def test_execution_composition_wires_owned_recorder_close_without_starting_worker(tmp_path):
    file = RunEvidenceFile(tmp_path,'execution',3,EvidenceBudget(2,10_000))
    recorder = ProjectionRunRecorder(file)
    deps = SimpleNamespace(vehicle=Mock(), mount=Mock(), logger=Mock(), scheduler_cadence=Mock())
    source = SimpleNamespace(coordinator=Mock(),pose_source=Mock(),activation=Mock(),record_outcome=Mock())
    render = SimpleNamespace(target_provider=Mock(),capture=Mock(),gap=Mock(),tracking=Mock(),
                             ideal_camera=Mock(),evidence_recorder=recorder)
    graph = build_execution_graph(deps, SimpleNamespace(ideal_360=False),source,render,Mock())
    assert graph.lifecycle.stop() is True
    assert file.status['complete']
    assert graph.lifecycle.stop() is True


def test_configured_factory_opens_once_and_default_ideal_does_no_io(tmp_path, monkeypatch):
    monkeypatch.setenv('NAVPY_SIM_PROJECTION_RECORDER','1')
    monkeypatch.setenv('NAVPY_SIM_EVIDENCE_DIR',str(tmp_path))
    monkeypatch.setenv('NAVPY_SIM_EVIDENCE_RUN_ID','configured')
    monkeypatch.setenv('NAVPY_SIM_EVIDENCE_MAX_RECORDS','2')
    monkeypatch.setenv('NAVPY_SIM_EVIDENCE_MAX_BYTES','10000')
    assert create_projection_recorder(3,ideal_360=True) is None
    assert list(tmp_path.iterdir()) == []
    recorder = create_projection_recorder(3,ideal_360=False)
    second = create_projection_recorder(3,ideal_360=False,name_reader=lambda:'second-mount')
    assert second.output.path != recorder.output.path
    assert second.output.status['detector_id'] != recorder.output.status['detector_id']
    assert second.output.status['detector_name'] == 'second-mount'
    recorder.close()
    second.close()


def test_real_render_composition_owns_recorder_and_binds_pipeline(tmp_path,monkeypatch):
    from navpy.args.uas_args import UasArgs
    from navpy.modules.navigation.geo.geo_ref_calc import GeoRefCalc
    from navpy.modules.vision.sim.sim_render_composition import build_render_foundation, build_pipeline
    _,camera,_,_,loc,target,*_ = rig(None)
    for key,value in dict(NAVPY_SIM_PROJECTION_RECORDER='1',NAVPY_SIM_EVIDENCE_DIR=str(tmp_path),
                          NAVPY_SIM_EVIDENCE_RUN_ID='render',NAVPY_SIM_EVIDENCE_MAX_RECORDS='20',
                          NAVPY_SIM_EVIDENCE_MAX_BYTES='100000').items():
        monkeypatch.setenv(key,value)
    monkeypatch.setattr('navpy.modules.vision.sim.sim_render_composition.TargetProvider',
                        lambda *args: SimpleNamespace(targets=[target]))
    mount = SimpleNamespace(get_k=camera.read_matrix,get_gimbal_data=camera.read_gimbal,
                            is_valid=camera.pixel_valid,image_width=1280,image_height=720,name='camera',
                            sync_zoom_from_hardware=lambda:False)
    deps = SimpleNamespace(mount=mount,geo_ref=GeoRefCalc(UasArgs()),logger=Mock(),args=Mock(),zc_util=None,
                           vehicle=SimpleNamespace(source_system=3))
    options = SimpleNamespace(ideal_360=False,tracking_config=None,zoom_config=None,sim_assets_path=None)
    coordinator,store,_ = _coordinator()
    source = SimpleNamespace(pose_source=SimpleNamespace(source_now_s=12.5),store=store,record_outcome=Mock(),
                             coordinator=coordinator,clock=Mock())
    render = build_render_foundation(deps,options,source)
    pipeline = build_pipeline(deps,options,source,render)
    assert pipeline.detect_targets(loc,Attitude(0,40,0),frame_timestamp_s=12.5,frame_generation=coordinator.token)
    render.evidence_recorder.close()
    rows = [json.loads(line) for line in render.evidence_recorder.output.path.read_text().splitlines()]
    assert [r['kind'] for r in rows] == ['projection','publication']
    deps.logger.debug.assert_not_called()


def test_composition_failure_closes_recorder_and_preserves_original_error(tmp_path,monkeypatch):
    from navpy.modules.vision.sim.projection_run_recorder import rollback_projection_recorder
    file = RunEvidenceFile(tmp_path,'rollback',3,EvidenceBudget(2,10000))
    recorder = ProjectionRunRecorder(file)
    cause = ValueError('construction failed')
    rollback_projection_recorder(recorder,cause)
    assert file.status['closed'] and not file.status['complete']
    monkeypatch.setattr(recorder,'close',Mock(side_effect=OSError('cleanup failed')))
    with pytest.raises(BaseExceptionGroup) as raised:
        rollback_projection_recorder(recorder,cause)
    assert raised.value.exceptions[0] is cause


def test_stop_waits_for_failed_producer_cleanup_before_sealing_capture(tmp_path):
    file = RunEvidenceFile(tmp_path,'producer-stop',3,EvidenceBudget(2,10000))
    recorder = ProjectionRunRecorder(file)
    coordinator,worker = Mock(),Mock()
    coordinator.stop.side_effect = [RuntimeError('source still owns callbacks'),None]
    worker.join.return_value = True
    stop = SimDetectorStopTransaction(coordinator,Mock(),worker,recorder.close)
    first = stop.run(.01)
    assert not first.complete and first.errors and not file.status['closed']
    assert stop.run(.01).complete and file.status['complete']


@pytest.mark.parametrize('projection_error',[False,True])
def test_recorder_io_failure_keeps_pipeline_result_or_original_exception(tmp_path,projection_error):
    from tests.modules.vision.test_projection_run_recorder import FaultFile
    def opener(path,mode):
        return FaultFile(path.open(mode),'write' if path.suffix == '.jsonl' else None)
    file = RunEvidenceFile(tmp_path,'io-error',3,EvidenceBudget(20,100000),opener=opener)
    recorder = ProjectionRunRecorder(file)
    projector,_,calc,_,loc,target,*_ = rig(recorder,valid=True,pixels=(640,360))
    original = ValueError('original projection failure')
    if projection_error:
        calc.side_effect = original
    def project(location,target,attitude,**kwargs):
        kwargs.pop('navigation_attitude')
        return projector.detect(location,target,attitude,**kwargs).target
    coordinator,_,_ = _coordinator()
    context = SimDetectionContext(False,Mock(),lambda:(target,),project,
                                  SimFrameTimestampResolver(lambda value,**kwargs:value),
                                  DetectionFrameTransactions(coordinator,lambda targets:'finite'))
    capture,gap,tracking = Mock(),Mock(),Mock()
    capture.wants_frame.return_value = False
    gap.commit.return_value = False
    pipeline = SimDetectionPipeline(context,confirmation_capture=capture,forced_gap_policy=gap,
                                    tracking_updater=tracking,evidence_recorder=recorder)
    if projection_error:
        with pytest.raises(ValueError) as caught:
            pipeline.detect_targets(loc,Attitude(0,40,0),frame_timestamp_s=12.5)
        assert caught.value is original
        capture.capture.assert_not_called()
        tracking.update.assert_not_called()
    else:
        assert pipeline.detect_targets(loc,Attitude(0,40,0),frame_timestamp_s=12.5) is True
        capture.capture.assert_called_once()
        tracking.update.assert_called_once()
    recorder.close()
    assert not file.status['complete'] and projector.evidence_failures == 1


@pytest.mark.parametrize('stage',['finish','asdict'])
def test_pre_emit_failure_latches_incomplete_through_production_error_callback(tmp_path,monkeypatch,stage):
    from navpy.args.uas_args import UasArgs
    from navpy.modules.navigation.geo.geo_ref_calc import GeoRefCalc
    from navpy.modules.vision.sim.finite_projection_evidence import ProjectionCapture
    from navpy.modules.vision.sim.ideal_camera_state import IdealCameraState
    from navpy.modules.vision.sim.sim_camera_ports import FrameSize
    from navpy.modules.vision.sim.sim_render_composition import _build_projector
    from tests.modules.vision.test_projection_run_recorder import frame
    file = RunEvidenceFile(tmp_path,stage,3,EvidenceBudget(20,100000))
    recorder = ProjectionRunRecorder(file)
    _,camera,_,_,loc,target,*_ = rig(None)
    mount = SimpleNamespace(get_k=camera.read_matrix,get_gimbal_data=camera.read_gimbal,is_valid=camera.pixel_valid)
    deps = SimpleNamespace(mount=mount,geo_ref=GeoRefCalc(UasArgs()),logger=Mock(),vehicle=SimpleNamespace(source_system=3))
    composed = _build_projector(deps,SimpleNamespace(ideal_360=False),SimpleNamespace(snapshot=lambda:(target,)),
                                SimpleNamespace(now=lambda:12.5),FrameSize(1280,720),
                                IdealCameraState(camera.read_gimbal),recorder)
    if stage == 'finish':
        monkeypatch.setattr(ProjectionCapture,'finish',Mock(side_effect=ValueError('before writer')))
    else:
        monkeypatch.setattr('navpy.modules.vision.sim.projection_run_recorder.asdict',Mock(side_effect=ValueError('before writer')))
    with recorder.frame(frame(1),4):
        composed.detect(loc,target,Attitude(0,40,0),timestamp_s=1.)
    recorder.close()
    assert not file.status['complete'] and file.status['failed'] > 0
    deps.logger.single_warning.assert_called_once()


def test_successfully_recorded_projection_exception_has_abandoned_publication(tmp_path):
    file = RunEvidenceFile(tmp_path,'abandoned',3,EvidenceBudget(20,100000))
    recorder = ProjectionRunRecorder(file)
    projector,_,calc,_,loc,target,*_ = rig(recorder)
    original = ValueError('projection failed')
    calc.side_effect = original
    from tests.modules.vision.test_projection_run_recorder import frame
    with pytest.raises(ValueError) as caught:
        with recorder.frame(frame(1),4):
            projector.detect(loc,target,Attitude(0,40,0),timestamp_s=1.)
    assert caught.value is original
    recorder.close()
    rows = [json.loads(line) for line in file.path.read_text().splitlines()]
    assert rows[0]['projection']['outcome']['exception_type'] == 'ValueError'
    assert rows[-1]['frame']['publication'] == 'abandoned'
    assert rows[-1]['frame']['exception_type'] == 'ValueError'
    assert file.status['complete']
