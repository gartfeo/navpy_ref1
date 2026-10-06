"""Production composition must wire both projection records and errors."""
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from navpy.args.uas_args import UasArgs
from navpy.modules.common.models.attitude import Attitude
from navpy.modules.navigation.geo.geo_ref_calc import GeoRefCalc
from navpy.modules.vision.models.detect_data import DetectStatus
from navpy.modules.vision.sim.ideal_camera_state import IdealCameraState
from navpy.modules.vision.sim.sim_camera_ports import FrameSize
from navpy.modules.vision.sim.sim_render_composition import _build_projector
from tests.modules.vision.test_finite_projection_evidence import rig


def composed(logger, *, ideal_360=False):
    _, camera, _, _, loc, poi, *_ = rig(None)
    mount = SimpleNamespace(get_k=camera.read_matrix, get_gimbal_data=camera.read_gimbal,
                            is_valid=camera.pixel_valid)
    dependencies = SimpleNamespace(mount=mount, geo_ref=GeoRefCalc(UasArgs()), logger=logger,
                                   vehicle=SimpleNamespace(source_system=3))
    projector = _build_projector(dependencies, SimpleNamespace(ideal_360=ideal_360),
                                 SimpleNamespace(snapshot=lambda: (poi,)),
                                 SimpleNamespace(now=lambda: 12.5), FrameSize(1280, 720),
                                 IdealCameraState(camera.read_gimbal))
    return projector, loc, poi


def test_disabled_production_composition_never_queries_level_or_emits(monkeypatch):
    monkeypatch.delenv('NAVPY_SIM_PROJECTION_EVIDENCE', raising=False)
    logger = Mock()
    logger.is_enabled_for.side_effect = AssertionError('disabled activation queried level')
    projector, loc, poi = composed(logger)
    assert projector.detect(loc, poi, Attitude(0, 40, 0), timestamp_s=12.5).status == DetectStatus.OutOfView
    logger.debug.assert_not_called()
    logger.warning.assert_not_called()
    logger.single_warning.assert_not_called()


def test_enabled_production_composition_emits_record(monkeypatch):
    monkeypatch.setenv('NAVPY_SIM_PROJECTION_EVIDENCE', '1')
    logger = Mock()
    logger.is_enabled_for.return_value = True
    projector, loc, poi = composed(logger)
    projector.detect(loc, poi, Attitude(0, 40, 0), timestamp_s=12.5)
    assert logger.debug.call_args.args[0].startswith('SIM_PROJECTION_EVIDENCE ')
    logger.is_enabled_for.assert_called_once()
    logger.warning.assert_not_called()
    logger.single_warning.assert_not_called()


def test_production_composition_rejects_filtered_logger(monkeypatch):
    monkeypatch.setenv('NAVPY_SIM_PROJECTION_EVIDENCE', '1')
    logger = Mock()
    logger.is_enabled_for.return_value = False
    with pytest.raises(ValueError, match='DEBUG logging'):
        composed(logger)


def test_production_composition_surfaces_record_write_failure(monkeypatch):
    monkeypatch.setenv('NAVPY_SIM_PROJECTION_EVIDENCE', '1')
    logger = Mock()
    logger.is_enabled_for.return_value = True
    logger.debug.side_effect = RuntimeError('record sink unavailable')
    projector, loc, poi = composed(logger)
    assert projector.detect(loc, poi, Attitude(0, 40, 0), timestamp_s=12.5).status == DetectStatus.OutOfView
    projector.detect(loc, poi, Attitude(0, 40, 0), timestamp_s=12.5)
    assert logger.single_warning.call_count == 2
    first, second = logger.single_warning.call_args_list
    assert first == second
    assert first.args[0].startswith('SIM_PROJECTION_EVIDENCE_ERROR ')
    assert first.kwargs['key'] == 'sim_projection_evidence_error'
    logger.warning.assert_not_called()


def test_ideal_projection_does_not_activate_finite_evidence(monkeypatch):
    monkeypatch.setenv('NAVPY_SIM_PROJECTION_EVIDENCE', '1')
    logger = Mock()
    logger.is_enabled_for.side_effect = AssertionError('unused finite evidence queried logging')
    projector, loc, poi = composed(logger, ideal_360=True)
    projector.detect(loc, poi, Attitude(0, 40, 0), timestamp_s=12.5)
    logger.is_enabled_for.assert_not_called()
    logger.debug.assert_not_called()
    logger.single_warning.assert_not_called()
