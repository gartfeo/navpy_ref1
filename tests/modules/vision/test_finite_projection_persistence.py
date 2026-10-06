"""Offline persistence through the production logger, with no simulator."""
import json
import logging
from unittest.mock import Mock

import pytest

from navpy.args.logger_args import LoggerArgsStub
from navpy.logger.cache_log_level import CacheLogLevel
from navpy.logger.logger_factory import initialize_logger
from navpy.modules.vision.sim.finite_projection_evidence import projection_log_sink
from tests.modules.vision.test_finite_projection_evidence import rig
from navpy.modules.common.models.attitude import Attitude


def close_logger(logger, sysid):
    logger.close()
    text = logging.getLogger(f'vehicle_logger_{sysid}')
    for handler in list(text.handlers):
        handler.flush()
        handler.close()
        text.removeHandler(handler)


def test_filtered_logger_refuses_evidence_activation(monkeypatch, tmp_path):
    monkeypatch.setenv('NAVPY_SIM_PROJECTION_EVIDENCE', '1')
    monkeypatch.setenv('NAVPY_LOG_DIR', str(tmp_path))
    args = LoggerArgsStub()  # Production CLI defaults to INFO, not DEBUG.
    logger = initialize_logger(args, 201)
    try:
        with pytest.raises(ValueError, match='DEBUG logging'):
            projection_log_sink(logger.debug, 201,
                                is_debug_enabled=lambda: logger.is_enabled_for(CacheLogLevel.DEBUG))
    finally:
        close_logger(logger, 201)


def test_production_logger_persists_exact_sequences_after_close(monkeypatch, tmp_path):
    monkeypatch.setenv('NAVPY_SIM_PROJECTION_EVIDENCE', '1')
    monkeypatch.setenv('NAVPY_LOG_DIR', str(tmp_path))
    args = LoggerArgsStub()
    args.log_level = CacheLogLevel.DEBUG
    logger = initialize_logger(args, 202)
    records = []
    projector, _, _, _, loc, target, *_ = rig(records.append)
    projector.detect(loc, target, Attitude(0, 40, 0), timestamp_s=12.5)
    sink = projection_log_sink(logger.debug, 202)
    try:
        for _ in range(1000):
            sink(records[0])
    finally:
        close_logger(logger, 202)
    rows = [json.loads(line.split('SIM_PROJECTION_EVIDENCE ', 1)[1])
            for line in (tmp_path / 'uav_202_navigation.log').read_text(encoding='utf-8').splitlines()
            if 'SIM_PROJECTION_EVIDENCE ' in line]
    assert [row['projection_sequence'] for row in rows] == list(range(1, 1001))
    assert all(row['projection']['geometry']['pixel_uv'] == [1293., 355.] for row in rows)
    assert all(row['projection']['outcome']['complete'] for row in rows)


def test_disabled_activation_does_not_query_logger_level(monkeypatch):
    monkeypatch.delenv('NAVPY_SIM_PROJECTION_EVIDENCE', raising=False)
    level_reader = Mock(side_effect=AssertionError('disabled path queried logger'))
    assert projection_log_sink(Mock(), 203, is_debug_enabled=level_reader) is None
    level_reader.assert_not_called()
