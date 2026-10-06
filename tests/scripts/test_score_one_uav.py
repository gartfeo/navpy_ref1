"""Geometry and evidence rejection gates for the offline one-UAV scorer."""
import json
from pathlib import Path
import sys
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'scripts'))
import score_one_uav as scorer
import score_one_uav_validation as validation


@pytest.fixture
def capture(tmp_path):
    vehicle = 7  # Intentionally not the archived vehicle ID.
    case = tmp_path / str(vehicle)
    case.mkdir()
    home, dock = '43,34,0,0', [43.009, 34., 30.]
    manifest = dict(instances=1, vehicles=[vehicle], home=home, dock=dock)
    (case / 'peer.json').write_text(json.dumps(dict(dock=dock)))
    (case / 'peer-command.json').write_text(json.dumps(['peer', '--dock', *map(str, dock)]))
    (tmp_path / 'command.json').write_text(json.dumps([
        'launch', '--instances', '1', '--home', home, '--dist', '0', '--single-boot', '--eval']))
    return tmp_path, manifest, {vehicle: ({}, [])}, home, dock


def test_explicit_single_vehicle_geometry(capture):
    assert validation.validate_geometry(*capture) == 7


@pytest.mark.parametrize('vehicles', [[], [7, 8], ['7'], [False]])
def test_reject_bad_vehicle_manifest(capture, vehicles):
    capture[1]['vehicles'] = vehicles
    with pytest.raises(ValueError):
        validation.validate_geometry(*capture)


@pytest.mark.parametrize('field,value', [('instances', 0), ('instances', 3),
    ('instances', True), ('home', '44,34,0,0'), ('dock', [43.01, 34., 30.])])
def test_reject_wrong_manifest_geometry(capture, field, value):
    capture[1][field] = value
    with pytest.raises(ValueError):
        validation.validate_geometry(*capture)


def test_reject_checked_id_mismatch(capture):
    capture[2][8] = capture[2].pop(7)
    with pytest.raises(ValueError, match='checked vehicle'):
        validation.validate_geometry(*capture)


@pytest.mark.parametrize('command', [['peer'], ['peer', '--dock', '43', '34', '30'],
                                   ['peer', '--dock', '43.009', '34']])
def test_reject_missing_or_wrong_command_dock(capture, command):
    (capture[0] / '7/peer-command.json').write_text(json.dumps(command))
    with pytest.raises(ValueError):
        validation.validate_geometry(*capture)


def test_reject_wrong_peer_dock(capture):
    (capture[0] / '7/peer.json').write_text(json.dumps(dict(dock=[0, 0, 0])))
    with pytest.raises(ValueError, match='peer dock'):
        validation.validate_geometry(*capture)


@pytest.mark.parametrize('field', ['TimeUS', 'PN', 'PE', 'PD', 'VN', 'VE', 'VD'])
def test_reject_nonfinite_source_position_velocity(field):
    rows = [dict(TimeUS=t, PN=0., PE=0., PD=-30., VN=1., VE=0., VD=0.) for t in (1, 2)]
    rows[1][field] = float('nan')
    with pytest.raises(ValueError, match='nonfinite'):
        validation.validate_raw_samples(rows)


def test_every_parameter_occurrence_is_checked_and_socket_closed():
    connection = MagicMock()
    wrong = MagicMock(Name='SIM_RATE_HZ', Value=500)
    wrong.get_type.return_value = 'PARM'
    corrected = MagicMock(Name='SIM_RATE_HZ', Value=1000)
    corrected.get_type.return_value = 'PARM'
    connection.recv_match.side_effect = [wrong, corrected, None]
    with patch('pymavlink.mavutil.mavlink_connection', return_value=connection), \
         pytest.raises(ValueError, match='wrong BIN parameter occurrence'):
        validation.read_bin(Path('fixture.BIN'))
    connection.close.assert_called_once()


@pytest.mark.parametrize('kind', ['BAD_DATA', 'missing'])
def test_reject_bad_bin_data_or_missing_parameters(kind):
    connection = MagicMock()
    message = MagicMock()
    message.get_type.return_value = kind
    connection.recv_match.side_effect = [message, None]
    with patch('pymavlink.mavutil.mavlink_connection', return_value=connection), \
         pytest.raises(ValueError):
        validation.read_bin(Path('fixture.BIN'))
    connection.close.assert_called_once()


@pytest.mark.parametrize('terrain', [[], [dict(Loaded=1, Status=1)],
    [dict(Loaded=0, Status=1), dict(Loaded=0, Status=0)]])
def test_reject_missing_or_contradictory_terrain(terrain):
    with pytest.raises(ValueError, match='terrain'):
        validation.validate_environment(terrain, [], 10, 20)


def test_ground_contact_rejected_only_in_scoring_interval():
    terrain = [dict(Loaded=0, Status=1)]
    contact = dict(Message='SIM Hit ground', TimeUS=10)
    with pytest.raises(ValueError, match='ground contact'):
        validation.validate_environment(terrain, [contact], 10, 20)
    assert validation.validate_environment(terrain, [contact], 11, 20) == [contact]


def test_interpolated_closest_and_bad_time_or_position():
    points = [(0, np.array([-1., 2., 0.])), (1000000, np.array([1., 2., 0.]))]
    result = scorer.closest(points)
    assert result['distance_m'] == 2
    assert result['time_s'] == .5
    with pytest.raises(ValueError, match='times'):
        scorer.closest([points[0], points[0]])
    with pytest.raises(ValueError, match='nonfinite'):
        scorer.closest([(0, np.array([float('nan'), 0, 0])), points[1]])


def test_changed_coordinate_source_fails_before_native_load(tmp_path):
    source = tmp_path / 'coordinates.cpp'
    source.write_text('changed')
    with patch.object(validation.ctypes, 'CDLL') as load, pytest.raises(ValueError, match='source identity'):
        validation.load_coordinates(tmp_path / 'coordinates.so', source)
    load.assert_not_called()


def test_changed_coordinate_binary_fails_before_native_load(tmp_path):
    library = tmp_path / 'coordinates.so'
    library.write_bytes(b'changed')
    with patch.object(validation, 'digest', side_effect=[validation.COORDINATE_SOURCE_SHA256, 'bad']), \
         patch.object(validation.ctypes, 'CDLL') as load, pytest.raises(ValueError, match='library identity'):
        validation.load_coordinates(library, tmp_path / 'coordinates.cpp')
    load.assert_not_called()


def test_existing_output_directory_is_never_reused(tmp_path, monkeypatch):
    monkeypatch.setattr(sys, 'argv', ['score', '--evidence-root', str(tmp_path), '--run', 'example',
        '--home', '43,34,0,0', '--dock', '43.009', '34', '30', '--coordinates', 'unused.so',
        '--coordinate-source', 'unused.cpp', '--output', str(tmp_path)])
    with pytest.raises(ValueError, match='already exists'):
        scorer.main()


@pytest.mark.parametrize('phase', ['approach', 'after_pass'])
@pytest.mark.parametrize('clearance', [0, -1, float('nan')])
def test_reject_nonpositive_clearance(phase, clearance):
    rows = [dict(phase=p, raw_clearance_m=1, pose_age_us=0) for p in ('approach', 'after_pass')]
    next(row for row in rows if row['phase'] == phase)['raw_clearance_m'] = clearance
    with pytest.raises(ValueError, match='clearance'):
        validation.validate_trace(rows)


def test_reject_missing_post_pass_and_stale_pose():
    rows = [dict(phase='approach', raw_clearance_m=1, pose_age_us=0)]
    with pytest.raises(ValueError, match='missing after_pass'):
        validation.validate_trace(rows)
    rows.append(dict(phase='after_pass', raw_clearance_m=1, pose_age_us=20001))
    with pytest.raises(ValueError, match='stale pose'):
        validation.validate_trace(rows)


def test_changed_runtime_is_rejected():
    with patch.object(validation.importlib.metadata, 'version', return_value='test'), \
         pytest.raises(ValueError, match='runtime differs'):
        validation.validate_runtime(dict(runtime={}, peer_python=sys.executable))


def test_only_qualified_reference_firmware_is_accepted():
    identity = dict(firmware_head=validation.REFERENCE_FIRMWARE_COMMIT,
                    binary_sha256=validation.REFERENCE_FIRMWARE_BINARY_SHA256)
    validation.validate_firmware(identity)
    for key in ('firmware_head', 'binary_sha256'):
        with pytest.raises(ValueError, match='qualified reference'):
            validation.validate_firmware({**identity, key: 'changed'})


def test_current_source_validator_rejects_one_changed_identity_hash(tmp_path):
    import compare_simtime_navigation as comparator
    defaults = tmp_path / 'navigation-defaults.parm'
    defaults.write_bytes(b'fixture')
    identity = dict(source_sha256=comparator.source_hashes(comparator.ROOT),
                    defaults_sha256=validation.digest(defaults))
    comparator.validate_source_identity(identity, tmp_path)
    identity['source_sha256']['src/navpy/__init__.py'] = 'changed'
    with pytest.raises(ValueError, match='source identity differs'):
        comparator.validate_source_identity(identity, tmp_path)


def test_current_fleet_validator_rejects_driver_identity_mismatch(tmp_path):
    from compare_lockstep_fleet import read_run
    from eval_lockstep_fleet import resolve_home
    home = resolve_home('43,34,0,0')
    manifest = dict(noise_profile='noise-off-1000', pose_capture=True, pose_source='precast',
        instances=1, version=1, camera_hz=40, supervisor_alive=False, status='captured', errors=[],
        home=home, template_directories=[], dock=[43.009, 34., 30.], driver_sha256={})
    (tmp_path / 'fleet.json').write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match='driver identity changed'):
        read_run(tmp_path, home=home, noise_profile='noise-off-1000', pose_capture=True, pose_source='precast')


def test_input_hashes_bind_rounding_validator_and_reject_changed_evidence(tmp_path):
    evidence = tmp_path / 'flight.BIN'
    evidence.write_bytes(b'captured')
    (tmp_path / 'fleet.json').write_text('{}')
    source = tmp_path / 'coordinates.cpp'
    source.write_text('source')
    library = tmp_path / 'coordinates.so'
    library.write_bytes(b'library')
    accepted = [(tmp_path, dict(evidence_sha256={'flight.BIN': validation.digest(evidence)}), {})]
    hashes = validation.input_hashes(accepted, library, source)
    rounding = Path(validation.__file__).with_name('pose_rounding_evidence.py').resolve()
    assert hashes[str(rounding)] == validation.digest(rounding)
    evidence.write_bytes(b'changed')
    with pytest.raises(ValueError, match='evidence changed'):
        validation.input_hashes(accepted, library, source)


@pytest.mark.parametrize('values,expected', [([], None), ([1], None), ([1, 1], True), ([1, 2], False)])
def test_equality_requires_two_independent_runs(values, expected):
    assert validation.equality_across_runs(values) is expected


@pytest.mark.parametrize('runs', [['../outside'], ['.'], ['same', 'same']])
def test_cli_rejects_traversal_or_duplicate_run(tmp_path, monkeypatch, runs):
    argv = ['score', '--evidence-root', str(tmp_path), '--home', '43,34,0,0',
            '--dock', '43.009', '34', '30', '--coordinates', 'unused.so',
            '--coordinate-source', 'unused.cpp', '--output', str(tmp_path / 'new')]
    for run in runs:
        argv += ['--run', run]
    monkeypatch.setattr(sys, 'argv', argv)
    with pytest.raises(ValueError, match='run'):
        scorer.main()
    assert not (tmp_path / 'new').exists()


def test_runtime_rejects_wrong_interpreter_even_when_versions_match(tmp_path):
    runtime = dict(python=sys.version, packages={name: 'test' for name in validation.RUNTIME_PACKAGES})
    with patch.object(validation.importlib.metadata, 'version', return_value='test'), \
         pytest.raises(ValueError, match='interpreter differs'):
        validation.validate_runtime(dict(runtime=runtime, peer_python=str(tmp_path / 'other-python')))
