"""The test adapter must use the same throttle authority as normal navigation."""
import hashlib
import json
from dataclasses import replace
from types import SimpleNamespace

import pytest

from navpy.modules.common.models.location import Location
from navpy.modules.navigation.nav.terminal_airframe_config import TerminalAirframeConfigProvider, TerminalParameterPort
from scripts import simtime_navigation_config as config
from scripts.simtime_navigation_protocol import ATTITUDE, Snapshot
from scripts.simtime_navigation_runtime import SynchronousNavigation
from tests.scripts.test_simtime_navigation_protocol import packet

PARAMETERS = dict(zip(config.LIMIT_PARAMETERS, (-20., 25., 65., .25)), TRIM_THROTTLE=37.)


@pytest.mark.parametrize('trim,override', [(0., -1.), (37., -1.), (50., -1.), (100., -1.), (37., .1234567), (37., 0.)])
def test_runtime_matches_normal_provider_including_missing_camera_holds(trim, override):
    expected = TerminalAirframeConfigProvider(TerminalParameterPort(dict(PARAMETERS, TRIM_THROTTLE=trim).get),
                                             lambda: None if override < 0 else override * 100).read().throttle
    engine = SynchronousNavigation(Location(40.01, 44., 1300., is_absolute=True), speedup=10,
                                 wall_now=lambda: 100., trim_throttle_percent=trim)
    issued = []
    try:
        for index in range(10):
            snapshot = Snapshot.decode(packet(index + 1))
            snapshot = replace(snapshot, limits=(*snapshot.limits[:4], override))
            command, _ = engine.advance(snapshot, 100.)
            if command.kind == ATTITUDE:
                assert command.mask == 132
                assert command.throttle == expected
                issued.append(command)
        assert len(issued) >= 8
    finally:
        engine.close()


@pytest.mark.parametrize('text', ['AIRSPEED_CRUISE 22', 'TRIM_THROTTLE -1', 'TRIM_THROTTLE 101',
                                  'TRIM_THROTTLE nan', 'TRIM_THROTTLE inf',
                                  'TRIM_THROTTLE 40\nTRIM_THROTTLE 50'])
def test_unusable_defaults_rejected(tmp_path, text):
    path = tmp_path / 'defaults.parm'; path.write_text(text)
    with pytest.raises(ValueError):
        config.read_defaults(path)


def test_nonpositive_tau_keeps_normal_provider_semantics_and_inverted_limits_fail():
    assert config.resolve_config((-20., 25., 65., 0., -1.), 37.).pitch_time_constant_s is None
    with pytest.raises(ValueError, match='configuration or limits'):
        config.resolve_config((25., -20., 65., .25, -1.), 37.)
    with pytest.raises(ValueError, match='exceeds one'):
        config.resolve_config((-20., 25., 65., .25, 1.01), 37.)


@pytest.mark.parametrize('mask,thrust', [(196, 0.), (132, .5), (132, float('nan'))])
def test_command_validation_rejects_tecs_handoff_or_wrong_throttle(mask, thrust):
    with pytest.raises(ValueError, match='delegates throttle'):
        config.validate_command((-20., 25., 65., .25, -1.), ATTITUDE, mask, thrust, PARAMETERS)


def test_command_validation_uses_float32_and_verified_limits():
    limits = (-20., 25., 65., .25, -1.)
    config.validate_command(limits, ATTITUDE, 132, config.expected_thrust(limits, 37.), PARAMETERS)
    with pytest.raises(ValueError, match='limits differ'):
        config.validate_command((-20., 30., 65., .25, -1.), ATTITUDE, 132, .37, PARAMETERS)


def evidence(tmp_path, monkeypatch):
    case = tmp_path
    (case / 'navigation-defaults.parm').write_text(''.join(f'{k} {v}\n' for k, v in PARAMETERS.items()))
    params, policy = config.read_defaults(case / 'navigation-defaults.parm')
    peer = dict(version=4, throttle_policy=policy, records=[dict(snapshot=packet().hex())])
    path = lambda p: str(p).replace('\\', '/')
    monkeypatch.setattr('scripts.eval_simtime_step.wsl_path', path)
    command = ['python', '--defaults', path(case / 'navigation-defaults.parm'), '--sitl-root', '/owned']
    peer_command = ['python', '--defaults', path(case / 'navigation-defaults.parm'), '--directory', '/owned/201', '--fault', 'none']
    (case / 'command.json').write_text(json.dumps(command))
    (case / 'peer-command.json').write_text(json.dumps(peer_command))
    (case / 'flight.BIN').write_bytes(b'bound fixture BIN')
    digest = hashlib.sha256((case / 'flight.BIN').read_bytes()).hexdigest()
    (case / 'logs-before.json').write_text(json.dumps(dict(directory='/owned/201', entries={}, lastlog=0)))
    (case / 'logs-after.json').write_text(json.dumps(dict(directory='/owned/201', entries={'00000001.BIN':dict(size=17)}, lastlog=1)))
    (case / 'bin-binding.json').write_text(json.dumps(dict(source='/owned/201/logs/00000001.BIN', sha256=digest)))
    # Existing reader's conflict semantics are separately exercised below.
    def reader(argv, **kwargs):
        assert json.loads(argv[-1]) == params
        return SimpleNamespace(stdout=json.dumps(dict(binary_sha256=digest)))
    monkeypatch.setattr('subprocess.run', reader)
    return case, peer, dict(defaults_sha256=policy['defaults_sha256'], peer_python='/peer-python')


def test_evidence_accepts_matching_files_and_bin(tmp_path, monkeypatch):
    case, peer, identity = evidence(tmp_path, monkeypatch)
    assert config.validate_evidence(case, case, peer, identity) == PARAMETERS


@pytest.mark.parametrize('change', ['old-version', 'policy', 'peer-path', 'launch-path', 'bin', 'limits', 'identity'])
def test_evidence_rejects_mismatched_binding(tmp_path, monkeypatch, change):
    case, peer, identity = evidence(tmp_path, monkeypatch)
    if change == 'old-version': peer['version'] = 3
    elif change == 'policy': peer['throttle_policy']['trim_throttle_percent'] = 50.
    elif change == 'identity': identity['defaults_sha256'] = 'wrong'
    elif change == 'bin': (case / 'flight.BIN').write_bytes(b'changed')
    elif change == 'limits': (case / 'navigation-defaults.parm').write_text('TRIM_THROTTLE 37')
    else:
        name = 'peer-command.json' if change == 'peer-path' else 'command.json'
        command = json.loads((case / name).read_text()); command[command.index('--defaults') + 1] = '/other'
        (case / name).write_text(json.dumps(command))
    with pytest.raises(ValueError):
        config.validate_evidence(case, case, peer, identity)


def test_actual_parameter_history_must_include_every_value_and_no_conflict():
    from scripts.read_noise_parameters import summarize
    rows = [dict(name=k, value=v, time_us=1) for k,v in PARAMETERS.items()]
    assert summarize(rows, PARAMETERS)['TRIM_THROTTLE']['value'] == 37.
    with pytest.raises(ValueError, match='TRIM_THROTTLE'):
        summarize(rows + [dict(name='TRIM_THROTTLE', value=50., time_us=2)], PARAMETERS)
    with pytest.raises(ValueError, match='TRIM_THROTTLE'):
        summarize(rows[:-1], PARAMETERS)


@pytest.mark.parametrize('override,mask,thrust', [(-1.,196,0.), (0.,132,0.), (65.,132,.65)])
def test_explicit_user_throttle_and_mask_survive_runtime_and_holds(override,mask,thrust):
    engine = SynchronousNavigation(Location(40.01,44.,1300.,is_absolute=True), speedup=10,
                                 wall_now=lambda:100., trim_throttle_percent=37.,
                                 configured_throttle_percent=override)
    try:
        for i in range(10):
            snap=Snapshot.decode(packet(i+1))
            snap=replace(snap,limits=(*snap.limits[:4],-1.))
            command,_=engine.advance(snap,100.)
            if command.kind==ATTITUDE:
                assert command.mask==mask and command.throttle==thrust
                packed=config.expected_thrust(snap.limits,37.,override)
                config.validate_command(snap.limits,ATTITUDE,mask,0. if packed is None else packed,PARAMETERS,override)
    finally:
        engine.close()


@pytest.mark.parametrize('value',[float('nan'),float('inf'),-2.,101.,True])
def test_invalid_explicit_policy_rejected(value):
    with pytest.raises(ValueError,match='throttle configuration'):
        config.resolve_config((-20.,25.,65.,.25,-1.),37.,value)
