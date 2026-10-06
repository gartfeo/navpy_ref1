"""Interruptions and partial peer startup must reject and clean every owned peer."""

from types import SimpleNamespace
import json
import argparse

import pytest

from scripts import eval_lockstep_fleet as fleet


@pytest.mark.parametrize('dock', [None, [40.3207414, 44.4552111, 1324.85], [-0.00001, 44.4552111, 1324.85]])
@pytest.mark.parametrize('failure', [KeyboardInterrupt(), RuntimeError('second peer failed')])
def test_failed_second_peer_cleans_first_and_never_claims_capture(tmp_path, monkeypatch, failure, dock):
    monkeypatch.setattr(fleet, 'ROOT', tmp_path)
    monkeypatch.setattr(fleet, 'driver_identity', lambda: {})
    monkeypatch.setattr(fleet.swarm_run, '_resolve_chat', lambda *a, **k: 40)
    monkeypatch.setattr(fleet.registry, 'get', lambda chat: {})
    monkeypatch.setattr(fleet, 'record_identity', lambda *a: None)
    monkeypatch.setattr(fleet, 'log_operation', lambda op, root, vehicle, case: {'directory': root + '/' + str(vehicle)})
    monkeypatch.setattr(fleet.subprocess, 'run', lambda command, **kw:
        SimpleNamespace(stdout='hash launcher' if 'sha256sum' in command else '[]'))
    calls = []
    def artifacts(operation, firmware, vehicle, case, mode):
        calls.append((operation, vehicle))
        if operation == 'prepare':
            (case / 'launch-probe.sh').write_text('wrapper')
            (case / 'navigation-defaults.parm').write_text('defaults')
            return {'directory': '/own/' + str(vehicle)}
        return {'collected': False}
    monkeypatch.setattr(fleet, 'artifacts', artifacts)
    monkeypatch.setattr(fleet.subprocess, 'Popen', lambda *a, **k:
                        SimpleNamespace(wait=lambda **k: 0))
    ready = []
    def wait(*args):
        ready.append(1)
        if len(ready) == 2:
            raise failure
    monkeypatch.setattr(fleet, 'wait_marker', wait)
    args = SimpleNamespace(instances=3, speedup=10., delayed=False,
                           firmware_root='/own', peer_python='/python', dock=dock)
    with pytest.raises(type(failure)):
        fleet.run(args, tmp_path / 'case')
    manifest = json.loads((tmp_path / 'case/fleet.json').read_text())
    assert manifest['status'] == 'rejected'
    assert manifest['errors']
    assert not manifest['supervisor_alive']
    assert [v for op, v in calls if op == 'stop-peer'] == [121, 122, 123]
    assert [v for op, v in calls if op == 'collect'] == [121, 122, 123]

    for vehicle in (121, 122):
        command = json.loads((tmp_path / f'case/{vehicle}/peer-command.json').read_text())
        if dock is None:
            assert '--dock' not in command
            assert 'dock' not in manifest
        else:
            offset = command.index('--dock')
            assert [float(x) for x in command[offset + 1:offset + 4]] == dock
            assert manifest['dock'] == dock
            parser = argparse.ArgumentParser()
            parser.add_argument('--dock', type=float, nargs=3)
            assert parser.parse_args(command[offset:offset + 4]).dock == dock


@pytest.mark.parametrize('dock', [[91, 0, 30], [0, 181, 30], [0, 0, float('nan')],
                                  [0, float('inf'), 30], [0, 0]])
def test_invalid_dock_rejected_before_startup(tmp_path, dock):
    directory = tmp_path / 'invalid'
    with pytest.raises(ValueError, match='dock'):
        fleet.run(SimpleNamespace(dock=dock), directory)
    assert not directory.exists()
