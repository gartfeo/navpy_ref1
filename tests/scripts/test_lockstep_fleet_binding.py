"""Exercise real artifact/profile binding, independently of trace decoding."""

import argparse
import json
from pathlib import Path

import pytest

from scripts import compare_lockstep_fleet as check
from scripts.eval_lockstep_fleet import HOME, resolve_home, run


def fixture(tmp_path, monkeypatch):
    root = tmp_path / 'run'
    case = root / '121'
    case.mkdir(parents=True)
    for name in ('peer.json', 'identity.json', 'navpy-navigation.csv', 'peer-command.json',
                 'launch-probe.sh', 'navigation-defaults.parm', 'flight.BIN',
                 'logs-before.json', 'logs-after.json', 'bin-binding.json'):
        (case / name).write_text('{}' if name == 'peer.json' else name)
    (case / 'peer-command.json').write_text(json.dumps(['python', 'peer.py']))
    for name in ('supervisor.log', 'teardown.log', 'launch-unlock.json'):
        (root / name).write_text(name)
    command = ['python', 'swarm_run.py', '--eval', '--single-boot', '--instances', '1',
               '--home', check.HOME, '--speedup', '1.0', '--dist', '0', '--sitl-root', '/own',
               '--defaults', check.wsl_path(case / 'navigation-defaults.parm'),
               '--sitl-binary', check.wsl_path(case / 'launch-probe.sh')]
    (root / 'command.json').write_text(json.dumps(command))
    manifest = dict(version=1, run_id='fixture-run', camera_hz=40, supervisor_alive=False, status='captured', errors=[],
        instances=1, home=check.HOME, firmware_root='/own', template_directories=[], driver_sha256={'a': 'b'},
        vehicles=[121], chat=40, shared_profile_vehicle=121, speedup=1., delayed=False,
        wrapper_sha256=check.digest(case / 'launch-probe.sh'),
        defaults_sha256=check.digest(case / 'navigation-defaults.parm'),
        evidence_sha256={p.relative_to(root).as_posix(): check.digest(p) for p in root.rglob('*') if p.is_file()})
    (root / 'fleet.json').write_text(json.dumps(manifest))
    monkeypatch.setattr(check, 'driver_identity', lambda: {'a': 'b'})
    monkeypatch.setattr(check, 'validate', lambda *a, **k: (dict(vehicle=121, camera_hz=40,
        requested_speed=1., delayed=False), []))
    return root, manifest


def test_valid_artifact_bindings_are_accepted(tmp_path, monkeypatch):
    root, _ = fixture(tmp_path, monkeypatch)
    assert list(check.read_run(root)[1]) == [121]


def test_explicit_home_is_bound_to_manifest_and_launch(tmp_path, monkeypatch, capsys):
    root, manifest = fixture(tmp_path, monkeypatch)
    home = resolve_home("43,34,60,0")
    assert resolve_home(None) == HOME
    manifest["home"] = home
    manifest["dock"] = [43., 34., 90.]
    peer_command_path = root / "121/peer-command.json"
    peer_command_path.write_text(json.dumps(['python', 'peer.py', '--dock', '43', '34', '90']))
    manifest["evidence_sha256"]["121/peer-command.json"] = check.digest(peer_command_path)
    peer_path = root / "121/peer.json"
    peer_path.write_text(json.dumps({"dock": manifest["dock"]}))
    manifest["evidence_sha256"]["121/peer.json"] = check.digest(peer_path)
    command_path = root / "command.json"
    command = json.loads(command_path.read_text())
    command[command.index("--home") + 1] = home
    command_path.write_text(json.dumps(command))
    manifest["evidence_sha256"]["command.json"] = check.digest(command_path)
    (root / "fleet.json").write_text(json.dumps(manifest))

    assert list(check.read_run(root, home="43,34,60,0")[1]) == [121]
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr("sys.argv", ["compare_lockstep_fleet.py", "--run", root.name,
                                     "--home", "43,34,60,0"])
    check.main()
    assert json.loads(capsys.readouterr().out)["home"] == home
    with pytest.raises(ValueError, match="unaccepted fleet lifecycle/profile"):
        check.read_run(root)

    command[command.index("--home") + 1] = HOME
    command_path.write_text(json.dumps(command))
    manifest["evidence_sha256"]["command.json"] = check.digest(command_path)
    (root / "fleet.json").write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match="launched profile differs: --home"):
        check.read_run(root, home="43,34,60,0")


def test_peer_dock_must_match_manifest_even_when_hashes_match(tmp_path, monkeypatch):
    root, manifest = fixture(tmp_path, monkeypatch)
    manifest["dock"] = [43., 34., 90.]
    peer_command_path = root / "121/peer-command.json"
    peer_command_path.write_text(json.dumps(['python', 'peer.py', '--dock', '43', '34', '91']))
    manifest["evidence_sha256"]["121/peer-command.json"] = check.digest(peer_command_path)
    (root / "fleet.json").write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match="peer dock command differs"):
        check.read_run(root)


def test_peer_recorded_dock_must_match_manifest_even_when_hashes_match(tmp_path, monkeypatch):
    root, manifest = fixture(tmp_path, monkeypatch)
    manifest["dock"] = [43., 34., 90.]
    peer_command_path = root / "121/peer-command.json"
    peer_command_path.write_text(json.dumps(['python', 'peer.py', '--dock', '43', '34', '90']))
    manifest["evidence_sha256"]["121/peer-command.json"] = check.digest(peer_command_path)
    peer_path = root / "121/peer.json"
    peer_path.write_text(json.dumps({"dock": [43., 34., 91.]}))
    manifest["evidence_sha256"]["121/peer.json"] = check.digest(peer_path)
    (root / "fleet.json").write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match="peer recorded dock differs"):
        check.read_run(root)


def test_undeclared_peer_dock_is_rejected(tmp_path, monkeypatch):
    root, manifest = fixture(tmp_path, monkeypatch)
    peer_command_path = root / "121/peer-command.json"
    peer_command_path.write_text(json.dumps(['python', 'peer.py', '--dock', '43', '34', '90']))
    manifest["evidence_sha256"]["121/peer-command.json"] = check.digest(peer_command_path)
    (root / "fleet.json").write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match="peer dock command differs"):
        check.read_run(root)


def test_explicit_home_without_declared_dock_is_rejected(tmp_path, monkeypatch):
    root, manifest = fixture(tmp_path, monkeypatch)
    manifest["home"] = resolve_home("43,34,60,0")
    command_path = root / "command.json"
    command = json.loads(command_path.read_text())
    command[command.index("--home") + 1] = manifest["home"]
    command_path.write_text(json.dumps(command))
    manifest["evidence_sha256"]["command.json"] = check.digest(command_path)
    (root / "fleet.json").write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match="explicit dock in captured evidence"):
        check.read_run(root, home="43,34,60,0")


def test_non_list_peer_command_is_rejected(tmp_path, monkeypatch):
    root, manifest = fixture(tmp_path, monkeypatch)
    peer_command_path = root / "121/peer-command.json"
    peer_command_path.write_text(json.dumps({"--dock": "43,34,90"}))
    manifest["evidence_sha256"]["121/peer-command.json"] = check.digest(peer_command_path)
    (root / "fleet.json").write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match="peer dock command differs"):
        check.read_run(root)


def test_single_run_cli_accepts_pose_control_profile(tmp_path, monkeypatch, capsys):
    root = tmp_path / "run"
    root.mkdir()
    (root / "fleet.json").write_text("{}")
    observed = {}

    def fake_read_run(directory, **kwargs):
        observed.update(kwargs)
        return dict(run_id="fixture-run", status="captured", home=HOME), {121: None}

    monkeypatch.setattr(check, "read_run", fake_read_run)
    monkeypatch.setattr("sys.argv", ["compare_lockstep_fleet.py", "--run", str(root),
                                     "--noise-profile", "noise-off-1000", "--pose-control"])
    check.main()
    assert observed["pose_capture"] is False
    assert json.loads(capsys.readouterr().out)["manifest_sha256"] == check.digest(root / "fleet.json")


@pytest.mark.parametrize("raw", ["43,34,60", "nan,34,60,0", "91,34,60,0", "43,181,60,0",
                                 "43,34,60,0;echo", "43,34,60,30", "43.123456789,34,60,0"])
def test_invalid_explicit_home_is_rejected(raw):
    with pytest.raises(argparse.ArgumentTypeError):
        resolve_home(raw)


def test_eight_decimal_launch_home_is_accepted_without_claiming_effective_sitl_position():
    assert resolve_home("43.12345678,34.12345678,60,-0.0") == "43.12345678,34.12345678,60.0,0.0"


def test_explicit_home_requires_dock_before_creating_capture(tmp_path):
    from types import SimpleNamespace

    args = SimpleNamespace(home="43,34,60,0", dock=None)
    with pytest.raises(ValueError, match="requires --dock with a selected point"):
        run(args, tmp_path / "run")
    assert not (tmp_path / "run").exists()


@pytest.mark.parametrize('field,value', [
    ('status', 'rejected'), ('errors', ['failed']), ('supervisor_alive', True), ('version', 2),
    ('camera_hz', 50), ('template_directories', ['/own/1']), ('driver_sha256', {}),
    ('vehicles', [122]), ('shared_profile_vehicle', 122), ('wrapper_sha256', 'wrong'),
    ('defaults_sha256', 'wrong'), ('evidence_sha256', {}), ('firmware_root', '/other'),
])
def test_changed_profile_binding_is_rejected(tmp_path, monkeypatch, field, value):
    root, manifest = fixture(tmp_path, monkeypatch)
    manifest[field] = value
    (root / 'fleet.json').write_text(json.dumps(manifest))
    with pytest.raises(ValueError):
        check.read_run(root)


@pytest.mark.parametrize('flag', ['--instances', '--home', '--speedup', '--dist', '--defaults', '--sitl-binary'])
def test_command_must_name_the_captured_profile(tmp_path, monkeypatch, flag):
    root, manifest = fixture(tmp_path, monkeypatch)
    path = root / 'command.json'
    command = json.loads(path.read_text())
    command[command.index(flag) + 1] = 'other'
    path.write_text(json.dumps(command))
    manifest['evidence_sha256']['command.json'] = check.digest(path)
    (root / 'fleet.json').write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match='launched profile'):
        check.read_run(root)


def test_evidence_cannot_escape_or_change(tmp_path, monkeypatch):
    root, manifest = fixture(tmp_path, monkeypatch)
    manifest['evidence_sha256']['../foreign'] = 'anything'
    (root / 'fleet.json').write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match='hash mismatch'):
        check.read_run(root)
    del manifest['evidence_sha256']['../foreign']
    (root / 'fleet.json').write_text(json.dumps(manifest))
    (root / '121/peer.json').write_text('changed')
    with pytest.raises(ValueError, match='hash mismatch'):
        check.read_run(root)


def test_trace_must_belong_to_declared_vehicle(tmp_path, monkeypatch):
    root, _ = fixture(tmp_path, monkeypatch)
    monkeypatch.setattr(check, 'validate', lambda *a, **k: (dict(vehicle=122, camera_hz=40,
        requested_speed=1., delayed=False), []))
    with pytest.raises(ValueError, match='trace identity'):
        check.read_run(root)
