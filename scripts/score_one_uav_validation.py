"""Strict evidence boundaries for offline one-UAV approach scoring."""
from __future__ import annotations

import ctypes
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path
import sys

COORDINATE_SOURCE_SHA256 = 'afac76b27ff8e2b26c0a34b90583add8914cd77d318de925d680b1993bafd52b'
COORDINATE_LIBRARY_SHA256 = '9671c294e1f1ffd61d5238a15ef4b56c983bee7c20869290784690c96d3237d6'
REFERENCE_FIRMWARE_COMMIT = '584dba4a6cf6813d8601ff9e6f65cf1bc2ae3d91'
REFERENCE_FIRMWARE_BINARY_SHA256 = '061e847a3059456883371889dae25d46103f58ae2fb6bcc1b0ee9adb16a621e7'
REQUIRED_PARAMETERS = {
    'SIM_NOISE_OFF': 8191, 'SIM_RATE_HZ': 1000,
    'SIM_PLD_ENABLE': 0, 'SIM_SHIP_ENABLE': 0,
}
RUNTIME_PACKAGES = ('numpy', 'pymap3d', 'geopy', 'opencv-python-headless', 'pymavlink')
# Captured source-state boundary spacing in this lockstep profile (20 ms).
MAX_POSE_AGE_US = 20_000


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def argument(command: list, flag: str, count: int = 1) -> list[str]:
    require(isinstance(command, list) and command.count(flag) == 1, f'missing/duplicate {flag}')
    index = command.index(flag)
    values = command[index + 1:index + 1 + count]
    require(len(values) == count, f'truncated {flag}')
    return values


def validate_geometry(directory: Path, manifest: dict, checked: dict,
                      home: str, dock: list[float]) -> int:
    """Require explicit geometry and exactly the manifest's single vehicle."""
    vehicles = manifest.get('vehicles')
    require(type(manifest.get('instances')) is int and manifest['instances'] == 1,
            'scorer requires exactly one instance')
    require(isinstance(vehicles, list) and len(vehicles) == 1
            and type(vehicles[0]) is int and 1 <= vehicles[0] <= 255,
            'scorer requires exactly one vehicle')
    require(set(checked) == set(vehicles), 'checked vehicle identity differs')
    expected_home = list(map(float, home.split(',')))
    require(len(expected_home) == 4 and all(map(math.isfinite, expected_home))
            and -90 <= expected_home[0] <= 90 and -180 <= expected_home[1] <= 180
            and expected_home[3] == 0, 'invalid expected HOME')
    require(len(dock) == 3 and all(map(math.isfinite, dock))
            and -90 <= dock[0] <= 90 and -180 <= dock[1] <= 180,
            'invalid expected dock')
    require(list(map(float, manifest['home'].split(','))) == expected_home, 'wrong HOME')
    require(manifest.get('dock') == dock, 'wrong manifest dock')
    vehicle = vehicles[0]
    case = directory / str(vehicle)
    peer = json.loads((case / 'peer.json').read_text())
    require(peer.get('dock') == dock, 'wrong peer dock')
    command = json.loads((case / 'peer-command.json').read_text())
    require(list(map(float, argument(command, '--dock', 3))) == dock, 'wrong command dock')
    launch = json.loads((directory / 'command.json').read_text())
    require(argument(launch, '--dist') == ['0'], 'nonzero launch displacement')
    require(argument(launch, '--instances') == ['1'], 'wrong launched instance count')
    require(list(map(float, argument(launch, '--home')[0].split(','))) == expected_home,
            'wrong launched HOME')
    require(launch.count('--single-boot') == 1 and launch.count('--eval') == 1,
            'scorer requires an isolated single-boot capture')
    return vehicle


def validate_runtime(identity: dict) -> None:
    runtime = dict(python=sys.version, packages={
        name: importlib.metadata.version(name) for name in RUNTIME_PACKAGES})
    require(runtime == identity['runtime'], 'scoring runtime differs from capture')
    require(Path(sys.executable).resolve() == Path(identity['peer_python']).resolve(),
            'scoring interpreter differs from capture')


def validate_firmware(identity: dict) -> None:
    """Ground-plane reconstruction is qualified only for this reference build."""
    require(identity.get('firmware_head') == REFERENCE_FIRMWARE_COMMIT
            and identity.get('binary_sha256') == REFERENCE_FIRMWARE_BINARY_SHA256,
            'firmware differs from the qualified reference build')


def validate_parameter(name: str, value: float) -> None:
    if name in REQUIRED_PARAMETERS:
        require(math.isfinite(value) and value == REQUIRED_PARAMETERS[name],
                f'wrong BIN parameter occurrence: {name}')


def validate_raw_samples(raw: list[dict]) -> None:
    require(len(raw) >= 2, 'missing SIM2 samples')
    previous = -1
    for row in raw:
        require(all(math.isfinite(row[key]) for key in ('TimeUS', 'PN', 'PE', 'PD', 'VN', 'VE', 'VD')),
                'nonfinite SIM2 source/position/velocity')
        require(row['TimeUS'] > previous, 'SIM2 timestamps must strictly increase')
        previous = row['TimeUS']


def read_bin(path: Path) -> dict:
    """Check every required parameter occurrence, never just its final value."""
    from pymavlink import mavutil
    result = dict(params={}, raw=[], terrain=[], messages=[], controller=[], bad=0)
    groups = {'SIM2': 'raw', 'TERR': 'terrain', 'MSG': 'messages',
              **{name: 'controller' for name in ('PIDP', 'PIDR', 'CTUN', 'ATT', 'ARSP')}}
    connection = mavutil.mavlink_connection(str(path))
    try:
        while (message := connection.recv_match()) is not None:
            kind = message.get_type()
            if kind == 'BAD_DATA':
                result['bad'] += 1
            elif kind == 'PARM':
                validate_parameter(message.Name, message.Value)
                result['params'][message.Name] = message.Value
            elif kind in groups:
                result[groups[kind]].append(message.to_dict())
    finally:
        connection.close()
    require(result['bad'] == 0, 'BAD_DATA in BIN')
    require(REQUIRED_PARAMETERS.keys() <= result['params'].keys(), 'missing BIN parameters')
    validate_raw_samples(result['raw'])
    return result


def validate_environment(terrain: list[dict], messages: list[dict], start: int, end: int) -> list[dict]:
    require(terrain and all(row['Loaded'] == 0 and row['Status'] == 1 for row in terrain),
            'missing or contradictory terrain fallback evidence')
    contacts = [row for row in messages if 'SIM Hit ground' in row['Message']]
    require(all(math.isfinite(row['TimeUS']) for row in contacts), 'invalid contact timestamp')
    require(not any(start <= row['TimeUS'] <= end for row in contacts), 'ground contact during scoring')
    return contacts


def validate_trace(trace: list[dict]) -> None:
    for phase in ('approach', 'after_pass'):
        rows = [row for row in trace if row['phase'] == phase]
        require(rows, f'missing {phase} samples')
        require(all(math.isfinite(row['raw_clearance_m']) and row['raw_clearance_m'] > 0 for row in rows),
                f'nonpositive or nonfinite {phase} clearance')
    require(all(0 <= row['pose_age_us'] <= MAX_POSE_AGE_US for row in trace), 'stale pose association')


def equality_across_runs(values: list) -> bool | None:
    """A single run has no cross-boot comparison result."""
    return None if len(values) < 2 else all(value == values[0] for value in values[1:])


def load_coordinates(library: Path, source: Path) -> ctypes.CDLL:
    """Load only the independently reproduced native source/binary pair."""
    require(digest(source) == COORDINATE_SOURCE_SHA256, 'coordinate source identity changed')
    require(digest(library) == COORDINATE_LIBRARY_SHA256, 'coordinate library identity changed')
    loaded = ctypes.CDLL(str(library.resolve()))
    require(Path(loaded._name).resolve() == library.resolve(), 'wrong loaded coordinate library')
    loaded.scale.argtypes = [ctypes.c_int32]
    loaded.scale.restype = ctypes.c_double
    loaded.metres.restype = loaded.inverse_metres.restype = ctypes.c_double
    return loaded


def input_hashes(accepted: list[tuple[Path, dict, dict]], library: Path, source: Path) -> dict:
    paths = [library.resolve(), source.resolve(), Path(__file__).resolve(),
             Path(__file__).with_name('score_one_uav.py').resolve()]
    paths.extend(Path(__file__).with_name(name).resolve() for name in
                 ('compare_lockstep_fleet.py', 'compare_simtime_navigation.py',
                  'pose_precision_evidence.py', 'pose_rounding_evidence.py'))
    for directory, manifest, _ in accepted:
        paths.append(directory / 'fleet.json')
        for name, expected in manifest['evidence_sha256'].items():
            path = directory / name
            require(path.resolve().is_relative_to(directory.resolve()), 'input hash path escapes run')
            require(digest(path) == expected, 'evidence changed after validation')
            paths.append(path)
    return {str(path): digest(path) for path in paths}
