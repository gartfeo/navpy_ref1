"""Offline one-UAV approach scoring; no result enters vehicle commands.

Numerical reconstruction follows the preserved 2026-09-24 scorer. Raw CPA is
limited to seed through first pass; interpolation is an estimate, not a bound.
"""
from pathlib import Path
import argparse
import sys
import json
import csv
import math
import bisect
import hashlib
import ctypes
from collections import Counter
import numpy as np
import pymap3d
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'src')]
from simtime_navigation_protocol import Snapshot
from pose_precision_evidence import read_precision
from score_one_uav_validation import require, validate_geometry, validate_runtime, validate_firmware, load_coordinates, input_hashes, read_bin, validate_environment, validate_trace, equality_across_runs

def closest(points: list[tuple[int, np.ndarray]]) -> dict:
    require(len(points) >= 2, 'closest approach needs two samples')
    require(all(math.isfinite(t) and np.isfinite(p).all() for t, p in points), 'nonfinite scoring points')
    require(all(b[0] > a[0] for a, b in zip(points, points[1:])), 'scoring times must increase')
    best = None
    for (ta, a), (tb, b) in zip(points, points[1:]):
        delta = b - a
        q = float(delta @ delta)
        u = float(np.clip(-float(a @ delta) / q, 0, 1)) if q else 0.0
        v = a + u * delta
        dist = float(np.linalg.norm(v))
        if best is None or dist < best['distance_m']:
            best = dict(distance_m=dist, time_s=(ta + u * (tb - ta)) / 1000000.0, ned_m=v.tolist(), horizontal_m=float(np.linalg.norm(v[:2])), vertical_m=float(v[2]), fraction=u, segment_start_us=ta, segment_end_us=tb)
    sample = min(points, key=lambda r: float(r[1] @ r[1]))
    best['sampled_distance_m'] = float(np.linalg.norm(sample[1]))
    best['sampled_time_s'] = sample[0] / 1000000.0
    return best

def interpolation_estimate(trace: list[dict], raw_cpa: dict) -> tuple[float, float]:
    """Return adjacent logged acceleration and its interpolation error scale."""
    close = [r for r in trace if raw_cpa['segment_start_us'] - 40000 <= r['source_s'] * 1000000.0 <= raw_cpa['segment_end_us'] + 40000]
    acceleration = []
    for a, b in zip(close, close[1:]):
        dt = b['source_s'] - a['source_s']
        acceleration.append(float(np.linalg.norm(np.array([b[k] - a[k] for k in ('vn', 've', 'vd')])) / dt))
    dt = (raw_cpa['segment_end_us'] - raw_cpa['segment_start_us']) / 1000000.0
    require(acceleration and all(map(math.isfinite, acceleration)), 'missing finite local acceleration samples')
    return max(acceleration), max(acceleration) * dt * dt / 8


def score_case(directory: Path, manifest: dict, checked: dict, expected_home: str,
               expected_dock: list[float], lib: ctypes.CDLL, out: Path) -> tuple[dict, list]:
    """Score evidence already accepted by its captured-source fleet validator."""
    vehicle = validate_geometry(directory, manifest, checked, expected_home, expected_dock)
    name = directory.name
    case = directory / str(vehicle)
    identity, history = checked[vehicle]
    validate_firmware(identity['identity'])
    validate_runtime(identity['identity'])
    factor = lib.metres()
    inverse = lib.inverse_metres()
    peer = json.loads((case / 'peer.json').read_text())
    dock = peer['dock']
    require(dock == expected_dock, 'peer dock differs from requested geometry')
    cmd = json.loads((case / 'peer-command.json').read_text())
    require(cmd.count('--dock') == 1, "invalid scoring evidence: cmd.count('--dock') == 1")
    n = cmd.index('--dock')
    require(list(map(float, cmd[n + 1:n + 4])) == dock, 'invalid scoring evidence: list(map(float, cmd[n + 1:n + 4])) == dock')
    poses, pairs = read_precision(case, peer, 'precast')
    ss = [Snapshot.decode(bytes.fromhex(x['snapshot'])) for x in peer['records']]
    seed = peer['seed_step']
    passed = next((i + 1 for i, r in enumerate(peer['records']) if r['evidence'].get('passed')), None)
    require(passed is not None, 'capture has no first-pass event')
    require(1 <= seed < passed < len(ss), 'invalid seed/pass interval or missing post-pass evidence')
    require(len(poses) == len(pairs) == len(ss), 'pose/source count mismatch')
    start = ss[seed - 1].identity.source_us
    end = ss[passed - 1].identity.source_us
    last = ss[-1].identity.source_us
    times = [s.identity.source_us for s in ss]
    require(all(b > a for a, b in zip(times, times[1:])), 'source timestamps must increase')
    launch = json.loads((directory / 'command.json').read_text())
    require(launch[launch.index('--dist') + 1] == '0', "invalid scoring evidence: launch[launch.index('--dist') + 1] == '0'")
    parts = manifest['home'].split(',')
    hlat = int(float(format(float(parts[0]), '.8f')) * 10000000.0)
    hlng = int(float(format(float(parts[1]), '.8f')) * 10000000.0)
    parsed_home_alt = int(float(parts[2]) * 100)
    require(parsed_home_alt == int(poses[0]['home_alt']), "invalid scoring evidence: parsed_home_alt == int(poses[0]['home_alt'])")
    homealt = parsed_home_alt * 0.01
    require(all((int(p['home_alt']) == int(poses[0]['home_alt']) for p in poses)), "invalid scoring evidence: all((int(p['home_alt']) == int(poses[0]['home_alt']) for p in poses))")
    binary = read_bin(case / 'flight.BIN')
    params, raw, terrain, messages, controller = (binary[key] for key in
        ('params', 'raw', 'terrain', 'messages', 'controller'))
    contacts = validate_environment(terrain, messages, start, end)
    ground = float(np.float32(np.float32(int(poses[0]['home_alt'])) * np.float32(0.01)))

    def origin_offset(p: dict) -> tuple[float, float]:
        lat = int(p['origin_lat'])
        lon = int(p['origin_lng'])
        return ((lat - hlat) * factor, (lon - hlng) * factor * lib.scale(int((lat + hlat) / 2)))

    def geo(p: dict, north: float, east: float, down: float) -> tuple[float, float, float]:
        dn, de = origin_offset(p)
        n = north - dn
        e = east - de
        dlat = n * inverse
        return ((int(p['origin_lat']) + dlat) * 1e-07, (int(p['origin_lng']) + e * inverse / lib.scale(int(p['origin_lat']) + int(int(dlat) / 2))) * 1e-07, (int(p['home_alt']) - down * 100) * 0.01)
    maximum_reconstruction_m = 0.0
    for p, pair in zip(poses, pairs):
        dn, de = origin_offset(p)
        dlat = float.fromhex(p['dlat_hex'])
        dlng = float.fromhex(p['dlng_hex'])
        n = dn + dlat / inverse
        e = de + dlng * lib.scale(int(p['origin_lat']) + int(int(dlat) / 2)) / inverse
        d = (int(p['home_alt']) - float.fromhex(p['alt_cm_hex'])) / 100
        reconstructed = geo(p, n, e, d)
        maximum_reconstruction_m = max(maximum_reconstruction_m, float(np.linalg.norm(pymap3d.geodetic2ned(*reconstructed, *pair.precast))))
    require(maximum_reconstruction_m < 1e-06, 'invalid scoring evidence: maximum_reconstruction_m < 1e-06')
    rawpoints = []
    renderpoints = []
    trace = []
    for r in raw:
        t = r['TimeUS']
        if not start <= t <= last:
            continue
        index = bisect.bisect_right(times, t) - 1
        p = poses[index]
        pos = geo(p, r['PN'], r['PE'], r['PD'])
        ned = np.array(pymap3d.geodetic2ned(*pos, *dock))
        clear = pos[2] - ground - 0.1
        require(all((math.isfinite(x) for x in (*pos, *ned, clear))), 'nonfinite reconstructed pose')
        trace.append(dict(
            source_s=t / 1000000.0,
            phase='approach' if t <= end else 'after_pass',
            north_m=r['PN'],
            east_m=r['PE'],
            down_m=r['PD'],
            raw_lat=pos[0],
            raw_lon=pos[1],
            raw_alt_m=pos[2],
            raw_clearance_m=clear,
            relative_north_m=ned[0],
            relative_east_m=ned[1],
            relative_down_m=ned[2],
            distance_m=float(np.linalg.norm(ned)),
            vn=r['VN'],
            ve=r['VE'],
            vd=r['VD'],
            pose_age_us=t - times[index],
        ))
        if t <= end:
            rawpoints.append((t, ned))
    for s in ss[seed - 1:passed]:
        tr = s.truth
        renderpoints.append((s.identity.source_us, np.array(pymap3d.geodetic2ned(tr.latitude, tr.longitude, tr.altitude, *dock))))
    validate_trace(trace)
    raw_cpa = closest(rawpoints)
    render_cpa = closest(renderpoints)
    acceleration, estimate = interpolation_estimate(trace, raw_cpa)
    report = dict(
        directory=str(directory),
        run_id=manifest['run_id'],
        boot=identity['boot'],
        speedup=peer['speedup'],
        delayed=peer['delayed'],
        dock=dock,
        home=[hlat * 1e-07, hlng * 1e-07, homealt],
        seed_step=seed,
        first_pass_step=passed,
        seed_time_s=start / 1000000.0,
        pass_time_s=end / 1000000.0,
        last_boundary_s=last / 1000000.0,
        contact_messages=contacts,
        raw_closest=raw_cpa,
        render_closest=render_cpa,
        raw_log_intervals_us=dict(Counter((b[0] - a[0] for a, b in zip(rawpoints, rawpoints[1:])))),
        coordinate_reconstruction_max_error_m=maximum_reconstruction_m,
        local_acceleration_estimate_mps2=acceleration,
        linear_interpolation_error_scale_m=estimate,
        interpolation_limit='Estimate from adjacent logged velocity changes, not a proven continuous-acceleration bound; raw physics substeps are not logged.',
        terrain_records=len(terrain),
        terrain_loaded_values=sorted({m['Loaded'] for m in terrain}),
        terrain_status_values=sorted({m['Status'] for m in terrain}),
        terrain_model='Flat home-plane fallback observed; real terrain unavailable.',
        point_height_above_model_ground_m=dock[2] - ground,
        minimum_raw_clearance_approach_m=min((r['raw_clearance_m'] for r in trace if r['phase'] == 'approach')),
        minimum_raw_clearance_after_pass_m=min((r['raw_clearance_m'] for r in trace if r['phase'] == 'after_pass')),
        navigation_entry_raw_clearance_m=trace[0]['raw_clearance_m'],
        parameters={k: params.get(k) for k in ('SIM_NOISE_OFF', 'SIM_RATE_HZ', 'SIM_TERRAIN', 'SIM_GND_BEHAV', 'SIM_WIND_SPD', 'SIM_PLD_ENABLE', 'SIM_SHIP_ENABLE', 'RLL2SRV_TCONST', 'PTCH2SRV_TCONST', 'SIM_SERVO_SPEED')},
        source_identity=identity['identity'],
        trace_hash=hashlib.sha256(json.dumps(history, sort_keys=True).encode()).hexdigest(),
    )
    require(report['minimum_raw_clearance_approach_m'] > 0 and report['minimum_raw_clearance_after_pass_m'] > 0, "invalid scoring evidence: report['minimum_raw_clearance_approach_m'] > 0 and report['minimum_raw_clearance_after_pass_m'] > 0")
    with (out / (name + '-raw.csv')).open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(trace[0]))
        w.writeheader()
        w.writerows(trace)
    (out / (name + '-bin.json')).write_text(json.dumps(dict(params=params, terrain=terrain, messages=messages, controller=controller), indent=2, allow_nan=False))
    (out / (name + '-result.json')).write_text(json.dumps(report, indent=2, allow_nan=False))
    report['raw_trace_sha256'] = hashlib.sha256(json.dumps(trace, sort_keys=True).encode()).hexdigest()
    return (report, history)

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence-root', required=True, type=Path)
    parser.add_argument('--run', required=True, action='append')
    parser.add_argument('--home', required=True)
    parser.add_argument('--dock', required=True, nargs=3, type=float)
    parser.add_argument('--coordinates', required=True, type=Path)
    parser.add_argument('--coordinate-source', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    from compare_lockstep_fleet import read_run
    from eval_lockstep_fleet import resolve_home
    home = resolve_home(args.home)
    require(len(set(args.run)) == len(args.run), 'duplicate run')
    require(not args.output.exists(), 'output directory already exists')
    root = args.evidence_root.resolve()
    directories = []
    for name in args.run:
        require(Path(name).name == name and name not in ('.', '..'), 'run must be a directory name')
        directory = (root / name).resolve()
        require(directory.parent == root, 'run escapes evidence root')
        directories.append(directory)
    lib = load_coordinates(args.coordinates, args.coordinate_source)
    accepted = []
    for directory in directories:
        manifest, checked = read_run(directory, home=home, noise_profile='noise-off-1000', pose_capture=True, pose_source='precast')
        validate_geometry(directory, manifest, checked, home, args.dock)
        accepted.append((directory, manifest, checked))
    hashes = input_hashes(accepted, args.coordinates, args.coordinate_source)
    args.output.mkdir(parents=True, exist_ok=False)
    reports = []
    histories = []
    for directory, manifest, checked in accepted:
        report, history = score_case(directory, manifest, checked, home, args.dock, lib, args.output)
        reports.append(report)
        histories.append(history)
    require(len({tuple(r['boot']) for r in reports}) == len(reports), 'duplicate vehicle boot')
    require(all((r['source_identity'] == reports[0]['source_identity'] for r in reports)), 'mixed flight implementation')
    for key in ('firmware_root', 'wrapper_sha256', 'defaults_sha256', 'launcher_sha256'):
        require(len({m[key] for _, m, _ in accepted}) == 1, 'mixed fleet profile: ' + key)
    require(hashes == input_hashes(accepted, args.coordinates, args.coordinate_source), 'inputs changed during scoring')
    result = dict(
        runs=reports,
        history_equal=equality_across_runs(histories),
        raw_closest_equal=equality_across_runs([r['raw_closest'] for r in reports]),
        steps=sum(map(len, histories)),
        expected_home=home,
        expected_dock=args.dock,
        coordinate_library=str(args.coordinates.resolve()),
        input_sha256=hashes,
        limitation='Observed simulated approach only; no accuracy threshold or physical dock capture claim.',
    )
    (args.output / 'airborne-results.json').write_text(json.dumps(result, indent=2, allow_nan=False))
    print(json.dumps(dict(output=str(args.output), runs=len(reports), history_equal=result['history_equal'])))
if __name__ == '__main__':
    main()
