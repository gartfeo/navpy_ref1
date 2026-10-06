"""Independent runs, not sibling vehicles, must establish repeatability."""

import itertools
from pathlib import Path

import pytest

from scripts import compare_lockstep_fleet as comparison


def matrix():
    fixtures = {}
    for index, (speed, delayed, repeat) in enumerate(itertools.product((1., 10.), (False, True), range(3))):
        fixtures[Path(str(index))] = entry(index, 3, speed, delayed)
    fixtures[Path('solo')] = entry(12, 1, 1., False)
    return fixtures


def entry(index, count, speed, delayed):
    return (dict(run_id=str(index), instances=count, speedup=speed, delayed=delayed, chat=40,
                 launcher_sha256='launcher', wrapper_sha256='wrapper', defaults_sha256='defaults',
                 firmware_root='/own', driver_sha256={}),
            {v: (dict(vehicle=v, boot=[index, v], identity={"version": 1}, rows=3000),
                 [{"control": v}]) for v in (121, 122, 123)[:count]})


def arrange(monkeypatch, fixtures):
    monkeypatch.setattr(comparison, "read_run", lambda p: fixtures[p])
    monkeypatch.setattr(comparison, "digest", lambda p: 'hash')


def test_complete_matrix_compares_only_same_vehicle(monkeypatch):
    fixtures = matrix()
    arrange(monkeypatch, fixtures)
    report = comparison.compare(list(fixtures))
    assert report['vehicle_runs'] == 37
    assert report['control_steps'] == 111000
    assert len(report['comparisons']) == 210
    assert all(p['equal'] for p in report['comparisons'])


def test_siblings_cannot_replace_independent_fleet_boots(monkeypatch):
    fixtures = matrix()
    fixtures.pop(Path('1'))
    arrange(monkeypatch, fixtures)
    with pytest.raises(ValueError, match='incomplete'):
        comparison.compare(list(fixtures))


@pytest.mark.parametrize('kind', ['path', 'fleet', 'vehicle', 'slot', 'implementation'])
def test_reused_or_incompatible_runs_fail(monkeypatch, kind):
    fixtures = matrix()
    paths = list(fixtures)
    if kind == 'path':
        paths[1] = paths[0]
    elif kind == 'fleet':
        fixtures[paths[1]][0]['run_id'] = fixtures[paths[0]][0]['run_id']
    elif kind == 'vehicle':
        fixtures[paths[1]][1][121][0]['boot'] = fixtures[paths[0]][1][121][0]['boot']
    elif kind == 'slot':
        fixtures[paths[1]][0]['chat'] = 41
    else:
        fixtures[paths[1]][1][121][0]['identity'] = {'version': 2}
    arrange(monkeypatch, fixtures)
    with pytest.raises(ValueError):
        comparison.compare(paths)


def test_one_changed_control_is_reported_even_when_other_vehicles_match(monkeypatch):
    fixtures = matrix()
    fixtures[Path('0')][1][122][1][0]['control'] = 100
    arrange(monkeypatch, fixtures)
    report = comparison.compare(list(fixtures))
    failures = [p for p in report['comparisons'] if not p['equal']]
    assert len(failures) == 11
    assert all(p['vehicle'] == 122 and p['first_divergence'] == 1 for p in failures)


@pytest.mark.parametrize('status', ['rejected', 'pending', 'running'])
def test_failed_or_unattempted_matrix_cannot_be_published(tmp_path, status):
    import json
    path = tmp_path / 'attempts.json'
    path.write_text(json.dumps({'attempts': [{'status': status}]}))
    with pytest.raises(ValueError, match='uncompleted'):
        comparison.compare_ledger(path)
