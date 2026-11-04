"""The replay CLI's comparison (``scripts/eval_gcs_demo_lockstep_replay.py``)."""

from __future__ import annotations

import json
from types import SimpleNamespace

import pytest

from navpy.sim.lockstep.replay import ActionFile, ActionReplay
from scripts.eval_gcs_demo_lockstep import JOURNAL_FILE
from scripts.eval_gcs_demo_lockstep_replay import MANIFEST_FILE, prefix, verdict
from scripts.eval_gcs_demo_models import RegressionError

ACTION = {"id": [0, 1], "k": 2, "kind": "request", "payload": {"kind": "http"}, "source": "relay", "target": 2}
RUN = [{"boot": 1}, {"k": 1, "phase": "boot"}, {"k": 2, "phase": "mission", "operator": {"seen": 1}},
       {"k": 3, "phase": "mission"}, {"end": 3}]


def _write(directory, records, manifest):
    directory.mkdir()
    (directory / JOURNAL_FILE).write_text("".join(json.dumps(r) + "\n" for r in records), encoding="utf-8")
    (directory / MANIFEST_FILE).write_text(json.dumps(manifest), encoding="utf-8")


def _driver(actions, admitted):
    file = ActionFile(tuple(actions), 3)
    feed = ActionReplay(file)
    for k in sorted({a["k"] for a in actions}):
        feed.k = k
        feed.take("relay")
    return SimpleNamespace(file=file, feed=feed, side=SimpleNamespace(admission=SimpleNamespace(actions=admitted)))


def test_prefix_cuts_after_tick_n_and_refuses_a_missing_tick():
    assert prefix(RUN, None) == RUN
    assert prefix(RUN, 2) == RUN[:3]
    with pytest.raises(RegressionError):
        prefix(RUN, 9)


def test_verdict_matches_without_operator_notes_and_with_other_sources(tmp_path):
    _write(tmp_path / "run", RUN, {"run_options": {"seed": 1}, "source": {"commit": "a"}})
    replay = [{k: v for k, v in r.items() if k != "operator"} for r in RUN]
    _write(tmp_path / "replay", replay, {"run_options": {"seed": 1}, "source": {"commit": "b"}})
    report = verdict(tmp_path / "run", tmp_path / "replay", _driver([ACTION], [ACTION]), None)
    assert report["matched"] and report["first_difference"] is None and report["actions_equal"]


def test_verdict_names_the_first_difference_and_unequal_inputs(tmp_path):
    _write(tmp_path / "run", RUN, {"run_options": {"seed": 1}})
    _write(tmp_path / "replay", [*RUN[:3], {"k": 3, "phase": "drain"}, RUN[4]], {"run_options": {"seed": 2}})
    report = verdict(tmp_path / "run", tmp_path / "replay", _driver([ACTION], []), None)
    assert not report["matched"]
    assert report["first_difference"] == {"index": 3, "k": 3}
    assert not report["actions_equal"] and not report["inputs_equal"]


def test_verdict_at_stop_compares_the_prefix_and_needs_the_stop_record(tmp_path):
    _write(tmp_path / "run", RUN, {"run_options": {"seed": 1}})
    _write(tmp_path / "stopped", [*RUN[:3], {"stopped_at": 2, "phase": "mission", "snapshot": {}}],
           {"run_options": {"seed": 1}})
    _write(tmp_path / "cut", RUN[:3], {"run_options": {"seed": 1}})
    driver = _driver([ACTION], [ACTION])
    assert verdict(tmp_path / "run", tmp_path / "stopped", driver, 2)["matched"]
    assert not verdict(tmp_path / "run", tmp_path / "cut", driver, 2)["matched"]
