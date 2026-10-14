"""9f acceptance on a real serial-step SITL (opt-in: ``NAVPY_LOCKSTEP_SITL=1``).

Needs WSL with step 7's serial-step firmware (``scripts/lockstep_firmware.lock.json``).
Each run gets its own WSL root and eval-band slot (``scripts/eval_lockstep_navpy.py``):
NavPy is spawned through the launcher port after 9e's setup, boots on the
serial-step SITL, is stopped through the API, and two runs (the second with
host delays, under another slot) give the same journal and digest.
"""

from __future__ import annotations

import json
import os

import pytest

from gcs.backend import instance_registry_lockstep as lockstep
from scripts import eval_lockstep_navpy as sitl_run

pytestmark = pytest.mark.skipif(os.environ.get("NAVPY_LOCKSTEP_SITL") != "1",
                                reason="real serial-step SITL run: set NAVPY_LOCKSTEP_SITL=1")


def test_navpy_on_a_serial_step_sitl_is_stopped_through_the_api_and_repeats(tmp_path):
    claims = []
    try:
        for _ in range(2):
            claims.append(lockstep.claim(run_token="seed-1", clone=str(sitl_run.ROOT), owner="dir:test"))
        outcomes = [sitl_run.run_once(tmp_path / f"run{i + 1}", slot, owner, seed=1,
                                      delay_seed=7 if i else 0)
                    for i, (slot, owner) in enumerate(claims)]
    finally:
        for slot, owner in claims:
            lockstep.release_if_owner(slot, owner)
    first, second = outcomes
    assert claims[0][0] != claims[1][0]
    assert first.digest == second.digest
    assert first.result == second.result
    journals = [(tmp_path / name / "lockstep_journal.json").read_bytes() for name in ("run1", "run2")]
    lines = [(tmp_path / name / "navpy_log_lines.txt").read_bytes() for name in ("run1", "run2")]
    assert journals[0] == journals[1] and lines[0] == lines[1]
    actor = first.result["actor"]
    assert actor["ready"]["runtime"]["role"] == "navpy" and actor["stopped"]["instances"] == []
    host = json.loads((tmp_path / "run1" / "lockstep_participants_host.json").read_text())
    assert sorted((row["role"], row["exit_code"]) for row in host) == [("backend", 0), ("navpy", 0)]
