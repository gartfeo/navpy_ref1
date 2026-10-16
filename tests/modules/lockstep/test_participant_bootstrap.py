"""Participant bootstrap in a fresh interpreter: import graph and product import.

Step 1 imports only the standard library, ``runtime_time`` and
``navpy.sim.lockstep``. After step 2 (bind, logging stamp, thread guard) the
product imports and is constructed on a managed bootstrap thread at the
admission tick; with pose recording on, its import-time flusher must be a
managed thread, and nothing may start an unmanaged one.
"""

from __future__ import annotations

import json

from tests.guards.step1_fresh import error_tail, run_fresh

IMPORT_GRAPH = """
import json, sys
baseline = set(sys.modules)
import navpy.sim.lockstep.participant
print(json.dumps(sorted(set(sys.modules) - baseline)))
"""

PRODUCT_IMPORT = """
import glob, json, os, sys, threading
from navpy.core.kernel import runtime_time as rt
from navpy.sim.lockstep.baton import audit
from navpy.sim.lockstep.participant import ParticipantSpec, bootstrap

baseline = set(threading.enumerate())
participant = bootstrap(ParticipantSpec(seed=1, ordinal=2, generation=3, epoch_us=1_760_000_000_000_000))
baton = participant.runtime.baton
found = {}

def construct():
    import navpy.main  # the NavPy composition modules
    from navpy.app import runtime_stop
    from navpy.core.kernel.logging import pose_cadence_debug as pose
    found["recording"] = pose.ENABLED
    found["buffer lock"] = type(pose._buffers_lock).__name__
    pose.record_observation(2, rt.monotonic(), 0.0, 0.0, "fresh")
    rt.sleep(0.3)  # the managed flusher writes on its 0.25 s cadence meanwhile
    found["threads"] = sorted(t.name for t in baton.threads.alive())
    pattern = os.path.join(os.environ["NAVPY_POSE_CADENCE_DEBUG"], "pose_cadence_observation_uav_2_*.csv")
    found["periodic rows"] = [  # on disk before the final flush: the flusher wrote them
        open(path, encoding="utf-8").read().splitlines()[1:] for path in glob.glob(pattern)
    ]
    runtime_stop.flush_pose_cadence()

bootstrap_thread = participant.admit(construct)
for k in range(1, 30):  # admission at tick 1
    baton.tick(k * 20_000)
    audit(baton, baseline)
found["bootstrap done"] = bootstrap_thread.done
found["identity"] = participant.identity()
found["used before bind"] = False  # bind() raises if anything used rt first
print(json.dumps(found, sort_keys=True))
"""


def test_the_bootstrap_module_imports_no_product_module():
    result = run_fresh(IMPORT_GRAPH)
    assert result.returncode == 0, error_tail(result, 20)
    loaded = json.loads(result.stdout.strip().splitlines()[-1])
    navpy = sorted(name for name in loaded if name == "navpy" or name.startswith("navpy."))
    assert [name for name in navpy if not name.startswith("navpy.sim.lockstep")] == [
        "navpy", "navpy.core", "navpy.core.kernel", "navpy.core.kernel.runtime_time", "navpy.sim",
    ]
    import sys

    third_party = sorted(
        name for name in loaded
        if name.split(".")[0] not in sys.stdlib_module_names and not name.startswith("navpy")
    )
    assert third_party == []


def test_the_product_imports_on_the_baton_with_pose_recording_on(tmp_path, monkeypatch):
    monkeypatch.setenv("NAVPY_POSE_CADENCE_DEBUG", str(tmp_path))
    result = run_fresh(PRODUCT_IMPORT)
    assert result.returncode == 0, error_tail(result, 30)
    found = json.loads(result.stdout.strip().splitlines()[-1])
    assert found == {
        "bootstrap done": True,
        "buffer lock": "BatonLock",
        "identity": {"generation": 3, "ordinal": 2, "process_pid": 2},
        "periodic rows": [["0.02,0.0,0.0,fresh,"]],  # recorded at the admission tick
        "recording": True,
        "threads": ["bootstrap", "pose-cadence-flush"],
        "used before bind": False,
    }
    csv = [path.name for path in tmp_path.glob("pose_cadence_observation_uav_2_*.csv")]
    assert len(csv) == 1
