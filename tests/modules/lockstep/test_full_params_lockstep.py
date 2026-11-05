"""Dev sim step 13: the full-parameter read runs MAVFTP on lockstep.

The real backend runs as a participant against the reference autopilot,
which serves ``@PARAM/param.pck`` over MAVFTP with its bursts spread over
ticks. ``GET /api/vehicles/{sys_id}/parameters`` (the app's prewarm and the
Parameters tab) answers the autopilot's table, and the run repeats byte for
byte under host delays: every MAVFTP clock and the bridge's inbox are
``runtime_time``, so the transfer waits in sim time on the baton.
"""

from __future__ import annotations

import json

from navpy.sim.lockstep.operator_baton import OperatorBaton
from scripts.eval_gcs_demo_lifecycle import connect_vehicles
from scripts.lockstep_fleet_actor import stack_context as fleet_context
from scripts.lockstep_slice_actor import BACKEND, SLICE
from tests.modules.lockstep.backend_doubles import BOUND_VALUES, build
from tests.modules.lockstep.lockstep_autopilot import FTP_BURST_PACKETS, SYS_ID, ReferenceAutopilot

TERMINATE, RESET, OPEN_RO, BURST_READ = 1, 2, 4, 15
#: Enough parameters that the file spans several ticks of burst replies.
FILLER = {f"FILL_{index:04d}": float(index) for index in range(300)}


def _script(seen: dict):
    def make(scenario):
        def script(operator: OperatorBaton) -> None:
            operator.wait_ready(BACKEND)
            context = fleet_context(operator, scenario, SLICE)
            connect_vehicles(context)
            seen["body"] = context.api.get_json(f"/api/vehicles/{SYS_ID}/parameters")
        return script
    return make


def _run(tmp_path, name: str, delay_seed: int):
    seen: dict = {}
    autopilot = ReferenceAutopilot(parameters={**BOUND_VALUES, **FILLER})
    run = build(tmp_path / name, delay_seed=delay_seed, script=_script(seen), autopilot=autopilot)
    result = run.run()
    return run, result, seen


def test_the_full_parameter_read_is_served_by_mavftp_and_repeats_under_host_delays(tmp_path):
    plain, plain_result, plain_seen = _run(tmp_path, "plain", 0)
    delayed, delayed_result, delayed_seen = _run(tmp_path, "delayed", 5)

    autopilot = plain.sitl.autopilot
    records = {p["name"]: p["value"] for p in plain_seen["body"]["params"]}
    assert records == {name: float(value) for name, value in autopilot.parameters.items()}
    assert set(BOUND_VALUES) <= set(records)
    # Sessions reset, opened, read in a burst, ended; the file took more than one
    # tick of burst replies.
    requests = autopilot.ftp_requests
    assert requests[:2] == [RESET, OPEN_RO] and BURST_READ in requests and requests[-1] == TERMINATE
    assert len(autopilot.param_pck()) > FTP_BURST_PACKETS * 239

    assert plain_seen == delayed_seen
    assert plain.coordinator.journal.records == delayed.coordinator.journal.records
    assert plain.coordinator.journal.digest == delayed.coordinator.journal.digest
    assert plain.sitl.serial1 == delayed.sitl.serial1
    for run, result in ((plain, plain_result), (delayed, delayed_result)):
        assert result.outcome == "timeout"  # no NavPy: the mission phase ends on its budget
        assert run.dones[-1].ended is True
        assert all(done.faults == () for done in run.dones)
    # The progress broadcasts reached the app's telemetry, ending done.
    progress = [json.loads(e["data"]) for _k, e in plain.events("broadcast")
                if json.loads(e["data"])["type"] == "full_param_progress"]
    assert progress and progress[-1]["done"] is True
