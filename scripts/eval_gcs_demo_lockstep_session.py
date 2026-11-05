"""What a lockstep run's script does besides the scripted mission (step 13 of
``deterministic_dev_sim.md``): the app's full-parameter prewarm, and waiting
while the app drives the run."""

from __future__ import annotations

from pathlib import Path

from scripts.eval_gcs_demo_mission import all_have_snap, run_mission
from scripts.eval_gcs_demo_models import RegressionError, StackContext
from scripts.eval_gcs_demo_ports import JsonValue

#: How often an app session (step 13) looks for the fleet's SNAP ends, in sim
#: seconds: well inside the coordinator's 30 s finalization budget, which starts
#: at the last SNAP and waits for the operator to return.
APP_SESSION_POLL_S = 1.0
#: Sim seconds for one full-parameter read: the route's MAVFTP budget (30 s,
#: ``full_params.py``) and its one retry, with margin.
FULL_PARAMETERS_TIMEOUT_S = 90.0


def read_full_parameters(context: StackContext) -> None:
    """The app's connect-time prewarm (``App.jsx``): each vehicle's full table, read
    by MAVFTP, so every scripted run also checks the FTP worker (step 13)."""
    for sys_id in context.sys_ids:
        body = context.api.get_json(f"/api/vehicles/{sys_id}/parameters", timeout=FULL_PARAMETERS_TIMEOUT_S)
        if not isinstance(body, dict) or not body.get("params"):
            raise RegressionError(f"full parameter read of {sys_id} returned no parameters")


def app_session(context: StackContext, session_s: float) -> None:
    """The app drives the run: wait until every UAV reached its SNAP end, as the
    scripted mission's approval loop does, or until ``session_s`` sim seconds passed."""
    deadline = context.clock.monotonic() + session_s
    while not all_have_snap(context) and context.clock.monotonic() < deadline:
        context.clock.sleep(APP_SESSION_POLL_S)


def drive(context: StackContext, run_dir: Path, app_session_s: float | None,
          timeout_s: float, confirmation_delay_s: float) -> list[dict[str, JsonValue]]:
    """The script's part of the run, returning its approvals: with ``app_session_s``
    the app connects, uploads and launches; else the scripted mission, with the prewarm."""
    if app_session_s is not None:
        app_session(context, app_session_s)
        return []
    mission = run_mission(context, run_dir, timeout_s, confirmation_delay_s,
                          after_connect=read_full_parameters)
    return mission.approvals
