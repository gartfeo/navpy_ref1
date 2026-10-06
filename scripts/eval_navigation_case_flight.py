"""In-flight POI gating, SNAP observation, and exact resource teardown."""

from __future__ import annotations

import argparse
import json
import time
from dataclasses import asdict
from pathlib import Path

from scripts import eval_certificate as cert

from eval_navigation_case_ports import CaseProcessPorts
from eval_navigation_case_state import CaseState, ScoringIntervalWindow
from eval_navigation_evidence import (
    await_poi_snap_binding,
    parse_selection_evidence,
    selected_location_from_evidence,
    validate_pre_snap_evidence,
)
from eval_navigation_logs import snap_from_compact
from eval_navigation_scoring import CoordinateScorer
from eval_navigation_telemetry import (
    drain_position_messages,
    position_stream_is_live,
)


def fly_case(state: CaseState, args: argparse.Namespace) -> None:
    """Observe the final-approach scoring interval until one fully bound SNAP is present."""
    processes = state.processes
    assert processes.master is not None and processes.sysid is not None
    deadline = time.time() + args.timeout
    while time.time() < deadline:
        if processes.navpy is not None and processes.navpy.poll() is not None:
            raise RuntimeError(f"NavPy exited code={processes.navpy.returncode}")
        scoring_interval = state.evidence.scoring_interval
        drain_position_messages(
            processes.master,
            sysid=processes.sysid,
            live_anchors=state.evidence.live_anchors,
            scorer=scoring_interval.scorer if scoring_interval else None,
            rate_tracker=scoring_interval.rate_tracker if scoring_interval else None,
        )
        if state.evidence.gate is None:
            _try_accept_poi(state, args)
        if snap_from_compact(state.paths.compact):
            _validate_snap(state)
            return
        time.sleep(0.05)
    raise RuntimeError("no SNAP found before timeout")


def _try_accept_poi(state: CaseState, args: argparse.Namespace) -> None:
    navigation_path = state.paths.navigation
    if navigation_path is None or not navigation_path.exists():
        return
    log_text = navigation_path.read_text(encoding="utf-8", errors="replace")
    evidence = parse_selection_evidence(log_text)
    if (
        evidence.obj_id is None
        or evidence.poi_lat_deg is None
        or evidence.poi_lon_deg is None
        or evidence.poi_abs_alt_m is None
    ):
        return
    expectation = state.evidence.expectation
    home_abs_alt_m = state.evidence.home_abs_alt_m
    assert expectation is not None and home_abs_alt_m is not None
    selected_location = selected_location_from_evidence(
        evidence,
        home_abs_alt_m=home_abs_alt_m,
    )
    gate = validate_pre_snap_evidence(
        expectation,
        evidence,
        selected_location,
        configured_rel_alt_m=state.poi_rel_alt_m,
        coordinate_tolerance_m=args.coordinate_tolerance_m,
        altitude_tolerance_m=args.altitude_tolerance_m,
    )
    state.evidence.selection = evidence
    state.evidence.gate = gate
    (state.paths.case_dir / "poi_evidence.json").write_text(
        json.dumps(
            {
                "selection": evidence.to_record(),
                "selected_location": (
                    asdict(selected_location) if selected_location else None
                ),
                "gate": asdict(gate),
            },
            indent=2,
        ),
        encoding="utf-8",
    )
    if not gate.passed:
        raise RuntimeError("POI evidence rejected: " + "; ".join(gate.errors))
    state.evidence.scoring_interval = ScoringIntervalWindow(
        CoordinateScorer(expectation.location),
        cert.ClockRateTracker(),
    )


def _validate_snap(state: CaseState) -> None:
    gate = state.evidence.gate
    if gate is None or not gate.passed:
        raise RuntimeError(
            "SNAP observed before POI identity/coordinate gate passed"
        )
    state.evidence.coordinate_stream_live_at_snap = position_stream_is_live(
        state.evidence.live_anchors,
        time.time(),
    )
    if not state.evidence.coordinate_stream_live_at_snap:
        raise RuntimeError(
            "GLOBAL_POSITION_INT stream was not at the live edge when SNAP "
            "was observed"
        )
    assert state.paths.navigation is not None
    state.evidence.episode_error = await_poi_snap_binding(
        state.paths.navigation,
        state.evidence.selection,
    )
    if state.evidence.episode_error:
        raise RuntimeError(state.evidence.episode_error)


def teardown_case(
    state: CaseState,
    args: argparse.Namespace,
    *,
    process_ports: CaseProcessPorts,
) -> None:
    """Close telemetry, flush source evidence, and stop only owned processes."""
    master = state.processes.master
    if master is not None:
        try:
            master.close()
        except Exception:
            pass
    navpy = state.processes.navpy
    if navpy is not None and navpy.poll() is None:
        time.sleep(cert.SOURCE_TIME_FLUSH_GRACE_S)
    process_ports.terminate_child(navpy)
    try:
        process_ports.stop_stack(
            args.python,
            chat=state.processes.chat,
        )
    except Exception:
        pass
    process_ports.terminate_child(state.processes.swarm)
