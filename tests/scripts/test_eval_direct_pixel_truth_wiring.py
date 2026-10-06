"""Runtime truth wiring: request -> _fly routing -> finalize -> verdict.

The first implementation round shipped three defects a green suite missed
because nothing exercised this seam (Codex findings 3, 5, 6): parsers that
advertised a policy they ignored, and verdict fields assembled from state the
flight loop never produced.  These tests run ``run_case`` and ``_fly`` for
real with everything below the harness stubbed at module seams -- no SITL, no
sockets, no child process.
"""

from __future__ import annotations

import importlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from navpy.modules.vision.sim import determinism_trace
from scripts import eval_direct_pixel_pn as pn
from scripts import eval_direct_pixel_pn_three_uav as three

# The stage module must be THE instance whose globals SimCpaStage's methods
# resolve: pn imports it top-level (scripts/ is on sys.path), and importing
# it as scripts.eval_sim_cpa_stage here would patch a second, unused copy.
stage = importlib.import_module(pn.SimCpaStage.__module__)
from scripts.eval_navigation_models import PoiLocation
from scripts.pixel_pn_case_manifest import (
    MANIFEST_NAME as CASE_MANIFEST,
    case_definition,
    definition_sha256,
)
from scripts.pixel_pn_final_approach_speed import FinalApproachSpeedPlan

from pymavlink import mavutil


POI = PoiLocation(
    lat_deg=43.0, lon_deg=34.0, rel_alt_m=60.0, abs_alt_m=500.0
)


def test_scoring_policy_flag_only_exists_where_it_is_implemented() -> None:
    args = pn._parser(scoring_policy=True).parse_args([])
    assert args.scoring_policy == pn.SCORING_POLICY_SITL_TRUTH

    plain = pn._parser().parse_args([])
    assert plain.scoring_policy == "vehicle-estimate"
    with pytest.raises(SystemExit):
        pn._parser().parse_args(["--scoring-policy", "sitl-truth"])


def test_three_uav_parser_is_pinned_to_the_estimate_policy() -> None:
    args = three._parser().parse_args([])
    assert args.scoring_policy == "vehicle-estimate"
    with pytest.raises(SystemExit):
        three._parser().parse_args(["--scoring-policy", "sitl-truth"])


class _FakeTruthRecorder:
    def __init__(self, poi: PoiLocation, home_abs_alt_m: float) -> None:
        self.poi = poi
        self.home_abs_alt_m = home_abs_alt_m
        self.finalized = False
        self.track_path: Path | None = None

    def closure_ready(self) -> bool:
        return True

    def finalize(self) -> None:
        self.finalized = True

    def write_track(self, path: Path) -> None:
        self.track_path = path

    def verdict(self) -> dict[str, object]:
        return {"certification_error": None, "dist_3d_m": 0.05}


def _stub_run_case(monkeypatch, tmp_path: Path) -> dict[str, object]:
    """Stub every side-effectful seam under run_case; capture the calls."""
    calls: dict[str, object] = {"interval_requests": [], "fly_kwargs": None,
                                "verdict_kwargs": None, "recorders": []}

    monkeypatch.setattr(pn, "stop_own_stack", lambda *a, **k: None)
    monkeypatch.setattr(
        pn, "_start_swarm",
        lambda *a, **k: (None, SimpleNamespace(chat=1)),
    )
    monkeypatch.setattr(pn.ip, "sysids_for_chat", lambda chat: [121])
    monkeypatch.setattr(pn.ip, "monitor_device", lambda chat: "udp:monitor")
    monkeypatch.setattr(pn.ip, "companion_device", lambda sysid: "udp:companion")
    monkeypatch.setattr(pn, "upload_north_line", lambda *a, **k: None)
    monkeypatch.setattr(
        pn, "wait_for_heartbeat",
        lambda *a, **k: SimpleNamespace(close=lambda: None),
    )
    monkeypatch.setattr(pn, "request_coordinate_score_stream", lambda m: True)

    def _request_interval(master, message_id, rate_hz):
        calls["interval_requests"].append((message_id, rate_hz))
        return True

    monkeypatch.setattr(
        pn, "request_message_interval_stream", _request_interval
    )
    monkeypatch.setattr(pn, "download_mission", lambda m: object())
    monkeypatch.setattr(
        pn, "resolve_home_abs_alt_m", lambda m, timeout_s: 440.0
    )
    monkeypatch.setattr(
        pn, "resolve_poi_expectation",
        lambda *a, **k: SimpleNamespace(location=POI, mission_seq=3),
    )
    monkeypatch.setattr(pn, "sim_parameters", lambda args: [])
    monkeypatch.setattr(
        pn, "_source_identity", lambda: {"sha256": "0" * 64, "files": 1}
    )
    monkeypatch.setattr(pn, "final_approach_speed_plan", lambda *a, **k: None)
    monkeypatch.setattr(pn, "write_case_manifest", lambda *a, **k: None)
    monkeypatch.setattr(
        pn, "_launch_child", lambda *a, **k: SimpleNamespace(poll=lambda: 0)
    )
    monkeypatch.setattr(pn, "_wait_ready", lambda *a, **k: None)
    monkeypatch.setattr(pn, "_start_mission", lambda m: None)

    def _recorder(poi, home_abs_alt_m):
        recorder = _FakeTruthRecorder(poi, home_abs_alt_m)
        calls["recorders"].append(recorder)
        return recorder

    monkeypatch.setattr(pn, "TruthRecorder", _recorder)

    def _fly(master, process, case_dir, **kwargs):
        calls["fly_kwargs"] = kwargs
        return {"snap_3d_m": 0.4}

    monkeypatch.setattr(pn, "_fly", _fly)

    def _verdict(case_dir, **kwargs):
        calls["verdict_kwargs"] = kwargs
        return {"passed": True}

    monkeypatch.setattr(pn, "case_verdict", _verdict)
    return calls


def test_run_case_wires_truth_from_request_to_verdict(
    tmp_path: Path, monkeypatch
) -> None:
    calls = _stub_run_case(monkeypatch, tmp_path)
    args = pn._parser(scoring_policy=True).parse_args([])

    result = pn.run_case(
        Path("python"), tmp_path, speedup=1.0, repetition=0, args=args
    )

    assert result["passed"] is True
    # Without the --sim-cpa gate the module is never configured, but the
    # block still rides on every verdict so the field set stays
    # homogeneous (schema v3).
    assert result["sim_cpa"]["configuration"]["status"] == "not_attempted"
    assert calls["interval_requests"] == [
        (mavutil.mavlink.MAVLINK_MSG_ID_SIM_STATE, pn.TRUTH_SCORE_RATE_HZ)
    ]
    assert len(calls["recorders"]) == 1
    recorder = calls["recorders"][0]
    assert recorder.poi == POI
    assert recorder.home_abs_alt_m == 440.0
    assert calls["fly_kwargs"]["truth"] is recorder
    assert recorder.finalized
    assert recorder.track_path == (
        tmp_path / "speed-1-run-0" / "truth_track.csv"
    )
    verdict_kwargs = calls["verdict_kwargs"]
    assert verdict_kwargs["truth"] is recorder
    assert verdict_kwargs["truth_stream_accepted"] is True
    assert verdict_kwargs["scoring_policy"] == pn.SCORING_POLICY_SITL_TRUTH


def test_run_case_estimate_policy_never_touches_truth(
    tmp_path: Path, monkeypatch
) -> None:
    calls = _stub_run_case(monkeypatch, tmp_path)
    args = pn._parser(scoring_policy=True).parse_args(
        ["--scoring-policy", "vehicle-estimate"]
    )

    result = pn.run_case(
        Path("python"), tmp_path, speedup=1.0, repetition=0, args=args
    )

    assert result["passed"] is True
    assert calls["interval_requests"] == []
    assert calls["recorders"] == []
    assert calls["fly_kwargs"]["truth"] is None
    verdict_kwargs = calls["verdict_kwargs"]
    assert verdict_kwargs["truth"] is None
    assert verdict_kwargs["truth_stream_accepted"] is None
    assert verdict_kwargs["scoring_policy"] == "vehicle-estimate"


def test_run_case_late_failure_salvages_truth_and_the_observed_ack(
    tmp_path: Path, monkeypatch
) -> None:
    """A crash after the truth request must not erase what was observed
    (Codex round-5 finding 4): the real ACK rides along, the recorder's
    audit trail lands on disk, and the unscored row persists like a scored
    one."""
    calls = _stub_run_case(monkeypatch, tmp_path)

    def _explode(master, process, case_dir, **kwargs):
        raise RuntimeError("child died mid-flight")

    monkeypatch.setattr(pn, "_fly", _explode)
    args = pn._parser(scoring_policy=True).parse_args([])

    result = pn.run_case(
        Path("python"), tmp_path, speedup=1.0, repetition=0, args=args
    )

    assert result["passed"] is False and result["valid"] is False
    assert result["errors"] == ["RuntimeError: child died mid-flight"]
    assert result["truth_stream_acknowledged"] is True
    recorder = calls["recorders"][0]
    assert recorder.finalized
    assert recorder.track_path == (
        tmp_path / "speed-1-run-0" / "truth_track.csv"
    )
    assert result["truth"] == recorder.verdict()
    on_disk = json.loads(
        (tmp_path / "speed-1-run-0" / "verdict.json").read_text(
            encoding="utf-8"
        )
    )
    assert on_disk["truth_stream_acknowledged"] is True


def test_run_case_post_flight_failure_keeps_child_and_coordinate_evidence(
    tmp_path: Path, monkeypatch
) -> None:
    """A crash AFTER the flight (track write, verdict assembly) must keep
    the child's result and the coordinate scorer's CPA as audit data
    (Codex round-6 finding 11) -- the row stays unscored either way."""
    from dataclasses import asdict

    from scripts.eval_navigation_models import ClosestApproach

    calls = _stub_run_case(monkeypatch, tmp_path)
    closest = ClosestApproach(0.3, 0.25, 0.15)

    class _FakeScorer:
        def __init__(self, poi) -> None:
            self.result = closest
            self.sample_count = 42

    monkeypatch.setattr(pn, "CoordinateScorer", _FakeScorer)

    def _explode_verdict(case_dir, **kwargs):
        raise RuntimeError("validity walked off the end of the compact log")

    monkeypatch.setattr(pn, "case_verdict", _explode_verdict)
    args = pn._parser(scoring_policy=True).parse_args([])

    result = pn.run_case(
        Path("python"), tmp_path, speedup=1.0, repetition=0, args=args
    )

    assert result["passed"] is False and result["valid"] is False
    assert result["child"] == {"snap_3d_m": 0.4}
    assert result["coordinate"] == asdict(closest)
    assert result["coordinate_samples"] == 42
    assert result["truth_stream_acknowledged"] is True
    assert result["truth"] == calls["recorders"][0].verdict()
    on_disk = json.loads(
        (tmp_path / "speed-1-run-0" / "verdict.json").read_text(
            encoding="utf-8"
        )
    )
    assert on_disk["coordinate_samples"] == 42


def test_run_case_failure_before_the_request_reads_never_requested(
    tmp_path: Path, monkeypatch
) -> None:
    """A failure BEFORE the truth request keeps the pre-try None binding:
    'never requested' stays honest, and the unscored row still persists."""
    _stub_run_case(monkeypatch, tmp_path)

    def _explode(*args, **kwargs):
        raise RuntimeError("stack already dead")

    monkeypatch.setattr(pn, "_start_swarm", _explode)
    args = pn._parser(scoring_policy=True).parse_args([])

    result = pn.run_case(
        Path("python"), tmp_path, speedup=1.0, repetition=0, args=args
    )

    assert result["errors"] == ["RuntimeError: stack already dead"]
    assert result["truth_stream_acknowledged"] is None
    assert result["truth"] is None
    on_disk = json.loads(
        (tmp_path / "speed-1-run-0" / "verdict.json").read_text(
            encoding="utf-8"
        )
    )
    assert on_disk["truth_stream_acknowledged"] is None


def test_run_case_hands_the_case_manifest_its_parsed_arguments(
    tmp_path: Path, monkeypatch
) -> None:
    """D7b: the case's definition is made from the harness's own parse, so
    run_case hands over the arguments themselves, and the raw overrides are
    read from them rather than passed beside them."""
    _stub_run_case(monkeypatch, tmp_path)
    handed: dict[str, object] = {}

    def _manifest(case_dir, **fields):
        handed.update(fields, case_dir=case_dir)

    monkeypatch.setattr(pn, "write_case_manifest", _manifest)
    args = pn._parser(scoring_policy=True).parse_args([])

    pn.run_case(Path("python"), tmp_path, speedup=1.0, repetition=0, args=args)

    assert handed["args"] is args
    assert "sitl_params" not in handed
    assert handed["case_dir"] == tmp_path / "speed-1-run-0"


@pytest.mark.parametrize("traced", [False, True], ids=["untraced", "traced"])
def test_run_case_names_only_a_traced_case(
    tmp_path: Path, monkeypatch, traced: bool
) -> None:
    """Delivery step 6's review, from the harness to the real writer: with
    tracing off the harness still named, defined and hashed every case, so
    an untraced case.json gained three keys. The gate is read where the
    manifest is written, in the harness's own process."""
    real_writer = pn.write_case_manifest
    _stub_run_case(monkeypatch, tmp_path)
    monkeypatch.setattr(pn, "write_case_manifest", real_writer)
    monkeypatch.setattr(
        pn, "final_approach_speed_plan", lambda *a, **k: FinalApproachSpeedPlan()
    )
    monkeypatch.setattr(determinism_trace, "ENABLED", traced)
    args = pn._parser(scoring_policy=True).parse_args([])

    pn.run_case(Path("python"), tmp_path, speedup=1.0, repetition=0, args=args)

    written = tmp_path / "speed-1-run-0" / CASE_MANIFEST
    case = json.loads(written.read_text(encoding="utf-8"))
    named = {
        "case_name",
        "case_definition",
        "case_definition_sha256",
        "case_definition_error",
    }
    if not traced:
        assert not named & set(case), sorted(named & set(case))
        return
    assert named <= set(case), sorted(named - set(case))
    assert case["case_name"] == "speed-1-run-0"
    assert case["case_definition_sha256"] == definition_sha256(
        case_definition(args, 1.0)
    )
    assert case["case_definition_error"] is None


def test_every_argument_the_harness_parses_has_a_case_definition() -> None:
    """A flag whose parsed value JSON cannot encode would fail every case's
    manifest, so every one the harness parses is encoded here, as main()
    leaves them."""
    args = pn._parser(scoring_policy=True, sim_cpa=True).parse_args(
        ["--sitl-param", "SIM_RATE_HZ=1200"]
    )
    args.sim_cpa_mode = pn.resolve_mode(args.sim_cpa, args.sitl_param)

    definition = case_definition(args, 1.0)

    assert set(definition) == set(vars(args)) - {"python", "repetitions"}
    assert definition["speedups"] == 1.0
    assert json.loads(json.dumps(definition)) == definition
    assert len(definition_sha256(definition)) == 64


def _event_recorder(monkeypatch, calls: dict[str, object]) -> list[str]:
    """Wrap the ordered seams of the SIM_CPA lifecycle in one event log."""
    events: list[str] = []

    def _push(master, name, value):
        events.append(f"param:{name}")
        return True

    monkeypatch.setattr(pn, "set_param", _push)
    monkeypatch.setattr(
        pn, "sim_parameters", lambda args: [("SIM_RATE_HZ", 1000.0)]
    )
    monkeypatch.setattr(
        pn, "write_case_manifest",
        lambda *a, **k: events.append("manifest"),
    )
    monkeypatch.setattr(
        stage, "binding_pre_flight",
        lambda sysid: events.append("binding") or {
            "status": "armed", "logs_dir": "/x", "lastlog_pre": 3,
            "expected_number": 4,
        },
    )
    monkeypatch.setattr(
        stage, "arduplane_sha256",
        lambda: events.append("sha_before") or ("a" * 64, ""),
    )
    monkeypatch.setattr(
        stage, "configure_sim_cpa",
        lambda master, **k: events.append("configure") or {
            "mode": k["mode"], "status": "configured",
            "expected_poi": None, "requested_params": [],
            "acknowledged_params": [], "failed_param": None, "error": None,
        },
    )
    monkeypatch.setattr(
        pn, "_launch_child",
        lambda *a, **k: events.append("child")
        or SimpleNamespace(poll=lambda: 0),
    )
    monkeypatch.setattr(
        pn, "stop_own_stack",
        lambda *a, **k: events.append("stop_stack"),
    )
    monkeypatch.setattr(
        stage, "score_after_teardown",
        lambda case_dir, **k: events.append("module_score") or {
            "schema_version": 1, "authority": "cross_check_only",
            "configuration": k["configuration"],
            "artifact": {"status": "acquired"},
            "provenance": {},
            "evidence": {"status": "accepted"},
            "comparison": {"status": "compared", "disagreement": False},
        },
    )
    monkeypatch.setattr(
        pn, "persist_verdict",
        lambda case_dir, verdict: events.append("persist"),
    )
    return events


def test_run_case_module_scoring_runs_after_teardown_and_before_persist(
    tmp_path: Path, monkeypatch
) -> None:
    """The reconciled lifecycle order (verdict-module-score R3/R7): the
    manifest lands before any parameter push, the module is configured
    after the generic pushes and before the child, and the BIN is only
    scored after this chat's stack was stopped -- with the one atomic
    verdict write after module analysis."""
    calls = _stub_run_case(monkeypatch, tmp_path)
    events = _event_recorder(monkeypatch, calls)
    args = pn._parser(scoring_policy=True, sim_cpa=True).parse_args([])
    args.sim_cpa_mode = "auto"

    result = pn.run_case(
        Path("python"), tmp_path, speedup=1.0, repetition=0, args=args
    )

    assert result["sim_cpa"]["evidence"]["status"] == "accepted"
    # stop_own_stack is stubbed for the whole case, so the pre-case stop
    # appears first; everything after it must follow the reconciled order.
    assert events == [
        "stop_stack",  # pre-case cleanup
        "manifest",  # before ANY parameter push (config-stage legibility)
        "param:SIM_RATE_HZ",
        "binding",  # successor read before module configuration
        "sha_before",
        "configure",  # module armed last, before the child exists
        "child",
        "stop_stack",  # ordered teardown inside the try
        "module_score",  # only after the stack stopped
        "persist",  # the one atomic verdict write, after module analysis
    ]


def test_run_case_module_crash_still_persists_the_stream_verdict(
    tmp_path: Path, monkeypatch
) -> None:
    """Cross-check-only authority under failure: an internal module-scoring
    crash must land in the block, never cost the case its stream verdict
    or its persistence."""
    calls = _stub_run_case(monkeypatch, tmp_path)
    events = _event_recorder(monkeypatch, calls)

    def _explode(case_dir, **kwargs):
        raise RuntimeError("BIN parser fell over")

    monkeypatch.setattr(stage, "score_after_teardown", _explode)
    persisted: list[dict] = []
    monkeypatch.setattr(
        pn, "persist_verdict",
        lambda case_dir, verdict: persisted.append(verdict),
    )
    args = pn._parser(scoring_policy=True, sim_cpa=True).parse_args([])
    args.sim_cpa_mode = "auto"

    result = pn.run_case(
        Path("python"), tmp_path, speedup=1.0, repetition=0, args=args
    )

    assert result["passed"] is True
    assert persisted and persisted[0] is result
    evidence = result["sim_cpa"]["evidence"]
    assert evidence["errors"] and "BIN parser fell over" in evidence["errors"][0]
    # An explicit status, not just an error string, so the summary's
    # rejected counter sees the crash (90_review finding 4).
    assert evidence["status"] == "internal_error"
    assert result["sim_cpa"]["configuration"]["status"] == "configured"


def test_run_case_pre_flight_module_crash_never_aborts_the_flight(
    tmp_path: Path, monkeypatch
) -> None:
    """90_review finding 3: an unexpected crash in the cross-check's own
    pre-flight code must land as an internal_error configuration while the
    case flies on and scores stream-only -- cross-check-only authority
    means the observer can never kill the flight it observes."""
    calls = _stub_run_case(monkeypatch, tmp_path)
    events = _event_recorder(monkeypatch, calls)

    def _explode(sysid):
        raise RuntimeError("binding probe fell over")

    monkeypatch.setattr(stage, "binding_pre_flight", _explode)
    monkeypatch.setattr(
        stage, "score_after_teardown",
        lambda case_dir, **k: stage.sim_cpa_block(
            configuration=k["configuration"]
        ),
    )
    args = pn._parser(scoring_policy=True, sim_cpa=True).parse_args([])
    args.sim_cpa_mode = "auto"

    result = pn.run_case(
        Path("python"), tmp_path, speedup=1.0, repetition=0, args=args
    )

    assert result["passed"] is True
    assert "configure" not in events  # the crash preempted configuration
    configuration = result["sim_cpa"]["configuration"]
    assert configuration["status"] == "internal_error"
    assert "binding probe fell over" in configuration["error"]
    assert result["sim_cpa"]["artifact"]["status"] == "not_attempted"


def test_run_case_sim_cpa_off_mode_skips_binding_and_scoring(
    tmp_path: Path, monkeypatch
) -> None:
    """The baseline arm: off mode still records its configuration push but
    never reads LASTLOG, never hashes the binary, never parses a BIN."""
    calls = _stub_run_case(monkeypatch, tmp_path)
    events = _event_recorder(monkeypatch, calls)
    monkeypatch.setattr(
        stage, "configure_sim_cpa",
        lambda master, **k: events.append("configure") or {
            "mode": k["mode"], "status": "disabled_by_operator",
            "expected_poi": None, "requested_params": [],
            "acknowledged_params": [], "failed_param": None, "error": None,
        },
    )
    monkeypatch.setattr(
        stage, "score_after_teardown",
        lambda case_dir, **k: stage.sim_cpa_block(
            configuration=k["configuration"]
        ),
    )
    args = pn._parser(scoring_policy=True, sim_cpa=True).parse_args([])
    args.sim_cpa_mode = "off"

    result = pn.run_case(
        Path("python"), tmp_path, speedup=1.0, repetition=0, args=args
    )

    assert "binding" not in events and "sha_before" not in events
    block = result["sim_cpa"]
    assert block["configuration"]["status"] == "disabled_by_operator"
    assert block["artifact"]["status"] == "not_attempted"
    assert block["comparison"]["disagreement"] is None


class _ClosureTruth:
    """closure_ready() flips true after a set number of polls."""

    def __init__(self, ready_after: int) -> None:
        self.ready_after = ready_after
        self.polls = 0

    def closure_ready(self) -> bool:
        self.polls += 1
        return self.polls >= self.ready_after


def _fly_setup(tmp_path: Path, monkeypatch) -> tuple[list[dict], Path]:
    """Result file present from the start; record each drain's consumers."""
    drains: list[dict] = []

    def _drain(master, *, sysid, live_anchors, scorer, track,
               truth=None, truth_scoring_active=False):
        drains.append({
            "engaged": truth_scoring_active,
            "scorer": scorer is not None,
            "track": track is not None,
        })

    monkeypatch.setattr(pn, "drain_position_messages", _drain)
    (tmp_path / "scoring_active.marker").write_text("", encoding="utf-8")
    (tmp_path / "result.json").write_text(
        json.dumps({"snap_3d_m": 0.4}), encoding="utf-8"
    )
    return drains, tmp_path


def test_fly_keeps_draining_until_truth_closure(
    tmp_path: Path, monkeypatch
) -> None:
    drains, case_dir = _fly_setup(tmp_path, monkeypatch)
    truth = _ClosureTruth(ready_after=3)

    result = pn._fly(
        SimpleNamespace(),
        SimpleNamespace(poll=lambda: None),
        case_dir,
        sysid=121,
        scorer=SimpleNamespace(),
        timeout_s=5.0,
        track=SimpleNamespace(),
        truth=truth,
    )

    assert result == {"snap_3d_m": 0.4}
    assert truth.polls == 3
    # One drain in the main loop, then one per not-ready closure poll.
    assert len(drains) == 3
    assert all(d["engaged"] for d in drains)
    # The closure tail feeds ONLY the truth recorder: the child's result ends
    # the EKF scorer's and ground track's episode.
    assert drains[0]["scorer"] and drains[0]["track"]
    assert all(not d["scorer"] and not d["track"] for d in drains[1:])


def test_fly_closure_drain_gives_up_at_the_named_deadline(
    tmp_path: Path, monkeypatch
) -> None:
    drains, case_dir = _fly_setup(tmp_path, monkeypatch)
    monkeypatch.setattr(pn, "TRUTH_CLOSURE_DRAIN_TIMEOUT_S", 0.05)
    truth = _ClosureTruth(ready_after=10 ** 9)

    result = pn._fly(
        SimpleNamespace(),
        SimpleNamespace(poll=lambda: None),
        case_dir,
        sysid=121,
        scorer=SimpleNamespace(),
        timeout_s=5.0,
        track=None,
        truth=truth,
    )

    # The child's result still comes back; certification will report the
    # truncated closure -- the flight loop must not hang on it.
    assert result == {"snap_3d_m": 0.4}
    assert truth.polls >= 2


def test_fly_without_truth_returns_with_no_closure_drain(
    tmp_path: Path, monkeypatch
) -> None:
    drains, case_dir = _fly_setup(tmp_path, monkeypatch)

    result = pn._fly(
        SimpleNamespace(),
        SimpleNamespace(poll=lambda: None),
        case_dir,
        sysid=121,
        scorer=SimpleNamespace(),
        timeout_s=5.0,
        track=None,
        truth=None,
    )

    assert result == {"snap_3d_m": 0.4}
    assert len(drains) == 1
