"""Security, evidence-integrity, and decomposition regressions for certificates."""

from __future__ import annotations

import ast
import importlib
import inspect
import subprocess
from pathlib import Path

import pytest

from scripts import eval_certificate as cert
from scripts import eval_certificate_process as cert_process
from scripts import eval_certificate_source_streams as cert_streams
from scripts import eval_certificate_source_time as cert_source_time


ROOT = Path(__file__).resolve().parents[2]
MAX_MODULE_LINES = 300
MAX_CLASS_LINES = 200
MAX_DIRECT_METHODS = 15
MAX_FUNCTION_LINES = 150


def _completed(command: list[str], stdout: str) -> subprocess.CompletedProcess[str]:
    return subprocess.CompletedProcess(command, 0, stdout, "")


def test_wsl_repo_is_direct_argv_even_when_repo_contains_shell_source(monkeypatch):
    calls: list[list[str]] = []

    def fake_run(command, **_kwargs):
        argv = list(command)
        calls.append(argv)
        if argv[-2:] == ["printenv", "HOME"]:
            return _completed(argv, "/home/evaluator\n")
        if "rev-parse" in argv:
            return _completed(argv, "abc123\n")
        return _completed(argv, "")

    malicious = "~/ardupilot; touch /tmp/evaluator-owned"
    monkeypatch.setattr(cert_process.shutil, "which", lambda _name: "wsl.exe")
    monkeypatch.setattr(cert_process.subprocess, "run", fake_run)

    identity = cert.wsl_ardupilot_identity(repo=malicious)

    assert identity["available"] is True
    assert calls
    assert all(call[:2] == ["wsl.exe", "--exec"] for call in calls)
    assert all("bash" not in call and "sh" not in call for call in calls)
    resolved = "/home/evaluator/ardupilot; touch /tmp/evaluator-owned"
    assert any(resolved in call for call in calls)
    assert all(not any(resolved in arg and arg != resolved for arg in call) for call in calls)


def test_wsl_eeprom_root_is_direct_argv_even_when_it_contains_shell_source(monkeypatch):
    calls: list[list[str]] = []

    def fake_run(command, **_kwargs):
        argv = list(command)
        calls.append(argv)
        if argv[-2:] == ["printenv", "HOME"]:
            return _completed(argv, "/home/evaluator\n")
        if "stat" in argv:
            return _completed(argv, "8192 1750000000\n")
        if "sha256sum" in argv:
            return _completed(argv, f"{'a' * 64}  ignored\n")
        raise AssertionError(argv)

    malicious = "~/ardupilot; touch /tmp/evaluator-owned"
    monkeypatch.setattr(cert_process.shutil, "which", lambda _name: "wsl.exe")
    monkeypatch.setattr(cert_process.subprocess, "run", fake_run)

    identity = cert.eeprom_identity(121, root=malicious)

    assert identity["available"] is True
    assert calls
    assert all(call[:2] == ["wsl.exe", "--exec"] for call in calls)
    # ``stat -c`` is a direct utility option; only a shell executable is forbidden.
    assert all("bash" not in call and "sh" not in call for call in calls)
    resolved = "/home/evaluator/ardupilot; touch /tmp/evaluator-owned/121/eeprom.bin"
    assert any(resolved in call for call in calls)
    assert all(not any(resolved in arg and arg != resolved for arg in call) for call in calls)
    assert ["wsl.exe", "--exec", "stat", "-c", "%s %Y", "--", resolved] in calls
    assert ["wsl.exe", "--exec", "sha256sum", "--", resolved] in calls


def test_present_source_csv_read_error_is_not_reported_as_empty(monkeypatch, tmp_path):
    path = tmp_path / "pose_cadence_worker_uav_121_pid_7.csv"
    path.write_text("wall_start_s,source,obs_ts,src_now_s\n", encoding="utf-8")
    original_open = Path.open

    def fail_poi(candidate: Path, *args, **kwargs):
        if candidate == path:
            raise PermissionError("denied by regression test")
        return original_open(candidate, *args, **kwargs)

    monkeypatch.setattr(Path, "open", fail_poi)

    with pytest.raises(PermissionError, match="denied by regression test"):
        cert.source_time_summary(tmp_path)


def test_facade_reexports_exact_leaf_objects():
    expected = {
        "scripts.eval_certificate_profile": (
            "CERTIFICATE_SPEEDUP",
            "CERTIFICATE_WIND_PARAMS",
            "CERTIFICATE_READBACK_PARAMS",
            "CERTIFICATE_PARAM_TOLERANCE",
            "IDENTITY_PROBE_TIMEOUT_S",
        ),
        "scripts.eval_certificate_clock": (
            "MIN_RATE_SPAN_S",
            "SOURCE_RESET_BACKSTEP_S",
            "ClockRate",
            "ClockRateTracker",
        ),
        "scripts.eval_certificate_identity": (
            "MISSION_IDENTITY_FIELDS",
            "ParameterSnapshot",
            "navpy_identity",
            "mission_identity",
            "parameter_snapshot_identity",
            "autopilot_identity",
        ),
        "scripts.eval_certificate_wsl": (
            "wsl_ardupilot_identity",
            "eeprom_identity",
        ),
        "scripts.eval_certificate_statistics": (
            "MetricStats",
            "DistributionStats",
            "summarize",
            "percentile",
            "distribution",
        ),
        "scripts.eval_certificate_source_time": (
            "SOURCE_TIME_SUBDIR",
            "SOURCE_TIME_ENV",
            "SOURCE_TIME_FLUSH_GRACE_S",
            "source_time_summary",
            "source_time_row_metrics",
        ),
        "scripts.eval_certificate_policy": (
            "CERTIFICATE_RATE_TOLERANCE",
            "CERTIFICATE_IDENTITY_KEYS",
            "CERTIFICATE_METRIC_KEYS",
            "certificate_rate_error",
            "certificate_identity_errors",
            "certificate_consistency_errors",
            "certificate_summary",
        ),
    }
    for module_name, names in expected.items():
        leaf = importlib.import_module(module_name)
        for name in names:
            assert getattr(cert, name) is getattr(leaf, name), f"{name} is wrapped"


def _physical_lines(node: ast.AST) -> int:
    return int(node.end_lineno or node.lineno) - int(node.lineno) + 1


def test_certificate_modules_functions_and_classes_stay_within_budgets():
    violations: list[str] = []
    for path in sorted((ROOT / "scripts").glob("eval_certificate*.py")):
        lines = path.read_text(encoding="utf-8").splitlines()
        if len(lines) > MAX_MODULE_LINES:
            violations.append(f"{path.name}: module has {len(lines)} lines")
        tree = ast.parse("\n".join(lines), filename=str(path))
        for node in tree.body:
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                size = _physical_lines(node)
                if size > MAX_FUNCTION_LINES:
                    violations.append(f"{path.name}:{node.name} has {size} lines")
            elif isinstance(node, ast.ClassDef):
                size = _physical_lines(node)
                methods = sum(
                    isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef))
                    for item in node.body
                )
                class_fields = {
                    item.target.id
                    for item in node.body
                    if isinstance(item, ast.AnnAssign)
                    and isinstance(item.target, ast.Name)
                }
                owned_fields = set(class_fields)
                constructor_dependencies = 0
                for item in node.body:
                    if not isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef)):
                        continue
                    if item.name == "__init__":
                        constructor_dependencies = max(0, len(item.args.args) - 1)
                    for descendant in ast.walk(item):
                        target = None
                        if (
                            isinstance(descendant, ast.Assign)
                            and len(descendant.targets) == 1
                            and isinstance(descendant.targets[0], ast.Attribute)
                        ):
                            target = descendant.targets[0]
                        elif (
                            isinstance(descendant, ast.AnnAssign)
                            and isinstance(descendant.target, ast.Attribute)
                        ):
                            target = descendant.target
                        if (
                            target is not None
                            and isinstance(target.value, ast.Name)
                            and target.value.id == "self"
                        ):
                            owned_fields.add(target.attr)
                if size > MAX_CLASS_LINES:
                    violations.append(f"{path.name}:{node.name} has {size} lines")
                if methods > MAX_DIRECT_METHODS:
                    violations.append(f"{path.name}:{node.name} has {methods} methods")
                if len(owned_fields) > 12:
                    violations.append(
                        f"{path.name}:{node.name} owns {len(owned_fields)} fields"
                    )
                if constructor_dependencies > 12:
                    violations.append(
                        f"{path.name}:{node.name} has "
                        f"{constructor_dependencies} constructor dependencies"
                    )
    assert not violations, "\n".join(violations)


@pytest.mark.parametrize("value", [True, float("nan"), float("inf"), -float("inf")])
def test_clock_tracker_rejects_nonfinite_or_boolean_policy_values(value):
    with pytest.raises((TypeError, ValueError)):
        cert.ClockRateTracker(min_span_s=value)


@pytest.mark.parametrize("q", [True, -0.1, 100.1, float("nan"), float("inf")])
def test_percentile_rejects_invalid_policy_quantiles(q):
    with pytest.raises((TypeError, ValueError)):
        cert.percentile([1.0, 2.0], q)


def test_certificate_summary_does_not_truthify_a_false_string():
    summary = cert.certificate_summary([{"passed": "false"}])
    assert summary["passed_runs"] == 0
    assert summary["failed_runs"] == 1


@pytest.mark.parametrize("value", [True, float("nan"), float("inf"), -float("inf")])
def test_parameter_snapshot_rejects_invalid_numeric_evidence(value):
    with pytest.raises((TypeError, ValueError)):
        cert.ParameterSnapshot(values={"SIM_SPEEDUP": value})


def test_invalid_autopilot_timeout_is_rejected_before_vehicle_io():
    class Master:
        mav = type(
            "Mav",
            (),
            {"command_long_send": lambda *_args: pytest.fail("external I/O occurred")},
        )()

    with pytest.raises((TypeError, ValueError)):
        cert.autopilot_identity(Master(), timeout_s=float("nan"))


def test_present_source_csv_schema_error_is_explicit(tmp_path):
    path = tmp_path / "pose_cadence_worker_uav_121_pid_7.csv"
    path.write_text("source,obs_ts\nmeasured,1.0\n", encoding="utf-8")

    with pytest.raises(ValueError, match="missing source-time columns"):
        cert.source_time_summary(tmp_path)


def test_regex_matching_unsupported_source_stream_fails_before_read(
    monkeypatch, tmp_path
):
    path = tmp_path / "pose_cadence_future_stream_uav_121_pid_7.csv"
    path.write_text("untrusted,data\n1,2\n", encoding="utf-8")
    read_attempted = False

    def fail_if_read(*_args, **_kwargs):
        nonlocal read_attempted
        read_attempted = True
        raise AssertionError("unsupported stream must be rejected before read")

    monkeypatch.setattr(cert_source_time, "read_rows", fail_if_read)
    with pytest.raises(ValueError, match="unsupported source-time stream"):
        cert.source_time_summary(tmp_path)
    assert read_attempted is False


def test_identity_admissibility_requires_exact_true_flags():
    identity = {
        "navpy": {"dirty": False, "commit": "abc"},
        "parameters": {
            "complete": "false",
            "parameter_count": 3,
            "vehicle_param_count": 3,
        },
        "mission": {"item_count": 3, "sha256": "a" * 64},
        "autopilot": {"available": "false", "flight_sw_version": 1},
    }
    errors = cert.certificate_identity_errors(identity)
    assert any("parameter snapshot incomplete" in error for error in errors)
    assert any("firmware identity unavailable" in error for error in errors)


def test_passing_bit_alone_cannot_make_a_certificate_valid():
    summary = cert.certificate_summary([{"passed": True}])
    assert summary["valid"] is False
    assert summary["invalid_runs"] == 1
    assert any("admissibility" in reason for reason in summary["invalid_reasons"])
    assert summary["consistency_errors"]


def test_existing_source_time_path_that_is_not_a_directory_fails(tmp_path):
    path = tmp_path / "source-time"
    path.write_text("not a directory", encoding="utf-8")
    with pytest.raises(NotADirectoryError):
        cert.source_time_summary(path)


def test_source_time_summary_filters_one_vehicle_and_process_file_set(tmp_path):
    for sys_id, pid in ((1, 10), (2, 20)):
        (tmp_path / f"pose_cadence_attitude_arrival_uav_{sys_id}_pid_{pid}.csv").write_text(
            "wall_s,boot_ms\n100.0,1000\n100.1,1020\n",
            encoding="utf-8",
        )
    selected = "pose_cadence_attitude_arrival_uav_2_pid_20.csv"

    summary = cert.source_time_summary(
        tmp_path,
        sys_id=2,
        include_files=(selected,),
    )

    assert summary["files"] == 1
    assert summary["file_names"] == [selected]
    assert summary["streams"]["attitude_arrival"]["rows"] == 2


@pytest.mark.parametrize("outcome", ["emitted", "emitted_reanchor"])
def test_emitted_detector_row_requires_frame_timestamp(tmp_path, outcome):
    path = tmp_path / "pose_cadence_detector_pose_uav_121_pid_7.csv"
    path.write_text(
        f"wall_s,boot_s,outcome,frame_ts\n100.0,1.0,{outcome},\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="frame_ts"):
        cert.source_time_summary(tmp_path)


def test_nonemitted_detector_row_rejects_frame_timestamp(tmp_path):
    path = tmp_path / "pose_cadence_detector_pose_uav_121_pid_7.csv"
    path.write_text(
        "wall_s,boot_s,outcome,frame_ts\n100.0,1.0,no_attitude,1.0\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="non-emitted.*frame_ts"):
        cert.source_time_summary(tmp_path)


def test_one_emitted_outcome_set_drives_detector_parser_and_metrics(tmp_path):
    assert cert_streams.EMITTED_OUTCOMES == frozenset(
        {"emitted", "emitted_reanchor"}
    )
    path = tmp_path / "pose_cadence_detector_pose_uav_121_pid_7.csv"
    path.write_text(
        "wall_s,boot_s,outcome,frame_ts\n"
        "100.0,1.0,emitted,1.0\n"
        "100.1,1.1,emitted_reanchor,1.1\n"
        "100.2,,no_attitude,\n",
        encoding="utf-8",
    )
    metrics = cert.source_time_row_metrics(cert.source_time_summary(tmp_path))
    assert metrics["frame_emitted_count"] == 2
    assert metrics["frame_reanchor_count"] == 1


def test_worker_source_marker_is_optional_but_cannot_claim_nonmeasured(tmp_path):
    path = tmp_path / "pose_cadence_worker_uav_121_pid_7.csv"
    path.write_text(
        "wall_start_s,exec_ms,obs_ts,src_now_s\n100.0,2.0,5.0,5.03\n",
        encoding="utf-8",
    )
    worker = cert.source_time_summary(tmp_path)["streams"]["worker"]
    assert worker["rows"] == 1
    assert worker["outcomes"] == {"fresh": 1}
    assert "sources" not in worker

    path.write_text(
        "wall_start_s,exec_ms,obs_ts,src_now_s,source\n"
        "100.0,2.0,5.0,5.03,prediction\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="must be 'measured'"):
        cert.source_time_summary(tmp_path)


def test_worker_rejects_unknown_command_outcome(tmp_path):
    path = tmp_path / "pose_cadence_worker_uav_121_pid_7.csv"
    path.write_text(
        "wall_start_s,exec_ms,obs_ts,src_now_s,source,outcome\n"
        "100.0,2.0,5.0,5.03,measured,predicted\n",
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="must be 'fresh' or 'held'"):
        cert.source_time_summary(tmp_path)


def test_clock_samples_and_autopilot_probe_use_narrow_annotations():
    add_parameters = inspect.signature(cert.ClockRateTracker.add).parameters
    assert add_parameters["source_time_s"].annotation != inspect.Signature.empty
    assert add_parameters["wall_time_s"].annotation != inspect.Signature.empty
    master_annotation = inspect.signature(cert.autopilot_identity).parameters[
        "master"
    ].annotation
    assert master_annotation != inspect.Signature.empty
    assert "AutopilotVersionProbe" in str(master_annotation)


def _valid_summary_row() -> dict:
    return {
        "index": 0,
        "repetition": 1,
        "passed": True,
        "dist_3d_m": 0.2,
        "coordinate_dist_3d_m": 0.21,
        "speedup": 10.0,
        "measured_clock_rate": 10.0,
        "certificate_invalid_reason": "",
        "error": "",
        "identity_navpy_commit": "abc",
        "identity_mission_sha": "a" * 64,
        "identity_parameters_sha": "b" * 64,
        "identity_autopilot": "fw",
    }


@pytest.mark.parametrize(
    ("field", "value", "reason"),
    [
        ("dist_3d_m", None, "SNAP"),
        ("coordinate_dist_3d_m", float("nan"), "coordinate"),
        ("speedup", 0.0, "speedup"),
        ("speedup", float("inf"), "speedup"),
        ("measured_clock_rate", None, "not measured"),
        ("measured_clock_rate", 7.0, "outside"),
        ("identity_mission_sha", "", "identity_mission_sha"),
    ],
)
def test_summary_derives_fail_closed_evidence_errors(field, value, reason):
    row = _valid_summary_row()
    row[field] = value
    summary = cert.certificate_summary([row])
    assert summary["valid"] is False
    assert summary["invalid_runs"] == 1
    assert any(reason in error for error in summary["per_run"][0]["evidence_errors"])


def test_summary_requires_explicit_admissibility_and_error_results():
    row = _valid_summary_row()
    del row["certificate_invalid_reason"]
    del row["error"]
    evidence_errors = cert.certificate_summary([row])["per_run"][0]["evidence_errors"]
    assert any("admissibility" in error for error in evidence_errors)
    assert any("error result" in error for error in evidence_errors)


@pytest.mark.parametrize("field", ["certificate_invalid_reason", "error"])
@pytest.mark.parametrize("value", [None, False, 0])
def test_summary_requires_exact_string_result_markers(field, value):
    row = _valid_summary_row()
    row[field] = value
    summary = cert.certificate_summary([row])
    assert summary["valid"] is False
    assert any(
        "string" in error
        for error in summary["per_run"][0]["evidence_errors"]
    )


def test_complete_failed_run_is_failed_but_not_evidence_invalid():
    row = _valid_summary_row()
    row["passed"] = False
    summary = cert.certificate_summary([row])
    assert summary["failed_runs"] == 1
    assert summary["invalid_runs"] == 0
    assert summary["valid"] is False
    assert summary["per_run"][0]["evidence_errors"] == []


def test_identity_requires_mission_parameter_digests_and_meaningful_firmware():
    identity = {
        "navpy": {"dirty": False, "commit": "abc"},
        "mission": {"item_count": 0, "sha256": ""},
        "parameters": {
            "complete": True,
            "parameter_count": 3,
            "vehicle_param_count": 3,
            "sha256": "",
        },
        "autopilot": {
            "available": True,
            "flight_sw_version": None,
            "board_version": None,
            "flight_custom_version": None,
            "os_custom_version": None,
        },
    }
    errors = cert.certificate_identity_errors(identity)
    assert any("mission identity" in error for error in errors)
    assert any("parameter snapshot digest" in error for error in errors)
    assert any("firmware build identifier" in error for error in errors)


@pytest.mark.parametrize(
    "autopilot",
    [
        {
            "available": True,
            "board_version": 1,
            "flight_custom_version": "1234567890abcdef",
        },
        {
            "available": True,
            "flight_sw_version": 0x04050000,
            "board_version": 1,
            "flight_custom_version": "0000000000000000",
        },
    ],
)
def test_identity_rejects_non_build_firmware_fields(autopilot):
    identity = {
        "navpy": {"dirty": False, "commit": "abc"},
        "mission": {"item_count": 3, "sha256": "a" * 64},
        "parameters": {"complete": True, "sha256": "b" * 64},
        "autopilot": autopilot,
    }
    errors = cert.certificate_identity_errors(identity)
    assert any("firmware build identifier" in error for error in errors)


@pytest.mark.parametrize(
    "labels",
    [
        ((0, 1), (0, 1)),
        ((0, 1), (0, 3)),
        ((0, "1"), (0, 2)),
        ((0, 1), (1, 2)),
    ],
)
def test_summary_rejects_non_independent_repetition_labels(labels):
    rows = []
    for index, repetition in labels:
        row = _valid_summary_row()
        row["index"] = index
        row["repetition"] = repetition
        rows.append(row)
    summary = cert.certificate_summary(rows)
    assert summary["valid"] is False
    assert any("repetition" in error or "index" in error for error in summary["consistency_errors"])
