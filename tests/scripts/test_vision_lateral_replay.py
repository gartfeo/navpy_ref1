import csv
import importlib.util
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / "scripts" / "vision_lateral_replay.py"

spec = importlib.util.spec_from_file_location("vision_lateral_replay", SCRIPT_PATH)
replay = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = replay
spec.loader.exec_module(replay)


def _write_compact(path: Path, rows: list[dict[str, object]]) -> None:
    fields = ["ts", "dist", "h_dist", "v_dist", "cmd_r", "cmd_p", "yaw_err", "pitch_err", "act_r", "act_p", "x_err", "y_err"]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in fields})


def test_compact_parser_skips_snap_and_non_numeric_rows(tmp_path):
    compact = tmp_path / "compact.csv"
    _write_compact(
        compact,
        [
            {"ts": "00:00:00.000", "dist": 100, "cmd_r": 1, "cmd_p": 0, "yaw_err": 2, "act_r": 0.5, "act_p": 0},
            {"ts": "00:00:00.100", "dist": "SNAP(VISION-NAV-PN)", "cmd_r": "", "yaw_err": "", "act_r": ""},
            {"ts": "00:00:00.250", "dist": 50, "cmd_r": -3, "cmd_p": 0, "yaw_err": -4, "act_r": -2.5, "act_p": 0},
        ],
    )

    rows = replay.parse_compact_csv(compact)

    assert len(rows) == 2
    assert rows[0].t_s == 0
    assert rows[1].t_s == 0.25
    assert rows[1].dist == 50
    assert rows[1].yaw_err == -4


def test_final_parser_reads_signed_lateral_and_resolves_compact(tmp_path):
    final_csv = tmp_path / "final.csv"
    compact = tmp_path / "logs" / "compact.csv"
    compact.parent.mkdir()
    compact.write_text("ts,dist,cmd_r,yaw_err,act_r\n", encoding="utf-8")
    with final_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=["name", "wind_speed", "wind_dir", "passed_accuracy", "dist_3d_m", "signed_lateral_m", "compact"],
        )
        writer.writeheader()
        writer.writerow(
            {
                "name": "case-a",
                "wind_speed": "8",
                "wind_dir": "210",
                "passed_accuracy": "False",
                "dist_3d_m": "2.0",
                "signed_lateral_m": "1.25",
                "compact": "logs/compact.csv",
            }
        )

    cases = replay.parse_final_csv(final_csv)

    assert len(cases) == 1
    assert cases[0].name == "case-a"
    assert cases[0].wind_speed == 8
    assert cases[0].wind_dir == 210
    assert cases[0].signed_lateral_m == 1.25
    assert cases[0].passed_accuracy is False
    assert cases[0].compact == compact.resolve()


def test_final_approach_features_integral_sign_flips_and_saturation():
    samples = [
        replay.CompactSample("0", 0.0, 400, 5, 0, 0, None, None),
        replay.CompactSample("1", 1.0, 250, 2, 45, 40, None, None),
        replay.CompactSample("2", 2.0, 100, -3, -45, -42, None, None),
        replay.CompactSample("3", 3.0, 50, 4, 10, 8, None, None),
    ]

    features = replay.compute_final_approach_features(samples, final_approach_dist_m=300)

    assert features.count == 3
    assert features.yaw_mean_deg == 1
    assert features.yaw_integral_deg_s == 0
    assert features.yaw_sign_flips == 2
    assert features.cmd_roll_saturation_frac == 2 / 3


def test_gate_extraction_returns_nearest_rows_and_candidate_values():
    samples = [
        replay.CompactSample("0", 0.0, 100, 1, 2, 3, None, None),
        replay.CompactSample("1", 1.0, 65, 4, 5, 6, None, None),
        replay.CompactSample("2", 2.0, 20, 7, 8, 9, None, None),
    ]
    gates = replay.gate_samples(samples, {"candidate": [10, 20, 30]}, [60, 25])

    assert [(gate, sample.dist, values["candidate"]) for gate, sample, values in gates] == [
        (60, 65, 20),
        (25, 20, 30),
    ]


def test_candidate_scoring_marks_needed_correction_sign():
    samples = [
        replay.CompactSample("0", 0.0, 200, 1, 10, 8, None, None),
        replay.CompactSample("1", 1.0, 100, 1, 10, 8, None, None),
        replay.CompactSample("2", 2.0, 50, 1, 10, 8, None, None),
    ]
    case = replay.FinalCase(
        name="miss-right",
        wind_speed=8,
        wind_dir=210,
        compact=Path("compact.csv"),
        signed_lateral_m=1.0,
        dist_3d_m=1.0,
        passed_accuracy=False,
    )

    diagnostics = replay.analyze_case(case, samples, gates_m=[100], lateral_deadband_m=0.1)
    visual_pos = next(score for score in diagnostics.candidates if score.name == "visual_p_pos")
    visual_neg = next(score for score in diagnostics.candidates if score.name == "visual_p_neg")

    assert diagnostics.need_roll_sign == -1
    assert visual_pos.sign_agree_frac == 0
    assert visual_pos.harmful_frac == 1
    assert visual_neg.sign_agree_frac == 1
    assert visual_neg.harmful_frac == 0
